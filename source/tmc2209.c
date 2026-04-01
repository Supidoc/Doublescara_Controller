/************************************************************
 * @file    tmc2209.c
 * @brief   Implementation file for TMC2209 driver interface module
 * @author  dg
 * @date    25 Feb 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/

#include "tmc2209.h"
#include "log.h"
#include "stdio.h"
#include "math.h"
#include "task_helpers.h"

/************************************
 *     Private Macros / Defines    *
 ************************************/

#define TMC_READ_PACKAGE_SIZE 4
#define TMC_WRITE_PACKAGE_SIZE 8

#define TMC_GCONF_ADDR 0x00
#define TMC_GSTAT_ADDR 0x01
#define TMC_IFCNT_ADDR 0x02
#define TMC_CHOPCONF_ADDR 0x6c
#define TMC_IHOLD_IRUN_ADDR 0x10

/***************************
 *     Private Typedefs     *
 ***************************/

typedef enum _TMC_CommandType
{
    TMC_CMD_DEFAULT_INIT,
    TMC_CMD_SET_MICROSTEPPING,
    TMC_CMD_SET_IHOLD_DIVIDER,
    TMC_CMD_SET_IRUN_DIVIDER
} TMC_CommandType_t;

typedef struct _TMC_HandleImpl
{
    uart_rtos_handle_t* uartRTOSHandle;
    uart_handle_t*      uartHandle;
    TMC_SerialAddress_t serialAdress;
    uint8_t             transmissionCount;
    TMC_MICROSTEPPING_t microstepping;
    uint8_t             iholdDivider;
    uint8_t             irunDivider;
    const char*         label;
} TMC_HandleImpl_t;

typedef struct _TMC_CommandQueueItem
{
    TMC_CommandType_t commandType;
    TMC_Handle_t      handle;
    TickType_t        deadline;
    THE_CmdHandle_t   cmdHandle; // NEW: async completion handle
    union
    {
        struct
        {
            TMC_MICROSTEPPING_t microstepping;
        } setMicrostepping;
        struct
        {
            uint8_t ihold;
        } setIholdDivider;
        struct
        {
            uint8_t irun;
        } setIrunDivider;
        struct
        {
            TMC_Config_t config;
        } initHandle;
    } data;
} TMC_CommandQueueItem_t;

typedef struct _TMC_HandleArrayItem
{
    TMC_HandleImpl_t handle;
    uint8_t          used;
} TMC_HandleArrayItem_t;

/*****************************************
 *     Private Function Declarations     *
 *****************************************/

static inline void create_read_datagram(TMC_Handle_t handle, uint8_t reg_addr,
                                        TMC_Read_Datagram_t* datagram);
static inline void create_write_datagram(TMC_Handle_t handle, uint8_t reg_addr, uint32_t data,
                                         TMC_Write_Datagram_t* datagram);
static inline void build_read_package(TMC_Read_Datagram_t* datagram, uint8_t* package);
static inline void build_write_package(TMC_Write_Datagram_t* datagram, uint8_t* package);
static inline void calc_CRC(uint8_t* datagram, size_t datagramLength);
static void        process(void);
static uint8_t     dump_rx_buffer(TMC_Handle_t handle, uint8_t dumpCount, TickType_t deadline);
static status_t    read_transmissioncount(TMC_Handle_t handle, TickType_t deadline);
static status_t    init_handle(TMC_Config_t config, TickType_t deadline);
static status_t    set_microstepping(TMC_Handle_t handle, TMC_MICROSTEPPING_t microstepping,
                                     TickType_t deadline);
static status_t    set_ihold_divider(TMC_Handle_t handle, uint8_t ihold, TickType_t deadline);
static status_t    set_irun_divider(TMC_Handle_t handle, uint8_t irun, TickType_t deadline);
static status_t    read(TMC_Handle_t handle, uint8_t regAddr, uint32_t* data, TickType_t deadline);
static status_t    write(TMC_Handle_t handle, uint8_t regAddr, uint32_t* data, TickType_t deadline);
static status_t    set_double_edge_step_pulse(TMC_Handle_t handle, uint8_t enabled,
                                              TickType_t deadline);
static status_t    free_handle(TMC_Handle_t handle);
static status_t    send_cmd_async(TMC_CommandQueueItem_t* queueItem, TickType_t deadline,
                                  THE_CmdHandle_t* cmdHandle);

/****************************
 *     Public Variables     *
 ****************************/

/*****************************
 *     Private Variables     *
 *****************************/

static uint8_t transmit_read_package[TMC_READ_PACKAGE_SIZE];
static uint8_t transmit_response_package[TMC_WRITE_PACKAGE_SIZE];
static uint8_t transmit_write_package[TMC_WRITE_PACKAGE_SIZE];

static TMC_HandleArrayItem_t tmcHandleArray[TMC_MAX_HANDLE_COUNT];

QueueHandle_t cmdQueue = NULL;

static THE_CmdHandleImpl_t cmdHandles[TMC_MAX_CMD_HANDLE_COUNT];

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

status_t TMC_init(void)
{
    cmdQueue = xQueueCreate(TMC_COMMAND_QUEUE_LENGTH, sizeof(TMC_CommandQueueItem_t));
    if (cmdQueue == NULL)
    {
        return kStatus_Fail;
    }
    vQueueAddToRegistry(cmdQueue, "TMC Command Queue");

    THE_init_cmd_handles(cmdHandles, TMC_MAX_CMD_HANDLE_COUNT); // NEW
    return kStatus_Success;
}

void TMC_task(void* pvParameters)
{
    LOG_INFO("Started TMC2209 Task");
    for (;;)
    {
        process();
    }
}

status_t TMC_init_handle_async(TMC_Config_t config, TickType_t deadline, THE_CmdHandle_t* cmdHandle)
{
    if (cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    TMC_CommandQueueItem_t queueItem = {0};
    queueItem.handle                 = NULL;
    queueItem.commandType            = TMC_CMD_DEFAULT_INIT;
    queueItem.deadline               = deadline;
    queueItem.data.initHandle.config = config;

    return send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t TMC_set_microstepping_async(TMC_Handle_t handle, TMC_MICROSTEPPING_t microstepping,
                                     TickType_t deadline, THE_CmdHandle_t* cmdHandle)
{
    if (handle == NULL || cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    TMC_CommandQueueItem_t queueItem              = {0};
    queueItem.handle                              = handle;
    queueItem.commandType                         = TMC_CMD_SET_MICROSTEPPING;
    queueItem.data.setMicrostepping.microstepping = microstepping;
    queueItem.deadline                            = deadline;

    return send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t TMC_microstepping_value_to_enum(uint8_t value, TMC_MICROSTEPPING_t* microstepping)
{
    if (microstepping == NULL)
    {
        return kStatus_Fail;
    }

    switch (value)
    {
        case 0b0000:
            *microstepping = TMC_MS_256;
            return kStatus_Success;
        case 0b0001:
            *microstepping = TMC_MS_128;
            return kStatus_Success;
        case 0b0010:
            *microstepping = TMC_MS_64;
            return kStatus_Success;
        case 0b0011:
            *microstepping = TMC_MS_32;
            return kStatus_Success;
        case 0b0100:
            *microstepping = TMC_MS_16;
            return kStatus_Success;
        case 0b0101:
            *microstepping = TMC_MS_8;
            return kStatus_Success;
        case 0b0110:
            *microstepping = TMC_MS_4;
            return kStatus_Success;
        case 0b0111:
            *microstepping = TMC_MS_2;
            return kStatus_Success;
        case 0b1000:
            *microstepping = TMC_MS_FULL;
            return kStatus_Success;
        default:
            return kStatus_Fail;
    }
}

status_t TMC_microstepping_uint_to_enum(uint16_t value, TMC_MICROSTEPPING_t* microstepping)
{
    if (microstepping == NULL)
    {
        return kStatus_Fail;
    }

    switch (value)
    {
        case 256:
            *microstepping = TMC_MS_256;
            return kStatus_Success;
        case 128:
            *microstepping = TMC_MS_128;
            return kStatus_Success;
        case 64:
            *microstepping = TMC_MS_64;
            return kStatus_Success;
        case 32:
            *microstepping = TMC_MS_32;
            return kStatus_Success;
        case 16:
            *microstepping = TMC_MS_16;
            return kStatus_Success;
        case 8:
            *microstepping = TMC_MS_8;
            return kStatus_Success;
        case 4:
            *microstepping = TMC_MS_4;
            return kStatus_Success;
        case 2:
            *microstepping = TMC_MS_2;
            return kStatus_Success;
        case 1:
            *microstepping = TMC_MS_FULL;
            return kStatus_Success;
        default:
            return kStatus_Fail;
    }
}

status_t TMC_set_ihold_divider_async(TMC_Handle_t handle, uint8_t ihold, TickType_t deadline,
                                     THE_CmdHandle_t* cmdHandle)
{
    if (handle == NULL || cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    TMC_CommandQueueItem_t queueItem     = {0};
    queueItem.handle                     = handle;
    queueItem.commandType                = TMC_CMD_SET_IHOLD_DIVIDER;
    queueItem.data.setIholdDivider.ihold = ihold;
    queueItem.deadline                   = deadline;

    return send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t TMC_set_irun_divider_async(TMC_Handle_t handle, uint8_t irun, TickType_t deadline,
                                    THE_CmdHandle_t* cmdHandle)
{
    if (handle == NULL || cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    TMC_CommandQueueItem_t queueItem   = {0};
    queueItem.handle                   = handle;
    queueItem.commandType              = TMC_CMD_SET_IRUN_DIVIDER;
    queueItem.data.setIrunDivider.irun = irun;
    queueItem.deadline                 = deadline;

    return send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t TMC_current_to_divider(float desired_current, TMC_RoundingMode_t rounding,
                                uint8_t* divider)
{

    float reference_current = 1.767766953f; // Reference current for TMC2209 at 100% (1.77A)
                                            // calculated from datasheet Rev. 1.09 p. 53

    /* Validate inputs */
    if (divider == NULL)
    {
        return kStatus_Fail;
    }

    if (desired_current <= 0.0f)
    {
        return kStatus_Fail;
    }

    if (desired_current > reference_current)
    {
        return kStatus_Fail;
    }

    float raw_divider = (desired_current * 32 / reference_current) - 1.0f;

    /* Apply rounding mode */
    uint8_t rounded_divider;
    switch (rounding)
    {
        case TMC_ROUND_FLOOR:
            rounded_divider = (uint8_t)floorf(raw_divider);
            break;
        case TMC_ROUND_CEIL:
            rounded_divider = (uint8_t)ceilf(raw_divider);
            break;
        case TMC_ROUND_NEAREST:
            rounded_divider = (uint8_t)roundf(raw_divider);
            break;
        default:
            return kStatus_Fail;
    }

    /* Validate divider is within valid range [0, 31] for 3-bit TMC2209 register */
    if (rounded_divider > 31)
    {
        return kStatus_Fail;
    }
    if (rounded_divider < 0)
    {
        rounded_divider = 0;
    }

    *divider = rounded_divider;
    return kStatus_Success;
}

status_t TMC_get_default_config(TMC_Config_t* config)
{
    if (config == NULL)
    {
        return kStatus_Fail;
    }

    config->uartRTOSHandle = &UART1_rtos_handle;
    config->uartHandle     = &UART1_uart_handle;
    config->serialAdress   = 0;
    config->label          = "TMC2209";
    config->microstepping  = TMC_MS_16; // Default to 1/16 microstepping
    config->iholdDivider   = 16;        // Default IHOLD divider (50% of max current)
    config->irunDivider    = 16;        // Default IRUN divider (50% of max current)

    return kStatus_Success;
}

status_t TMC_get_handle_by_label(const char* label, TMC_Handle_t* handle)
{
    if (label == NULL || handle == NULL)
    {
        return kStatus_Fail;
    }

    for (size_t i = 0; i < TMC_MAX_HANDLE_COUNT; i++)
    {
        if (tmcHandleArray[i].used && strcmp(tmcHandleArray[i].handle.label, label) == 0)
        {
            *handle = &tmcHandleArray[i].handle;
            return kStatus_Success;
        }
    }

    return kStatus_Fail; // No matching handle found
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/

static void process(void)
{
    TMC_CommandQueueItem_t queueItem = {0};
    BaseType_t             status;
    status = xQueueReceive(cmdQueue, &queueItem, pdMS_TO_TICKS(10));
    if (status != pdPASS)
    {
        return;
    }
    status_t cmdStatus = kStatus_Success;
    switch (queueItem.commandType)
    {
        case TMC_CMD_DEFAULT_INIT:
            cmdStatus = init_handle(queueItem.data.initHandle.config, queueItem.deadline);
            break;
        case TMC_CMD_SET_MICROSTEPPING:
            cmdStatus =
                set_microstepping(queueItem.handle, queueItem.data.setMicrostepping.microstepping,
                                  queueItem.deadline);
            break;
        case TMC_CMD_SET_IHOLD_DIVIDER:
            cmdStatus = set_ihold_divider(queueItem.handle, queueItem.data.setIholdDivider.ihold,
                                          queueItem.deadline);
            break;
        case TMC_CMD_SET_IRUN_DIVIDER:
            cmdStatus = set_irun_divider(queueItem.handle, queueItem.data.setIrunDivider.irun,
                                         queueItem.deadline);
            break;
        default:
            cmdStatus = kStatus_Fail;
            break;
    }

    if (queueItem.cmdHandle != NULL)
    {
        if (queueItem.deadline <= xTaskGetTickCount())
        {
            THE_notify_task_timeout(queueItem.cmdHandle);
        }
        else if (cmdStatus == kStatus_Success)
        {
            THE_notify_task_success(queueItem.cmdHandle);
        }
        else
        {
            THE_notify_task_failure(queueItem.cmdHandle);
        }

        THE_remove_cmd_handle_ref(queueItem.cmdHandle);
    }
}

static status_t send_cmd_async(TMC_CommandQueueItem_t* queueItem, TickType_t deadline,
                               THE_CmdHandle_t* cmdHandle)
{
    if (queueItem == NULL)
    {
        return kStatus_Fail;
    }
    THE_CmdHandle_t internaleCmdHandle = NULL;

    status_t allocStatus =
        THE_get_cmd_handle(&internaleCmdHandle, cmdHandles, TMC_MAX_CMD_HANDLE_COUNT);
    if (allocStatus != kStatus_Success)
    {
        return kStatus_Fail;
    }
    if (cmdHandle != NULL)
    {
        THE_add_cmd_handle_ref(internaleCmdHandle);
    }

    queueItem->cmdHandle = internaleCmdHandle;

    status_t sendStatus = THE_send_cmd(cmdQueue, queueItem, deadline, internaleCmdHandle);
    if (sendStatus != kStatus_Success)
    {
    	if(cmdHandle != NULL){
            THE_remove_cmd_handle_ref(*cmdHandle);
            *cmdHandle = NULL;
    	}
        return kStatus_Fail;
    }
    if(cmdHandle != NULL){
    *cmdHandle = internaleCmdHandle;
    }
    return kStatus_Success;
}

static status_t set_ihold_divider(TMC_Handle_t handle, uint8_t ihold, TickType_t deadline)
{

    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    uint32_t    ihold_irun;
    static char logMsg[60];

    if (read(handle, TMC_IHOLD_IRUN_ADDR, &ihold_irun, deadline) != kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to read IHOLD register - driver communication error", handle->label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }

    ihold_irun &= ~(0b11111 << 0);        // Clear IHOLD bits
    ihold_irun |= (ihold & 0b11111) << 0; // Set new IHOLD bits

    if (write(handle, TMC_IHOLD_IRUN_ADDR, &ihold_irun, deadline) != kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to write IHOLD register - driver communication error", handle->label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }

    snprintf(logMsg, sizeof(logMsg), "[%s] Hold current divider set to %d", handle->label, ihold);
    LOG_INFO(logMsg);

    return kStatus_Success;
}

static status_t set_irun_divider(TMC_Handle_t handle, uint8_t irun, TickType_t deadline)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    uint32_t    ihold_irun;
    static char logMsg[60];

    if (read(handle, TMC_IHOLD_IRUN_ADDR, &ihold_irun, deadline) != kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to read IRUN register - driver communication error", handle->label);
        LOG_ERROR(logMsg);

        return kStatus_Fail;
    }

    ihold_irun &= ~(0b11111 << 8);       // Clear IRUN bits
    ihold_irun |= (irun & 0b11111) << 8; // Set new IRUN bits

    if (write(handle, TMC_IHOLD_IRUN_ADDR, &ihold_irun, deadline) != kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to write IRUN register - driver communication error", handle->label);
        LOG_ERROR(logMsg);

        return kStatus_Fail;
    }

    snprintf(logMsg, sizeof(logMsg), "[%s] Run current divider set to %d", handle->label, irun);
    LOG_INFO(logMsg);

    return kStatus_Success;
}

static status_t set_microstepping(TMC_Handle_t handle, TMC_MICROSTEPPING_t microstepping,
                                  TickType_t deadline)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    static char logMsg[60];

    uint32_t chopConf;
    read(handle, TMC_CHOPCONF_ADDR, &chopConf, deadline);
    chopConf &= ~(0b1111 << 24);
    chopConf |= microstepping << 24;

    if (kStatus_Success != write(handle, TMC_CHOPCONF_ADDR, &chopConf, deadline))
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to write Microstepping to CHOPCONF register",
                 handle->label);
        LOG_ERROR(logMsg);

        return kStatus_Fail;
    }

    snprintf(logMsg, sizeof(logMsg), "[%s] Microstepping set", handle->label);
    LOG_INFO(logMsg);

    return kStatus_Success;
}

static status_t set_double_edge_step_pulse(TMC_Handle_t handle, uint8_t enabled,
                                           TickType_t deadline)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    uint32_t chopConf;
    read(handle, TMC_CHOPCONF_ADDR, &chopConf, deadline);
    chopConf &= ~(0b1 << 29);
    chopConf |= (enabled && 0b1) << 29;
    char logMsg[60];

    if (kStatus_Success != write(handle, TMC_CHOPCONF_ADDR, &chopConf, deadline))
    {
        snprintf(logMsg, sizeof(logMsg),
                 "[%s] Failed to write Double Edge Pulse to CHOPCONF register", handle->label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }
    return kStatus_Success;
}

static status_t init_handle(TMC_Config_t config, TickType_t deadline)
{
    TMC_Handle_t handle = NULL;

    for (size_t i = 0; i < TMC_MAX_HANDLE_COUNT; i++)
    {
        if (tmcHandleArray[i].used == 0)
        {
            tmcHandleArray[i].used = 1;
            handle                 = &tmcHandleArray[i].handle;
            break;
        }
    }

    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    handle->uartRTOSHandle = config.uartRTOSHandle;
    handle->uartHandle     = config.uartHandle;
    handle->serialAdress   = config.serialAdress;
    handle->microstepping  = config.microstepping;
    handle->iholdDivider   = config.iholdDivider;
    handle->irunDivider    = config.irunDivider;
    handle->label          = config.label;

    static char logMsg[60];
    snprintf(logMsg, sizeof(logMsg), "[%s] Initializing handle with config", handle->label);
    LOG_INFO(logMsg);

    if (read_transmissioncount(handle, deadline) != kStatus_Success)
    {
        free_handle(handle);

        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to read transmission count - cannot communicate with driver", handle->label);
        LOG_FATAL(logMsg);

        return kStatus_Fail;
    }

    // gconf default values
    uint32_t gconf = 0;
    gconf |= 0b1;      // use external VREF
    gconf |= 0b0 << 1; // use external sense resistors
    gconf |= 0b0 << 2; // use StealthChopfp PWM
    gconf |= 0b0 << 3; // Do not invert Shaft
    gconf |= 0b0 << 4; // Index Pin shows first microstep position of sequencer
    gconf |= 0b0 << 5; // Output Index of first microstep
    gconf |= 0b1 << 6; // PDN_UART input disabled, use uart
    gconf |= 0b1 << 7; // Microstep resolution set by register
    gconf |= 0b0 << 8; // No filtering of STEP pulses

    if (kStatus_Success != write(handle, TMC_GCONF_ADDR, &gconf, deadline))
    {
        free_handle(handle);

        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to write GCONF register - initialization failure", handle->label);
        LOG_FATAL(logMsg);

        return kStatus_Fail;
    }

    // reset GSTAT
    uint32_t gstat = 0b111;

    if (kStatus_Success != write(handle, TMC_GSTAT_ADDR, &gstat, deadline))
    {
        free_handle(handle);

        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to write GSTAT register - initialization failure", handle->label);
        LOG_FATAL(logMsg);

        return kStatus_Fail;
    }
    if (set_microstepping(handle, config.microstepping, deadline) != kStatus_Success)
    {
        free_handle(handle);

        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to set microstepping - initialization failure", handle->label);
        LOG_FATAL(logMsg);

        return kStatus_Fail;
    }

    if (set_double_edge_step_pulse(handle, 1, deadline) != kStatus_Success)
    {
        free_handle(handle);

        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to set double edge step pulse - initialization failure",
                 handle->label);
        LOG_FATAL(logMsg);

        return kStatus_Fail;
    }

    if (set_irun_divider(handle, config.irunDivider, deadline) != kStatus_Success)
    {
        free_handle(handle);

        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to set run current divider - initialization failure", handle->label);
        LOG_FATAL(logMsg);

        return kStatus_Fail;
    }

    if (set_ihold_divider(handle, config.iholdDivider, deadline) != kStatus_Success)
    {
        free_handle(handle);

        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to set hold current divider - initialization failure", handle->label);
        LOG_FATAL(logMsg);

        return kStatus_Fail;
    }

    snprintf(logMsg, sizeof(logMsg), "[%s] TMC2209 initialized successfully", handle->label);
    LOG_INFO(logMsg);

    return kStatus_Success;
}

static status_t free_handle(TMC_Handle_t handle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    for (size_t i = 0; i < TMC_MAX_HANDLE_COUNT; i++)
    {
        if (&tmcHandleArray[i].handle == handle)
        {
            tmcHandleArray[i].used = 0;
            return kStatus_Success;
        }
    }

    return kStatus_Fail; // Handle not found in array
}

static status_t read(TMC_Handle_t handle, uint8_t regAddr, uint32_t* data, TickType_t deadline)
{
    if (handle == NULL || data == NULL)
    {
        return kStatus_Fail;
    }

    status_t            status;
    TMC_Read_Datagram_t readDatagram;
    char                logMsg[60];

    create_read_datagram(handle, regAddr, &readDatagram);
    build_read_package(&readDatagram, transmit_read_package);
    TickType_t currentTick = xTaskGetTickCount();
    if (deadline <= currentTick)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Deadline for UART receive has already passed",
                 handle->label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }
    TickType_t ticksUntilDeadline = deadline - currentTick;
    status = UART_RTOS_Send(handle->uartRTOSHandle, transmit_read_package, TMC_READ_PACKAGE_SIZE);
    dump_rx_buffer(handle, TMC_READ_PACKAGE_SIZE, deadline);
    if (status != kStatus_Success)
    {
        char logMsg[60];
        snprintf(logMsg, sizeof(logMsg), "[%s] UART send failed during read", handle->label);
        LOG_ERROR(logMsg);
        return status;
    }

    status = UART_RTOS_Receive(handle->uartRTOSHandle, transmit_response_package,
                               TMC_WRITE_PACKAGE_SIZE, NULL, ticksUntilDeadline);
    if (status != kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] UART receive timeout during read", handle->label);
        LOG_WARN(logMsg);
        return status;
    }

    calc_CRC(transmit_response_package, TMC_WRITE_PACKAGE_SIZE);

    *data = (transmit_response_package[3] << 24) | (transmit_response_package[4] << 16) |
            (transmit_response_package[5] << 8) | transmit_response_package[6];

    /* Clear response buffer */
    for (size_t i = 0; i < TMC_WRITE_PACKAGE_SIZE; i++)
    {
        transmit_response_package[i] = 0;
    }

    return kStatus_Success;
}

static status_t write(TMC_Handle_t handle, uint8_t regAddr, uint32_t* data, TickType_t deadline)
{
    if (handle == NULL || data == NULL)
    {
        return kStatus_Fail;
    }

#if TMC_CONFIRM_WRITES == (1)
    uint8_t expectedTransmissionCount = handle->transmissionCount + 1;
#endif

    status_t             status;
    TMC_Write_Datagram_t writeDatagram;

    create_write_datagram(handle, regAddr, *data, &writeDatagram);
    build_write_package(&writeDatagram, transmit_write_package);

    status = UART_RTOS_Send(handle->uartRTOSHandle, transmit_write_package, TMC_WRITE_PACKAGE_SIZE);
    dump_rx_buffer(handle, TMC_WRITE_PACKAGE_SIZE, deadline);
    if (status != kStatus_Success)
    {
        char logMsg[60];
        snprintf(logMsg, sizeof(logMsg), "[%s] UART send failed during write", handle->label);
        LOG_ERROR(logMsg);
        return status;
    }

#if TMC_CONFIRM_WRITES == (1)
    status = read_transmissioncount(handle, deadline);
    if (status != kStatus_Success)
    {
        char logMsg[70];
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to confirm write (transmissioncount)",
                 handle->label);
        LOG_WARN(logMsg);
        return status;
    }

    if (expectedTransmissionCount != handle->transmissionCount)
    {
        char logMsg[80];
        snprintf(logMsg, sizeof(logMsg), "[%s] Write confirmation mismatch: exp=%d got=%d",
                 handle->label, expectedTransmissionCount, handle->transmissionCount);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }
#endif

    return kStatus_Success;
}

status_t read_transmissioncount(TMC_Handle_t handle, TickType_t deadline)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    status_t status;
    uint32_t receiveBuffer;

    status = read(handle, TMC_IFCNT_ADDR, &receiveBuffer, deadline);
    if (status != kStatus_Success)
    {
        return status;
    }

    handle->transmissionCount = receiveBuffer & 0xFF;
    return kStatus_Success;
}

static inline void create_write_datagram(TMC_Handle_t handle, uint8_t reg_addr, uint32_t data,
                                         TMC_Write_Datagram_t* datagram)
{
    if (handle == NULL || datagram == NULL)
    {
        return;
    }

    datagram->sync     = 0b0101;
    datagram->reserved = 0b0000;
    datagram->dev_addr = handle->serialAdress & 0xFF;
    datagram->reg_addr = (reg_addr | 0x80) & 0xFF;
    datagram->data     = data & 0xFFFFFFFF;
}

static inline void create_read_datagram(TMC_Handle_t handle, uint8_t reg_addr,
                                        TMC_Read_Datagram_t* datagram)
{
    if (handle == NULL || datagram == NULL)
    {
        return;
    }

    datagram->sync     = 0b0101;
    datagram->reserved = 0b0000;
    datagram->dev_addr = handle->serialAdress & 0xFF;
    datagram->reg_addr = reg_addr & 0xFF;
}

static inline void calc_CRC(uint8_t* datagram, size_t datagramLength)
{
    if (datagram == NULL || datagramLength == 0)
    {
        return;
    }

    uint8_t  i;
    uint8_t  j;
    uint8_t* crc = datagram + (datagramLength - 1); // CRC located in last byte of message
    uint8_t  currentByte;
    *crc = 0;
    for (i = 0; i < (datagramLength - 1); i++)
    {                              // Execute for all bytes of a message
        currentByte = datagram[i]; // Retrieve a byte to be sent from Array
        for (j = 0; j < 8; j++)
        {
            if ((*crc >> 7) ^ (currentByte & 0x01)) // update CRC based result of XOR operation
            {
                *crc = (*crc << 1) ^ 0x07;
            }
            else
            {
                *crc = (*crc << 1);
            }
            currentByte = currentByte >> 1;
        } // for CRC bit
    } // for message byte
}

static inline void build_read_package(TMC_Read_Datagram_t* datagram, uint8_t* package)
{
    if (datagram == NULL || package == NULL)
    {
        return;
    }

    package[0] = ((datagram->reserved << 4) & 0xF0) | (datagram->sync & 0x0F);
    package[1] = datagram->dev_addr & 0xFF;
    package[2] = datagram->reg_addr & 0xFF;
    package[3] = datagram->crc & 0xFF;
    calc_CRC(package, TMC_READ_PACKAGE_SIZE);
}

static inline void build_write_package(TMC_Write_Datagram_t* datagram, uint8_t* package)
{
    if (datagram == NULL || package == NULL)
    {
        return;
    }

    package[0] = ((datagram->reserved << 4) & 0xF0) | (datagram->sync & 0x0F);
    package[1] = datagram->dev_addr & 0xFF;
    package[2] = datagram->reg_addr & 0xFF;
    package[3] = (datagram->data >> 24) & 0xFF;
    package[4] = (datagram->data >> 16) & 0xFF;
    package[5] = (datagram->data >> 8) & 0xFF;
    package[6] = datagram->data & 0xFF;
    package[7] = datagram->crc & 0xFF;
    calc_CRC(package, TMC_WRITE_PACKAGE_SIZE);
}

static uint8_t dump_rx_buffer(TMC_Handle_t handle, uint8_t dumpCount, TickType_t deadline)
{
    if (handle == NULL)
    {
        return 0;
    }

    /* Dump received echo */
    static char logMsg[60];
    uint8_t     dump = 0;
    size_t      n;
    uint8_t     testCount = 0;

    for (uint8_t i = 0; i < dumpCount; i++)
    {
        TickType_t currentTick = xTaskGetTickCount();
        if (deadline <= currentTick)
        {
            snprintf(logMsg, sizeof(logMsg), "[%s] Deadline for UART receive has already passed",
                     handle->label);
            LOG_ERROR(logMsg);
            return kStatus_Fail;
        }
        TickType_t ticksUntilDeadline = deadline - currentTick;

        status_t status =
            UART_RTOS_Receive(handle->uartRTOSHandle, &dump, 1, &n, ticksUntilDeadline);
        // size_t   currentLength = UART_TransferGetRxRingBufferLength(handle->uartHandle);
        if (status == kStatus_Success && n > 0)
        {
            testCount++;
        }
        else
        {
            LOG_ERROR("Failed to dump RX buffer byte");
        }
    }

    return testCount;
}
