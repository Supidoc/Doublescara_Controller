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

/************************************
 *     Private Macros / Defines    *
 ************************************/

#define TMC_READ_PACKAGE_SIZE 4
#define TMC_WRITE_PACKAGE_SIZE 8

#define TMC_GCONF_ADDR 0x00
#define TMC_GSTAT_ADDR 0x01
#define TMC_IFCNT_ADDR 0x02
#define TMC_CHOPCONF_ADDR 0x6c

/***************************
 *     Private Typedefs     *
 ***************************/

typedef enum _TMC_CommandType
{
    TMC_CMD_DEFAULT_INIT
} TMC_CommandType_t;

typedef struct _TMC_CommandQueueItem
{
    TMC_CommandType_t commandType;
    TMC_Handle_t*     handle;
    union
    {
        uint8_t placeholder;
    } data;
} TMC_CommandQueueItem_t;

/*****************************************
 *     Private Function Declarations     *
 *****************************************/

static inline void create_read_datagram(TMC_Handle_t* handle, uint8_t reg_addr,
                                        TMC_Read_Datagram_t* datagram);
static inline void create_write_datagram(TMC_Handle_t* handle, uint8_t reg_addr, uint32_t data,
                                         TMC_Write_Datagram_t* datagram);
static inline void build_read_package(TMC_Read_Datagram_t* datagram, uint8_t* package);
static inline void build_write_package(TMC_Write_Datagram_t* datagram, uint8_t* package);
static inline void calc_CRC(uint8_t* datagram, size_t datagramLength);
static uint8_t     dump_rx_buffer(TMC_Handle_t* handle, uint8_t dumpCount);
static status_t    read_transmissioncount(TMC_Handle_t* handle);
static status_t    default_init(TMC_Handle_t* handle);
static status_t    read(TMC_Handle_t* handle, uint8_t regAddr, uint32_t* data);
static status_t    write(TMC_Handle_t* handle, uint8_t regAddr, uint32_t* data);
static void        process(void);
static status_t    set_double_edge_step_pulse(TMC_Handle_t* handle, uint8_t enabled);

/****************************
 *     Public Variables     *
 ****************************/

/*****************************
 *     Private Variables     *
 *****************************/

static uint8_t transmit_read_package[TMC_READ_PACKAGE_SIZE];
static uint8_t transmit_response_package[TMC_WRITE_PACKAGE_SIZE];
static uint8_t transmit_write_package[TMC_WRITE_PACKAGE_SIZE];

QueueHandle_t commandQueue = NULL;

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

status_t TMC_init(void)
{
    commandQueue = xQueueCreate(TMC_COMMAND_QUEUE_LENGTH, sizeof(TMC_CommandQueueItem_t));
    if (commandQueue == NULL)
    {
        return kStatus_Fail;
    }
    return kStatus_Success;
}

void TMC_task(void* pvParameters)
{
    LOG_INFO("Started TMC2209 Task");
    for (;;)
    {
        process();
        vTaskDelay(1000);
    }
}

status_t TMC_init_default(TMC_Handle_t* handle)
{
    TMC_CommandQueueItem_t queueItem;
    queueItem.handle      = handle;
    queueItem.commandType = TMC_CMD_DEFAULT_INIT;

    if (xQueueSend(commandQueue, (void*)&queueItem, portMAX_DELAY) != pdTRUE)
    {
        LOG_ERROR("Failed to queue init default command");
        return kStatus_Fail;
    }

    char logMsg[60];
    snprintf(logMsg, sizeof(logMsg), "[%s] Default init command queued", handle->label);
    LOG_DEBUG(logMsg);
    return kStatus_Success;
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/

static void process(void)
{
    TMC_CommandQueueItem_t queueItem;
    BaseType_t             status;
    status = xQueueReceive(commandQueue, &queueItem, pdMS_TO_TICKS(10));
    if (status != pdPASS)
    {
        return;
    }

    switch (queueItem.commandType)
    {
        case TMC_CMD_DEFAULT_INIT:
            default_init(queueItem.handle);
            break;
        default:
            break;
    }
}

static status_t set_microstepping(TMC_Handle_t* handle, TMC_MICROSTEPPING_t microstepping)
{
    uint32_t chopConf;
    read(handle, TMC_CHOPCONF_ADDR, &chopConf);
    chopConf &= ~(0b1111 << 24);
    chopConf |= microstepping << 24;
    char logMsg[60];

    if (kStatus_Success != write(handle, TMC_CHOPCONF_ADDR, &chopConf))
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to write Microstepping to CHOPCONF register",
                 handle->label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }
    return kStatus_Success;
}

static status_t set_double_edge_step_pulse(TMC_Handle_t* handle, uint8_t enabled)
{
    uint32_t chopConf;
    read(handle, TMC_CHOPCONF_ADDR, &chopConf);
    chopConf &= ~(0b1 << 29);
    chopConf |= (enabled && 0b1) << 24;
    char logMsg[60];

    if (kStatus_Success != write(handle, TMC_CHOPCONF_ADDR, &chopConf))
    {
        snprintf(logMsg, sizeof(logMsg),
                 "[%s] Failed to write Double Edge Pulse to CHOPCONF register", handle->label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }
    return kStatus_Success;
}

static status_t default_init(TMC_Handle_t* handle)
{
    static char logMsg[60];
    snprintf(logMsg, sizeof(logMsg), "[%s] Initializing with default config", handle->label);
    LOG_INFO(logMsg);

    if (read_transmissioncount(handle) != kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to read transmission count", handle->label);
        LOG_ERROR(logMsg);
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

    if (kStatus_Success != write(handle, TMC_GCONF_ADDR, &gconf))
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to write GCONF register", handle->label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }

    // reset GSTAT
    uint32_t gstat = 0b111;

    if (kStatus_Success != write(handle, TMC_GSTAT_ADDR, &gstat))
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to write GSTAT register", handle->label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }

    set_microstepping(handle, TMC_MS_4);
    set_double_edge_step_pulse(handle, 1);

    snprintf(logMsg, sizeof(logMsg), "[%s] Default init completed successfully", handle->label);
    LOG_INFO(logMsg);
    return kStatus_Success;
}

static status_t read(TMC_Handle_t* handle, uint8_t regAddr, uint32_t* data)
{
    status_t            status;
    TMC_Read_Datagram_t readDatagram;

    create_read_datagram(handle, regAddr, &readDatagram);
    build_read_package(&readDatagram, transmit_read_package);

    status = UART_RTOS_Send(handle->uartRTOSHandle, transmit_read_package, TMC_READ_PACKAGE_SIZE);
    dump_rx_buffer(handle, TMC_READ_PACKAGE_SIZE);
    if (status != kStatus_Success)
    {
        char logMsg[60];
        snprintf(logMsg, sizeof(logMsg), "[%s] UART send failed during read", handle->label);
        LOG_ERROR(logMsg);
        return status;
    }

    status = UART_RTOS_Receive(handle->uartRTOSHandle, transmit_response_package,
                               TMC_WRITE_PACKAGE_SIZE, NULL, TMC_TIMEOUT_MS);
    if (status != kStatus_Success)
    {
        char logMsg[60];
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

static status_t write(TMC_Handle_t* handle, uint8_t regAddr, uint32_t* data)
{
#if TMC_CONFIRM_WRITES == (1)
    uint8_t expectedTransmissionCount = handle->transmissionCount + 1;
#endif

    status_t             status;
    TMC_Write_Datagram_t writeDatagram;

    create_write_datagram(handle, regAddr, *data, &writeDatagram);
    build_write_package(&writeDatagram, transmit_write_package);

    status = UART_RTOS_Send(handle->uartRTOSHandle, transmit_write_package, TMC_WRITE_PACKAGE_SIZE);
    dump_rx_buffer(handle, TMC_WRITE_PACKAGE_SIZE);
    if (status != kStatus_Success)
    {
        char logMsg[60];
        snprintf(logMsg, sizeof(logMsg), "[%s] UART send failed during write", handle->label);
        LOG_ERROR(logMsg);
        return status;
    }

#if TMC_CONFIRM_WRITES == (1)
    status = read_transmissioncount(handle);
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

status_t read_transmissioncount(TMC_Handle_t* handle)
{
    status_t status;
    uint32_t receiveBuffer;

    status = read(handle, TMC_IFCNT_ADDR, &receiveBuffer);
    if (status != kStatus_Success)
    {
        return status;
    }

    handle->transmissionCount = receiveBuffer & 0xFF;
    return kStatus_Success;
}

static inline void create_write_datagram(TMC_Handle_t* handle, uint8_t reg_addr, uint32_t data,
                                         TMC_Write_Datagram_t* datagram)
{
    datagram->sync     = 0b0101;
    datagram->reserved = 0b0000;
    datagram->dev_addr = handle->serialAdress & 0xFF;
    datagram->reg_addr = (reg_addr | 0x80) & 0xFF;
    datagram->data     = data & 0xFFFFFFFF;
}

static inline void create_read_datagram(TMC_Handle_t* handle, uint8_t reg_addr,
                                        TMC_Read_Datagram_t* datagram)
{
    datagram->sync     = 0b0101;
    datagram->reserved = 0b0000;
    datagram->dev_addr = handle->serialAdress & 0xFF;
    datagram->reg_addr = reg_addr & 0xFF;
}

static inline void calc_CRC(uint8_t* datagram, size_t datagramLength)
{
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
    package[0] = ((datagram->reserved << 4) & 0xF0) | (datagram->sync & 0x0F);
    package[1] = datagram->dev_addr & 0xFF;
    package[2] = datagram->reg_addr & 0xFF;
    package[3] = datagram->crc & 0xFF;
    calc_CRC(package, TMC_READ_PACKAGE_SIZE);
}

static inline void build_write_package(TMC_Write_Datagram_t* datagram, uint8_t* package)
{
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

static uint8_t dump_rx_buffer(TMC_Handle_t* handle, uint8_t dumpCount)
{
    /* Dump received echo */
    uint8_t dump = 0;
    size_t  n;
    uint8_t testCount = 0;

    for (uint8_t i = 0; i < dumpCount; i++)
    {
        status_t status = UART_RTOS_Receive(handle->uartRTOSHandle, &dump, 1, &n, TMC_TIMEOUT_MS);
        size_t   currentLength = UART_TransferGetRxRingBufferLength(handle->uartHandle);
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
