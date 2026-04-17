/************************************************************
 * @file    tmc2209_core.c
 * @brief   Filedescription
 * @author  dg
 * @date    14 Apr 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include "tmc2209_core.h"
#include "tmc2209_shared.h"
#include "tmc2209_internal.h"
#include "tmc2209_uart.h"
#include "tmc2209_setup.h"
#include "log.h"
#include "stdio.h"
#include "fsl_uart_freertos.h"
#include "fsl_gpio.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "fsl_port.h"
#include "tmc2209_process.h"
#include "math.h"

/************************************
 *     Private Macros / Defines    *
 ************************************/

/***************************
 *     Private Typedefs     *
 ***************************/

/*****************************************
 *     Private Function Declarations     *
 *****************************************/

/****************************
 *     Public Variables     *
 ****************************/

TMC_HandleArrayItem_t tmcHandleArray[TMC_MAX_HANDLE_COUNT];

/*****************************
 *     Private Variables     *
 *****************************/

/*******************************************
 *     Public Function Implementations     *
 *******************************************/
status_t TMC_init(void)
{
    tmcCmdQueue = xQueueCreate(TMC_COMMAND_QUEUE_LENGTH, sizeof(TMC_CommandQueueItem_t));
    if (tmcCmdQueue == NULL)
    {
        return kStatus_Fail;
    }
    vQueueAddToRegistry(tmcCmdQueue, "TMC Command Queue");

    gpio_pin_config_t debugPinConfig;
    debugPinConfig.pinDirection = kGPIO_DigitalInput;

    GPIO_PinInit(GPIOE, 0, &debugPinConfig);

    PORT_SetPinMux(PORTE, 0, kPORT_MuxAlt1);

    CHD_init_cmd_handles(tmcCmdHandles, TMC_MAX_CMD_HANDLE_COUNT); // NEW
    return kStatus_Success;
}

void TMC_task(void* pvParameters)
{
    LOG_INFO("Started TMC2209 Task");
    for (;;)
    {
        TMCi_process();
    }
}

status_t TMC_init_handle_async(TMC_Config_t config, TickType_t deadline, CHD_CmdHandle_t* cmdHandle)
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

    return TMCi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t TMC_set_microstepping_async(TMC_Handle_t handle, TMC_MICROSTEPPING_t microstepping,
                                     TickType_t deadline, CHD_CmdHandle_t* cmdHandle)
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

    return TMCi_send_cmd_async(&queueItem, deadline, cmdHandle);
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

status_t TMC_microstepping_uint_to_enum(uint16_t in, TMC_MICROSTEPPING_t* out)
{
    return TMCi_microstepping_uint_to_enum_impl(in, out);
}

status_t TMC_microstepping_enum_to_uint(TMC_MICROSTEPPING_t in, uint16_t* out)
{
    return TMC_microstepping_enum_to_uint_impl(in, out);
}

status_t TMC_set_ihold_divider_async(TMC_Handle_t handle, uint8_t ihold, TickType_t deadline,
                                     CHD_CmdHandle_t* cmdHandle)
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

    return TMCi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t TMC_set_irun_divider_async(TMC_Handle_t handle, uint8_t irun, TickType_t deadline,
                                    CHD_CmdHandle_t* cmdHandle)
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

    return TMCi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t TMC_enable_freewheeling_async(TMC_Handle_t handle, TickType_t deadline,
                                       CHD_CmdHandle_t* cmdHandle)
{
    if (handle == NULL || cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    TMC_CommandQueueItem_t queueItem       = {0};
    queueItem.handle                       = handle;
    queueItem.commandType                  = TMC_CMD_ENABLE_FREEWHEELING;
    queueItem.data.setFreewheeling.enabled = 1;
    queueItem.deadline                     = deadline;

    return TMCi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t TMC_disable_freewheeling_async(TMC_Handle_t handle, TickType_t deadline,
                                        CHD_CmdHandle_t* cmdHandle)
{
    if (handle == NULL || cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    TMC_CommandQueueItem_t queueItem       = {0};
    queueItem.handle                       = handle;
    queueItem.commandType                  = TMC_CMD_DISABLE_FREEWHEELING;
    queueItem.data.setFreewheeling.enabled = 0;
    queueItem.deadline                     = deadline;

    return TMCi_send_cmd_async(&queueItem, deadline, cmdHandle);
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
