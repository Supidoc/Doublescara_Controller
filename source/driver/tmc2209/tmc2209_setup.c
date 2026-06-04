/************************************************************
 * @file    tmc2209_setup.c
 * @brief   Filedescription
 * @author  dg
 * @date    14 Apr 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include <infrastructure/log.h>
#include "tmc2209_setup.h"
#include "tmc2209_shared.h"
#include "tmc2209_internal.h"
#include "tmc2209_uart.h"
#include "stdio.h"
#include "fsl_common.h"

/************************************
 *     Private Macros / Defines    *
 ************************************/

#define TMC_GCONF_ADDR 0x00
#define TMC_GSTAT_ADDR 0x01
#define TMC_CHOPCONF_ADDR 0x6c
#define TMC_IHOLD_IRUN_ADDR 0x10
#define TMC_PWMCONF_ADDR 0x70

/***************************
 *     Private Typedefs     *
 ***************************/

/*****************************************
 *     Private Function Declarations     *
 *****************************************/

/****************************
 *     Public Variables     *
 ****************************/

/*****************************
 *     Private Variables     *
 *****************************/

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

status_t TMCi_init_handle(TMC_Config_t config, TickType_t deadline)
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
    handle->freewheeling   = 0;

    static char logMsg[100];
    snprintf(logMsg, sizeof(logMsg), "[%s] Initializing handle with config", handle->label);
    LOG_DEBUG(logMsg);

    if (TMCi_read_transmissioncount(handle, deadline) != kStatus_Success)
    {
        TMCi_free_handle(handle);

        snprintf(logMsg, sizeof(logMsg),
                 "[%s] Failed to read transmission count - cannot communicate with driver",
                 handle->label);
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
    gconf |= 0b1 << 8; // No filtering of STEP pulses

    if (kStatus_Success != TMCi_write(handle, TMC_GCONF_ADDR, &gconf, deadline))
    {
        TMCi_free_handle(handle);

        snprintf(logMsg, sizeof(logMsg),
                 "[%s] Failed to write GCONF register - initialization failure", handle->label);
        LOG_FATAL(logMsg);

        return kStatus_Fail;
    }

    // reset GSTAT
    uint32_t gstat = 0b111;

    if (kStatus_Success != TMCi_write(handle, TMC_GSTAT_ADDR, &gstat, deadline))
    {
        TMCi_free_handle(handle);

        snprintf(logMsg, sizeof(logMsg),
                 "[%s] Failed to write GSTAT register - initialization failure", handle->label);
        LOG_FATAL(logMsg);

        return kStatus_Fail;
    }
    if (TMCi_set_microstepping(handle, config.microstepping, deadline) != kStatus_Success)
    {
        TMCi_free_handle(handle);

        snprintf(logMsg, sizeof(logMsg),
                 "[%s] Failed to set microstepping - initialization failure", handle->label);
        LOG_FATAL(logMsg);

        return kStatus_Fail;
    }

    if (TMCi_set_double_edge_step_pulse(handle, 1, deadline) != kStatus_Success)
    {
        TMCi_free_handle(handle);

        snprintf(logMsg, sizeof(logMsg),
                 "[%s] Failed to set double edge step pulse - initialization failure",
                 handle->label);
        LOG_FATAL(logMsg);

        return kStatus_Fail;
    }

    if (TMCi_set_irun_divider(handle, config.irunDivider, deadline) != kStatus_Success)
    {
        TMCi_free_handle(handle);

        snprintf(logMsg, sizeof(logMsg),
                 "[%s] Failed to set run current divider - initialization failure", handle->label);
        LOG_FATAL(logMsg);

        return kStatus_Fail;
    }

    if (TMCi_set_ihold_divider(handle, config.iholdDivider, deadline) != kStatus_Success)
    {
        TMCi_free_handle(handle);

        snprintf(logMsg, sizeof(logMsg),
                 "[%s] Failed to set hold current divider - initialization failure", handle->label);
        LOG_FATAL(logMsg);

        return kStatus_Fail;
    }

    snprintf(logMsg, sizeof(logMsg), "[%s] TMC2209 initialized successfully", handle->label);
    LOG_DEBUG(logMsg);

    return kStatus_Success;
}

status_t TMCi_free_handle(TMC_Handle_t handle)
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

status_t TMCi_set_ihold_divider(TMC_Handle_t handle, uint8_t ihold, TickType_t deadline)
{

    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    uint32_t    ihold_irun;
    static char logMsg[60];

    if (TMCi_read(handle, TMC_IHOLD_IRUN_ADDR, &ihold_irun, deadline) != kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg),
                 "[%s] Failed to read IHOLD register - driver communication error", handle->label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }

    ihold_irun &= ~(0b11111 << 0);        // Clear IHOLD bits
    ihold_irun |= (ihold & 0b11111) << 0; // Set new IHOLD bits
    ihold_irun &= ~(0b1111 << 16);
    ihold_irun |= (0b0011 << 16);


    if (TMCi_write(handle, TMC_IHOLD_IRUN_ADDR, &ihold_irun, deadline) != kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg),
                 "[%s] Failed to write IHOLD register - driver communication error", handle->label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }

    snprintf(logMsg, sizeof(logMsg), "[%s] Hold current divider set to %d", handle->label, ihold);
    LOG_DEBUG(logMsg);

    return kStatus_Success;
}

status_t TMCi_set_irun_divider(TMC_Handle_t handle, uint8_t irun, TickType_t deadline)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    uint32_t    ihold_irun;
    static char logMsg[60];

    if (TMCi_read(handle, TMC_IHOLD_IRUN_ADDR, &ihold_irun, deadline) != kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg),
                 "[%s] Failed to read IRUN register - driver communication error", handle->label);
        LOG_ERROR(logMsg);

        return kStatus_Fail;
    }

    ihold_irun &= ~(0b11111 << 8);       // Clear IRUN bits
    ihold_irun |= (irun & 0b11111) << 8; // Set new IRUN bits

    ihold_irun &= ~(0b1111 << 16);
    ihold_irun |= (0b0011 << 16);

    if (TMCi_write(handle, TMC_IHOLD_IRUN_ADDR, &ihold_irun, deadline) != kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg),
                 "[%s] Failed to write IRUN register - driver communication error", handle->label);
        LOG_ERROR(logMsg);

        return kStatus_Fail;
    }

    snprintf(logMsg, sizeof(logMsg), "[%s] Run current divider set to %d", handle->label, irun);
    LOG_DEBUG(logMsg);

    return kStatus_Success;
}

status_t TMCi_set_microstepping(TMC_Handle_t handle, TMC_MICROSTEPPING_t microstepping,
                                TickType_t deadline)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    static char logMsg[60];

    uint32_t chopConf;
    TMCi_read(handle, TMC_CHOPCONF_ADDR, &chopConf, deadline);
    chopConf &= ~(0b1111 << 24);
    chopConf |= microstepping << 24;

    if (kStatus_Success != TMCi_write(handle, TMC_CHOPCONF_ADDR, &chopConf, deadline))
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to write Microstepping to CHOPCONF register",
                 handle->label);
        LOG_ERROR(logMsg);

        return kStatus_Fail;
    }
    uint16_t microsteppingInteger;
    TMC_microstepping_enum_to_uint_impl(microstepping, &microsteppingInteger);
    snprintf(logMsg, sizeof(logMsg), "[%s] Microstepping set to %d", handle->label,
             microsteppingInteger);
    LOG_DEBUG(logMsg);

    return kStatus_Success;
}

status_t TMCi_set_double_edge_step_pulse(TMC_Handle_t handle, uint8_t enabled, TickType_t deadline)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    uint32_t chopConf;
    TMCi_read(handle, TMC_CHOPCONF_ADDR, &chopConf, deadline);
    chopConf &= ~(0b1 << 29);
    chopConf |= (enabled && 0b1) << 29;
    char logMsg[60];

    if (kStatus_Success != TMCi_write(handle, TMC_CHOPCONF_ADDR, &chopConf, deadline))
    {
        snprintf(logMsg, sizeof(logMsg),
                 "[%s] Failed to write Double Edge Pulse to CHOPCONF register", handle->label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }
    return kStatus_Success;
}

status_t TMCi_set_freewheeling(TMC_Handle_t handle, uint8_t enabled, TickType_t deadline)
{
    char logMsg[60];

    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    status_t iholdStatus;
    if (enabled)
    {
        iholdStatus = TMCi_set_ihold_divider(handle, 0, deadline);
    }
    else
    {
        iholdStatus = TMCi_set_ihold_divider(handle, handle->iholdDivider, deadline);
    }
    if (iholdStatus != kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to set IHOLD for freewheeling",
                 handle->label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }

    uint32_t pwmconf = 0;
    if (TMCi_read(handle, TMC_PWMCONF_ADDR, &pwmconf, deadline) != kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to read PWMCONF for freewheeling",
                 handle->label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }
    pwmconf &= ~(0b11 << 20);
    if (enabled)
    {
        pwmconf |= 0b01 << 20; // Enable freewheeling
    }
    else
    {
        pwmconf &= ~(0b11 << 20); // Disable freewheeling
    }

    if (kStatus_Success != TMCi_write(handle, TMC_PWMCONF_ADDR, &pwmconf, deadline))
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to write Freewheeling to PWMCONF register",
                 handle->label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }

    handle->freewheeling = enabled;
    snprintf(logMsg, sizeof(logMsg), "[%s] Freewheeling %s", handle->label,
             enabled ? "enabled" : "disabled");
    LOG_INFO(logMsg);
    return kStatus_Success;
}

status_t TMCi_microstepping_uint_to_enum_impl(uint16_t in, TMC_MICROSTEPPING_t* out)
{
    if (out == NULL)
    {
        return kStatus_Fail;
    }

    switch (in)
    {
        case 256:
            *out = TMC_MS_256;
            return kStatus_Success;
        case 128:
            *out = TMC_MS_128;
            return kStatus_Success;
        case 64:
            *out = TMC_MS_64;
            return kStatus_Success;
        case 32:
            *out = TMC_MS_32;
            return kStatus_Success;
        case 16:
            *out = TMC_MS_16;
            return kStatus_Success;
        case 8:
            *out = TMC_MS_8;
            return kStatus_Success;
        case 4:
            *out = TMC_MS_4;
            return kStatus_Success;
        case 2:
            *out = TMC_MS_2;
            return kStatus_Success;
        case 1:
            *out = TMC_MS_FULL;
            return kStatus_Success;
        default:
            return kStatus_Fail;
    }
}

status_t TMC_microstepping_enum_to_uint_impl(TMC_MICROSTEPPING_t in, uint16_t* out)
{
    if (out == NULL)
    {
        return kStatus_Fail;
    }

    switch (in)
    {
        case TMC_MS_256:
            *out = 256;
            return kStatus_Success;
        case TMC_MS_128:
            *out = 128;
            return kStatus_Success;
        case TMC_MS_64:
            *out = 64;
            return kStatus_Success;
        case TMC_MS_32:
            *out = 32;
            return kStatus_Success;
        case TMC_MS_16:
            *out = 16;
            return kStatus_Success;
        case TMC_MS_8:
            *out = 8;
            return kStatus_Success;
        case TMC_MS_4:
            *out = 4;
            return kStatus_Success;
        case TMC_MS_2:
            *out = 2;
            return kStatus_Success;
        case TMC_MS_FULL:
            *out = 1;
            return kStatus_Success;
        default:
            *out = 0;
            return kStatus_Fail;
    }
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/
