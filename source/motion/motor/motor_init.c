/************************************************************
 * @file    motor_init.c
 * @brief   Filedescription
 * @author  dg
 * @date    15 Apr 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include "motor_init.h"
#include "log.h"
#include "stdio.h"
#include "step_core.h"
#include "tmc2209_core.h"
#include "sync_wait.h"

/************************************
 *     Private Macros / Defines    *
 ************************************/

/***************************
 *     Private Typedefs     *
 ***************************/

/*****************************************
 *     Private Function Declarations     *
 *****************************************/

static status_t wait_for_cmd_handle(CHD_CmdHandle_t cmdHandle, TickType_t deadline);

/****************************
 *     Public Variables     *
 ****************************/

/*****************************
 *     Private Variables     *
 *****************************/

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

status_t MTR_init_motor(MTR_MotorConfig_t config, TickType_t deadline,
                        MTR_HandlesArrayItem_t* handles, uint8_t maxHandles)
{
    static char logMsg[100];

    if (handles == NULL || maxHandles == 0)
    {
        return kStatus_Fail;
    }

    if (config.stepAngle <= 0.0 || config.microstep == 0 || config.reductionFactor <= 0.0)
    {
        snprintf(
            logMsg, sizeof(logMsg),
            "[%s] Invalid motor configuration: stepAngle=%.4f, microstep=%d, reductionFactor=%.4f",
            config.label, config.stepAngle, config.microstep, config.reductionFactor);
        LOG_FATAL(logMsg);
        return kStatus_Fail;
    }

    STP_StepperConfig_t stepperConfig;
    STP_get_default_config(&stepperConfig);

    stepperConfig.ftmBase               = config.stepperConfig.ftmBase;
    stepperConfig.ftmChannel            = config.stepperConfig.ftmChannel;
    stepperConfig.stepPort              = config.stepperConfig.stepPort;
    stepperConfig.stepGPIO              = config.stepperConfig.stepGPIO;
    stepperConfig.stepPin               = config.stepperConfig.stepPin;
    stepperConfig.dirPort               = config.stepperConfig.dirPort;
    stepperConfig.dirPin                = config.stepperConfig.dirPin;
    stepperConfig.dirLogicHighClockwise = config.stepperConfig.dirLogicHighClockwise;
    stepperConfig.label                 = config.label;

    stepperConfig.acceleration =
        config.acceleration * config.reductionFactor * config.microstep / config.stepAngle;
    stepperConfig.endVelocity =
        config.endVelocity * config.reductionFactor * config.microstep / config.stepAngle;

    CHD_CmdHandle_t stepCmdHandle = NULL;
    if (STP_init_handle_async(stepperConfig, deadline, &stepCmdHandle) != kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to initialize stepper motor handle",
                 config.label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }

    TMC_Config_t tmcConfig;
    TMC_get_default_config(&tmcConfig);

    tmcConfig.label = config.label;
    TMC_microstepping_uint_to_enum(config.microstep, &tmcConfig.microstepping);
    tmcConfig.serialAdress   = config.tmcConfig.serialAdress;
    tmcConfig.uartHandle     = config.tmcConfig.uartHandle;
    tmcConfig.uartRTOSHandle = config.tmcConfig.uartRTOSHandle;
    TMC_current_to_divider(config.tmcConfig.iHoldCurrentA, TMC_ROUND_NEAREST,
                           &tmcConfig.iholdDivider);
    TMC_current_to_divider(config.tmcConfig.iRunCurrentA, TMC_ROUND_NEAREST,
                           &tmcConfig.irunDivider);

    CHD_CmdHandle_t tmcCmdHandle = NULL;
    if (TMC_init_handle_async(tmcConfig, deadline, &tmcCmdHandle) != kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to initialize TMC driver handle",
                 config.label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }

    if (wait_for_cmd_handle(stepCmdHandle, deadline) != kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Stepper initialization timed out or failed",
                 config.label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }

    STP_Handle_t stepperHandle = NULL;
    if (STP_get_handle_by_label(config.label, &stepperHandle) != kStatus_Success ||
        stepperHandle == NULL)
    {
        snprintf(logMsg, sizeof(logMsg),
                 "[%s] Failed to retrieve stepper handle after initialization", config.label);
        LOG_FATAL(logMsg);
        return kStatus_Fail;
    }

    if (wait_for_cmd_handle(tmcCmdHandle, deadline) != kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] TMC initialization timed out or failed",
                 config.label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }

    TMC_Handle_t tmcHandle = NULL;
    if (TMC_get_handle_by_label(config.label, &tmcHandle) != kStatus_Success || tmcHandle == NULL)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to retrieve TMC handle after initialization",
                 config.label);
        LOG_FATAL(logMsg);
        return kStatus_Fail;
    }

    uint8_t handleIndex = maxHandles;
    for (uint8_t i = 0; i < maxHandles; i++)
    {
        if (handles[i].used == 0)
        {
            handles[i].used = 1;
            handleIndex     = i;
            break;
        }
    }

    if (handleIndex == maxHandles)
    {
        snprintf(logMsg, sizeof(logMsg),
                 "[%s] No free motor handle slots available (max %d motors)", config.label,
                 maxHandles);
        LOG_FATAL(logMsg);
        return kStatus_Fail;
    }

    MTR_MotorHandle_t newHandle    = &handles[handleIndex].handle;
    newHandle->stepperHandle       = stepperHandle;
    newHandle->tmcHandle           = tmcHandle;
    newHandle->stepAngle           = config.stepAngle;
    newHandle->microstep           = config.microstep;
    newHandle->reductionFactor     = config.reductionFactor;
    newHandle->label               = config.label;
    newHandle->freewheeling        = 0;
    newHandle->previousHoldCurrent = config.tmcConfig.iHoldCurrentA;

    snprintf(logMsg, sizeof(logMsg),
             "[%s] Motor initialized successfully (step=%.2f deg, microstep=%d, reduction=%.2f)",
             config.label, config.stepAngle, config.microstep, config.reductionFactor);
    LOG_INFO(logMsg);

    return kStatus_Success;
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/

static status_t wait_for_cmd_handle(CHD_CmdHandle_t cmdHandle, TickType_t deadline)
{
    if (cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    status_t status = SYW_cmd_wait_result(cmdHandle, deadline, NULL);
    CHD_remove_cmd_handle_ref(cmdHandle);
    return status;
}
