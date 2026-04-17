/************************************************************
 * @file    motor_tmc.c
 * @brief   Filedescription
 * @author  dg
 * @date    15 Apr 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include "motor_tmc.h"
#include "motor_motion.h"
#include "log.h"
#include "stdio.h"
#include "sync_wait.h"
#include "tmc2209_core.h"

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

status_t MTRi_set_run_current(MTR_MotorHandle_t handle, double current_a, TickType_t deadline)
{
    static char logMsg[100];
    if (MTR_is_emergency_stop_active())
    {
        return kStatus_Fail;
    }
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    uint8_t divider = 0;
    if (TMC_current_to_divider((float)current_a, TMC_ROUND_NEAREST, &divider) != kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Requested run current %.3f A is out of valid range",
                 handle->label, current_a);
        LOG_WARN(logMsg);
        return kStatus_Fail;
    }

    CHD_CmdHandle_t cmdHandle = NULL;
    if (TMC_set_irun_divider_async(handle->tmcHandle, divider, deadline, &cmdHandle) !=
        kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to queue run current command", handle->label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }

    if (wait_for_cmd_handle(cmdHandle, deadline) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    snprintf(logMsg, sizeof(logMsg), "[%s] Set run current requested: %.3f A", handle->label,
             current_a);
    LOG_DEBUG(logMsg);

    return kStatus_Success;
}

status_t MTRi_set_hold_current(MTR_MotorHandle_t handle, double current_a, TickType_t deadline)
{
    static char logMsg[100];
    if (MTR_is_emergency_stop_active())
    {
        return kStatus_Fail;
    }
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    uint8_t divider = 0;
    if (TMC_current_to_divider((float)current_a, TMC_ROUND_NEAREST, &divider) != kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Requested hold current %.3f A is out of valid range",
                 handle->label, current_a);
        LOG_WARN(logMsg);
        return kStatus_Fail;
    }

    CHD_CmdHandle_t cmdHandle = NULL;
    if (TMC_set_ihold_divider_async(handle->tmcHandle, divider, deadline, &cmdHandle) !=
        kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to queue hold current command",
                 handle->label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }

    if (wait_for_cmd_handle(cmdHandle, deadline) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    snprintf(logMsg, sizeof(logMsg), "[%s] Set hold current requested: %.3f A", handle->label,
             current_a);
    LOG_DEBUG(logMsg);

    return kStatus_Success;
}

status_t MTRi_enable_freewheeling(MTR_MotorHandle_t handle, TickType_t deadline)
{
    static char logMsg[100];
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_MotorHandleImpl_t* motorHandle = (MTR_MotorHandleImpl_t*)handle;

    CHD_CmdHandle_t cmdHandle = NULL;
    if (TMC_enable_freewheeling_async(motorHandle->tmcHandle, deadline, &cmdHandle) !=
        kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to queue freewheeling enable command",
                 handle->label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }

    if (wait_for_cmd_handle(cmdHandle, deadline) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    motorHandle->freewheeling = 1;
    snprintf(logMsg, sizeof(logMsg), "[%s] Freewheeling mode enabled", handle->label);
    LOG_INFO(logMsg);
    return kStatus_Success;
}

status_t MTRi_disable_freewheeling(MTR_MotorHandle_t handle, TickType_t deadline)
{
    static char logMsg[100];
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_MotorHandleImpl_t* motorHandle = (MTR_MotorHandleImpl_t*)handle;

    if (!motorHandle->freewheeling)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Motor is not in freewheeling mode", handle->label);
        LOG_WARN(logMsg);
        return kStatus_Fail;
    }

    CHD_CmdHandle_t cmdHandle = NULL;
    if (TMC_disable_freewheeling_async(motorHandle->tmcHandle, deadline, &cmdHandle) !=
        kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to queue freewheeling disable command",
                 handle->label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }

    if (wait_for_cmd_handle(cmdHandle, deadline) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    if (MTRi_set_home_position(handle) != kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to home motor after disabling freewheeling",
                 handle->label);
        LOG_ERROR(logMsg);
        motorHandle->freewheeling = 0;
        return kStatus_Fail;
    }

    motorHandle->freewheeling = 0;
    snprintf(logMsg, sizeof(logMsg), "[%s] Freewheeling mode disabled, motor homed", handle->label);
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
