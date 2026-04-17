/************************************************************
 * @file    motor_motion.c
 * @brief   Filedescription
 * @author  dg
 * @date    15 Apr 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include "motor_motion.h"
#include "motor_convert.h"
#include "log.h"
#include "math.h"
#include "step_core.h"
#include "stdio.h"

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

/*****************************
 *     Private Variables     *
 *****************************/

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

status_t MTRi_move_angle(MTR_MotorHandle_t handle, double angle, TickType_t deadline,
                         MTR_ParallelTaskItem* taskItem)
{
    static char logMsg[100];
    if (MTR_is_emergency_stop_active())
    {
        return kStatus_Fail;
    }
    if (handle == NULL || taskItem == NULL)
    {
        return kStatus_Fail;
    }
    snprintf(logMsg, sizeof(logMsg), "[%s] Moving by %.2f degrees", handle->label, angle);
    LOG_DEBUG(logMsg);

    int32_t steps = MTR_angle_to_steps(handle, angle, ROUND_NEAREST);

    CHD_CmdHandle_t cmdHandle = NULL;
    if (STP_move_relative_async(handle->stepperHandle, steps, deadline, &cmdHandle) !=
        kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to queue relative move command",
                 handle->label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }
    taskItem->cmdHandles[0] = cmdHandle;
    taskItem->count         = 1;
    return kStatus_Success;
}

status_t MTRi_move_absolute_angle(MTR_MotorHandle_t handle, double angle, TickType_t deadline,
                                  MTR_ParallelTaskItem* taskItem)
{
    static char logMsg[100];
    if (MTR_is_emergency_stop_active())
    {
        return kStatus_Fail;
    }
    if (handle == NULL || taskItem == NULL)
    {
        return kStatus_Fail;
    }
    snprintf(logMsg, sizeof(logMsg), "[%s] Moving to absolute angle %.2f degrees", handle->label,
             angle);
    LOG_DEBUG(logMsg);

    int32_t currentSteps;
    if (STP_get_absolute_steps(handle->stepperHandle, &currentSteps) != kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to read current position", handle->label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }
    int32_t targetSteps = MTR_angle_to_steps(handle, angle, ROUND_NEAREST);
    int32_t stepsToMove = targetSteps - currentSteps;

    CHD_CmdHandle_t cmdHandle = NULL;
    if (STP_move_relative_async(handle->stepperHandle, stepsToMove, deadline, &cmdHandle) !=
        kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to queue absolute move command",
                 handle->label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }

    taskItem->cmdHandles[0] = cmdHandle;
    taskItem->count         = 1;
    return kStatus_Success;
}

status_t MTRi_move_revolutions(MTR_MotorHandle_t handle, double revolutions, TickType_t deadline,
                               MTR_ParallelTaskItem* taskItem)
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
    double angle = revolutions * 360.0;
    snprintf(logMsg, sizeof(logMsg), "[%s] Moving %.2f revolutions (%.2f degrees)", handle->label,
             revolutions, angle);
    LOG_DEBUG(logMsg);

    return MTRi_move_angle(handle, angle, deadline, taskItem);
}

status_t MTRi_set_velocity(MTR_MotorHandle_t handle, double velocity_deg_per_sec,
                           TickType_t deadline, MTR_ParallelTaskItem* taskItem)
{
    static char logMsg[100];
    if (MTR_is_emergency_stop_active())
    {
        return kStatus_Fail;
    }
    if (handle == NULL || taskItem == NULL)
    {
        return kStatus_Fail;
    }
    snprintf(logMsg, sizeof(logMsg), "[%s] Setting velocity to %.2f deg/sec", handle->label,
             velocity_deg_per_sec);
    LOG_DEBUG(logMsg);

    CHD_CmdHandle_t cmdHandle = NULL;
    if (STP_set_end_velocity_async(
            handle->stepperHandle,
            MTR_angle_to_steps(handle, fabs(velocity_deg_per_sec), ROUND_NEAREST), deadline,
            &cmdHandle) != kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to queue velocity change command",
                 handle->label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }

    taskItem->cmdHandles[0] = cmdHandle;
    taskItem->count         = 1;
    return kStatus_Success;
}

status_t MTRi_set_acceleration(MTR_MotorHandle_t handle, double acceleration_deg_per_sec2,
                               TickType_t deadline, MTR_ParallelTaskItem* taskItem)
{
    static char logMsg[100];
    if (MTR_is_emergency_stop_active())
    {
        return kStatus_Fail;
    }
    if (handle == NULL || taskItem == NULL)
    {
        return kStatus_Fail;
    }
    snprintf(logMsg, sizeof(logMsg), "[%s] Setting acceleration to %.2f deg/sec2", handle->label,
             acceleration_deg_per_sec2);
    LOG_DEBUG(logMsg);

    CHD_CmdHandle_t cmdHandle = NULL;
    if (STP_set_acceleration_async(
            handle->stepperHandle,
            MTR_angle_to_steps(handle, fabs(acceleration_deg_per_sec2), ROUND_NEAREST), deadline,
            &cmdHandle) != kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to queue acceleration change command",
                 handle->label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }

    taskItem->cmdHandles[0] = cmdHandle;
    taskItem->count         = 1;
    return kStatus_Success;
}

status_t MTRi_stop_motor(MTR_MotorHandle_t handle, bool decelerate, TickType_t deadline,
                         MTR_ParallelTaskItem* taskItem)
{
    static char logMsg[100];
    if (MTR_is_emergency_stop_active())
    {
        return kStatus_Fail;
    }
    if (handle == NULL || taskItem == NULL)
    {
        return kStatus_Fail;
    }
    snprintf(logMsg, sizeof(logMsg), "[%s] Stopping motor (decelerate=%d)", handle->label,
             decelerate);
    LOG_DEBUG(logMsg);

    CHD_CmdHandle_t cmdHandle = NULL;
    if (STP_stop_async(handle->stepperHandle, decelerate, deadline, &cmdHandle) != kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to queue stop command", handle->label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }

    taskItem->cmdHandles[0] = cmdHandle;
    taskItem->count         = 1;
    return kStatus_Success;
}

status_t MTRi_get_current_angle(MTR_MotorHandle_t handle, double* angle, TickType_t deadline)
{
    (void)deadline;
    if (MTR_is_emergency_stop_active())
    {
        return kStatus_Fail;
    }
    if (handle == NULL || angle == NULL)
    {
        return kStatus_Fail;
    }

    int32_t absoluteSteps;
    if (STP_get_absolute_steps(handle->stepperHandle, &absoluteSteps) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    *angle = MTR_steps_to_angle(handle, absoluteSteps);

    return kStatus_Success;
}

status_t MTRi_get_movement_state(MTR_MotorHandle_t handle, STP_MovementState_t* state)
{
    if (MTR_is_emergency_stop_active())
    {
        return kStatus_Fail;
    }
    if (handle == NULL || state == NULL)
    {
        return kStatus_Fail;
    }

    return STP_get_movement_state(handle->stepperHandle, state);
}

status_t MTRi_set_home_position(MTR_MotorHandle_t handle)
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
    snprintf(logMsg, sizeof(logMsg), "[%s] Setting home position", handle->label);
    LOG_DEBUG(logMsg);

    STP_reset_absolute_position(handle->stepperHandle);

    return kStatus_Success;
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/
