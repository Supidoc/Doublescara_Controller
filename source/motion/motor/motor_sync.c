/************************************************************
 * @file    motor_sync.c
 * @brief   Filedescription
 * @author  dg
 * @date    15 Apr 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include "motor_sync.h"
#include "motor_convert.h"
#include "log.h"
#include "step_core.h"
#include "stdio.h"
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
static status_t calculate_sync_motion_parameters(MTR_MotorHandle_t* handles, double* angles,
                                                 uint8_t count, int32_t* stepCounts,
                                                 double* originalVelocities, double* maxDuration);
static status_t scale_velocities_for_synchronization(MTR_MotorHandle_t* handles, uint8_t count,
                                                     int32_t* stepCounts,
                                                     double* originalVelocities, double maxDuration,
                                                     TickType_t deadline);
static status_t prepare_synchronized_movements(MTR_MotorHandle_t* handles, uint8_t count,
                                               int32_t* stepCounts, double* originalVelocities,
                                               TickType_t deadline);
static status_t restore_original_velocities(MTR_MotorHandle_t* handles, uint8_t count,
                                            double* originalVelocities, TickType_t deadline);

/****************************
 *     Public Variables     *
 ****************************/

/*****************************
 *     Private Variables     *
 *****************************/

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

status_t MTRi_synchronized_move(MTR_MotorHandle_t* handles, double* angles, uint8_t count,
                                TickType_t deadline)
{
    static char logMsg[100];
    if (MTR_is_emergency_stop_active())
    {
        return kStatus_Fail;
    }
    if (handles == NULL || angles == NULL || count == 0)
    {
        return kStatus_Fail;
    }

    int32_t stepCounts[count];
    double  originalVelocities[count];
    double  maxDuration;

    if (calculate_sync_motion_parameters(handles, angles, count, stepCounts, originalVelocities,
                                         &maxDuration) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    if (scale_velocities_for_synchronization(handles, count, stepCounts, originalVelocities,
                                             maxDuration, deadline) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    snprintf(logMsg, sizeof(logMsg), "Starting synchronized move: %d motors, duration=%.3fs", count,
             maxDuration);
    LOG_INFO(logMsg);

    if (prepare_synchronized_movements(handles, count, stepCounts, originalVelocities, deadline) !=
        kStatus_Success)
    {
        return kStatus_Fail;
    }

    CHD_CmdHandle_t triggerCmdHandle = NULL;
    status_t        triggerResult = STP_trigger_prepared_moves_async(deadline, &triggerCmdHandle);
    if (triggerResult != kStatus_Success)
    {
        restore_original_velocities(handles, count, originalVelocities, deadline);
        return kStatus_Fail;
    }
    if (wait_for_cmd_handle(triggerCmdHandle, deadline) != kStatus_Success)
    {
        restore_original_velocities(handles, count, originalVelocities, deadline);
        return kStatus_Fail;
    }

    if (restore_original_velocities(handles, count, originalVelocities, deadline) !=
        kStatus_Success)
    {
        return kStatus_Fail;
    }

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

static status_t calculate_sync_motion_parameters(MTR_MotorHandle_t* handles, double* angles,
                                                 uint8_t count, int32_t* stepCounts,
                                                 double* originalVelocities, double* maxDuration)
{
    *maxDuration = 0.0;

    for (uint8_t i = 0; i < count; i++)
    {
        if (handles[i] == NULL)
        {
            return kStatus_Fail;
        }

        stepCounts[i] = MTR_angle_to_steps(handles[i], angles[i], ROUND_NEAREST);
        if (STP_get_end_velocity(handles[i]->stepperHandle, &originalVelocities[i]) !=
            kStatus_Success)
        {
            return kStatus_Fail;
        }

        if (originalVelocities[i] <= 0)
        {
            return kStatus_Fail;
        }

        uint32_t absSteps     = (stepCounts[i] >= 0) ? stepCounts[i] : -stepCounts[i];
        double   moveDuration = (double)absSteps / originalVelocities[i];
        if (moveDuration > *maxDuration)
        {
            *maxDuration = moveDuration;
        }
    }

    return kStatus_Success;
}

static status_t scale_velocities_for_synchronization(MTR_MotorHandle_t* handles, uint8_t count,
                                                     int32_t* stepCounts,
                                                     double* originalVelocities, double maxDuration,
                                                     TickType_t deadline)
{
    for (uint8_t i = 0; i < count; i++)
    {
        if (stepCounts[i] != 0)
        {
            uint32_t absSteps    = (stepCounts[i] >= 0) ? stepCounts[i] : -stepCounts[i];
            double   newVelocity = (double)absSteps / maxDuration;

            if (newVelocity > originalVelocities[i])
            {
                newVelocity = originalVelocities[i];
            }

            CHD_CmdHandle_t cmdHandle = NULL;
            if (STP_set_end_velocity_async(handles[i]->stepperHandle, newVelocity, deadline,
                                           &cmdHandle) != kStatus_Success)
            {
                return kStatus_Fail;
            }
            if (wait_for_cmd_handle(cmdHandle, deadline) != kStatus_Success)
            {
                return kStatus_Fail;
            }
        }
    }
    return kStatus_Success;
}

static status_t prepare_synchronized_movements(MTR_MotorHandle_t* handles, uint8_t count,
                                               int32_t* stepCounts, double* originalVelocities,
                                               TickType_t deadline)
{
    CHD_CmdHandle_t cmdHandles[count];
    size_t          cmdCount = 0;

    for (uint8_t i = 0; i < count; i++)
    {
        if (stepCounts[i] != 0)
        {
            status_t result = STP_move_relative_prepare_async(
                handles[i]->stepperHandle, stepCounts[i], deadline, &cmdHandles[cmdCount]);

            if (result != kStatus_Success)
            {
                for (size_t cmdIdx = 0; cmdIdx < cmdCount; cmdIdx++)
                {
                    CHD_remove_cmd_handle_ref(cmdHandles[cmdIdx]);
                }
                restore_original_velocities(handles, count, originalVelocities, deadline);
                return kStatus_Fail;
            }

            cmdCount++;
        }
    }

    if (cmdCount == 0)
    {
        return kStatus_Success;
    }

    status_t waitStatus = SYW_cmd_wait_all(cmdHandles, cmdCount, deadline, NULL);
    for (size_t cmdIdx = 0; cmdIdx < cmdCount; cmdIdx++)
    {
        CHD_remove_cmd_handle_ref(cmdHandles[cmdIdx]);
    }
    if (waitStatus != kStatus_Success)
    {
        restore_original_velocities(handles, count, originalVelocities, deadline);
        return kStatus_Fail;
    }

    return kStatus_Success;
}

static status_t restore_original_velocities(MTR_MotorHandle_t* handles, uint8_t count,
                                            double* originalVelocities, TickType_t deadline)
{
    for (uint8_t i = 0; i < count; i++)
    {
        CHD_CmdHandle_t cmdHandle = NULL;
        if (STP_set_end_velocity_async(handles[i]->stepperHandle, originalVelocities[i], deadline,
                                       &cmdHandle) != kStatus_Success)
        {
            return kStatus_Fail;
        }
        if (wait_for_cmd_handle(cmdHandle, deadline) != kStatus_Success)
        {
            return kStatus_Fail;
        }
    }
    return kStatus_Success;
}
