/************************************************************
 * @file    motor_process.c
 * @brief   Filedescription
 * @author  dg
 * @date    15 Apr 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include "motor_process.h"
#include "motor_init.h"
#include "motor_motion.h"
#include "motor_sync.h"
#include "motor_tmc.h"
#include "cmd_dispatch.h"
#include "sync_wait.h"
#include "log.h"
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

status_t MTRi_send_cmd_async(MTR_CmdQueueItem_t* queueItem, TickType_t deadline,
                             CHD_CmdHandle_t* cmdHandle)
{
    if (queueItem == NULL)
    {
        return kStatus_Fail;
    }
    CHD_CmdHandle_t internaleCmdHandle = NULL;

    status_t allocStatus =
        CHD_get_cmd_handle(&internaleCmdHandle, mtrCmdHandles, MTR_MAX_CMD_HANDLE_COUNT);
    if (allocStatus != kStatus_Success)
    {
        return kStatus_Fail;
    }
    if (cmdHandle != NULL)
    {
        CHD_add_cmd_handle_ref(internaleCmdHandle);
    }

    queueItem->cmdHandle = internaleCmdHandle;

    status_t sendStatus = CDP_send_cmd(mtrCmdQueue, queueItem, deadline, internaleCmdHandle);
    if (sendStatus != kStatus_Success)
    {
        CHD_remove_cmd_handle_ref(internaleCmdHandle);
        if (cmdHandle != NULL)
        {
            *cmdHandle = NULL;
        }
        return kStatus_Fail;
    }
    if (cmdHandle != NULL)
    {
        *cmdHandle = internaleCmdHandle;
    }
    return kStatus_Success;
}

void MTRi_process(void)
{
    MTR_CmdQueueItem_t queueItem;
    BaseType_t         queueStatus;
    uint8_t            activeTaskCount = 0;

    status_t taskStatus = kStatus_Fail;
    for (uint8_t i = 0; i < MTR_MAX_PARALLEL_TASKS; i++)
    {
        if (parallelTasks[i].used)
        {
            activeTaskCount++;
            taskStatus =
                SYW_cmd_check_all(parallelTasks[i].cmdHandles, parallelTasks[i].count, NULL);
            if (taskStatus == kStatus_Busy)
            {
                continue;
            }
            else if (taskStatus == kStatus_Success)
            {
                CDP_notify_task_success(parallelTasks[i].returnHandle);
            }
            else if (taskStatus == kStatus_Timeout)
            {
                CDP_notify_task_timeout(parallelTasks[i].returnHandle);
            }
            else
            {
                CDP_notify_task_failure(parallelTasks[i].returnHandle);
            }
            CHD_remove_cmd_handle_ref(parallelTasks[i].returnHandle);
            parallelTasks[i].used = 0;
        }
    }

    TickType_t delay = activeTaskCount ? pdMS_TO_TICKS(2) : portMAX_DELAY;

    queueStatus = xQueueReceive(mtrCmdQueue, &queueItem, delay);
    if (queueStatus != pdPASS)
    {
        return;
    }

    if (MTR_is_emergency_stop_active())
    {
        LOG_DEBUG("Command ignored due to active emergency stop");
        CDP_notify_task_failure(queueItem.cmdHandle);
        CHD_remove_cmd_handle_ref(queueItem.cmdHandle);
        return;
    }

    MTR_ParallelTaskItem* taskItem = NULL;
    for (uint8_t i = 0; i < MTR_MAX_PARALLEL_TASKS; i++)
    {
        if (parallelTasks[i].used == 0)
        {
            parallelTasks[i].used = 1;
            taskItem              = &parallelTasks[i];
            break;
        }
    }

    if (taskItem != NULL)
    {
        taskItem->count        = 0;
        taskItem->returnHandle = queueItem.cmdHandle;
        MTRi_process_cmd(queueItem, taskItem);
    }
}

status_t MTRi_process_cmd(MTR_CmdQueueItem_t queueItem, MTR_ParallelTaskItem* taskItem)
{
    if (taskItem == NULL)
    {
        return kStatus_Fail;
    }

    status_t cmdStatus = kStatus_Fail;

    switch (queueItem.type)
    {
        case MTR_CMD_MOVE_ANGLE:
            cmdStatus = MTRi_move_angle(queueItem.handle, queueItem.data.moveAngle.angle,
                                        queueItem.deadline, taskItem);
            break;

        case MTR_CMD_MOVE_ABSOLUTE_ANGLE:
            cmdStatus = MTRi_move_absolute_angle(queueItem.handle, queueItem.data.moveAngle.angle,
                                                 queueItem.deadline, taskItem);
            break;

        case MTR_CMD_MOVE_REVOLUTIONS:
            cmdStatus =
                MTRi_move_revolutions(queueItem.handle, queueItem.data.moveRevolutions.revolutions,
                                      queueItem.deadline, taskItem);
            break;

        case MTR_CMD_SET_VELOCITY:
            cmdStatus =
                MTRi_set_velocity(queueItem.handle, queueItem.data.setVelocity.velocity_deg_per_sec,
                                  queueItem.deadline, taskItem);
            break;

        case MTR_CMD_SET_ACCELERATION:
            cmdStatus = MTRi_set_acceleration(
                queueItem.handle, queueItem.data.setAcceleration.acceleration_deg_per_sec2,
                queueItem.deadline, taskItem);
            break;

        case MTR_CMD_STOP:
            cmdStatus = MTRi_stop_motor(queueItem.handle, queueItem.data.stop.decelerate,
                                        queueItem.deadline, taskItem);
            break;

        case MTR_CMD_GET_CURRENT_ANGLE:
            cmdStatus = MTRi_get_current_angle(
                queueItem.handle, queueItem.data.getCurrentAngle.angle, queueItem.deadline);
            break;

        case MTR_CMD_GET_MOVEMENT_STATE:
            cmdStatus =
                MTRi_get_movement_state(queueItem.handle, queueItem.data.getMovementState.state);
            break;
        case MTR_CMD_SYNCHRONIZED_MOVE:
            cmdStatus = MTRi_synchronized_move(
                queueItem.data.synchronizedMove.handles, queueItem.data.synchronizedMove.angles,
                queueItem.data.synchronizedMove.count, queueItem.deadline);
            break;

        case MTR_CMD_SET_RUN_CURRENT:
            cmdStatus = MTRi_set_run_current(
                queueItem.handle, queueItem.data.setRunCurrent.current_a, queueItem.deadline);
            break;

        case MTR_CMD_SET_HOLD_CURRENT:
            cmdStatus = MTRi_set_hold_current(
                queueItem.handle, queueItem.data.setHoldCurrent.current_a, queueItem.deadline);
            break;
        case MTR_CMD_INIT_MOTOR:
            cmdStatus = MTR_init_motor(queueItem.data.initMotor.config, queueItem.deadline,
                                       motorHandles, MTR_MAX_MOTORS);
            break;
        case MTR_CMD_ENABLE_FREEWHEELING:
            cmdStatus = MTRi_enable_freewheeling(queueItem.handle, queueItem.deadline);
            break;
        case MTR_CMD_DISABLE_FREEWHEELING:
            cmdStatus = MTRi_disable_freewheeling(queueItem.handle, queueItem.deadline);
            break;
        default:
        {
            static char errMsg[100];
            snprintf(errMsg, sizeof(errMsg), "Unknown motor command type: %d", queueItem.type);
            LOG_ERROR(errMsg);
            break;
        }
    }

    if (cmdStatus == kStatus_Success && taskItem->count == 0)
    {
        CDP_notify_task_success(queueItem.cmdHandle);
        taskItem->used = 0;
        CHD_remove_cmd_handle_ref(queueItem.cmdHandle);
    }
    if (cmdStatus == kStatus_Timeout)
    {
        CDP_notify_task_timeout(queueItem.cmdHandle);
        taskItem->used = 0;
        CHD_remove_cmd_handle_ref(queueItem.cmdHandle);
    }
    else if (cmdStatus == kStatus_Fail)
    {
        CDP_notify_task_failure(queueItem.cmdHandle);
        taskItem->used = 0;
        CHD_remove_cmd_handle_ref(queueItem.cmdHandle);
    }

    return cmdStatus;
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/
