/************************************************************
 * @file    motor_core.c
 * @brief   Motor core implementations
 * @author  dg
 * @date    15 Apr 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include "motor_core.h"
#include "motor_process.h"
#include "cmd_dispatch.h"
#include "log.h"
#include "queue.h"
#include "string.h"
#include "stdio.h"
#include "motor_motion.h"

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

volatile uint8_t emergencyStopFlag = 0;

/*****************************
 *     Private Variables     *
 *****************************/

MTR_HandlesArrayItem_t motorHandles[MTR_MAX_MOTORS]            = {0};
QueueHandle_t          mtrCmdQueue                             = NULL;
CHD_CmdHandleImpl_t    mtrCmdHandles[MTR_MAX_CMD_HANDLE_COUNT] = {0};
MTR_ParallelTaskItem   parallelTasks[MTR_MAX_PARALLEL_TASKS]   = {0};

static TaskHandle_t mtrTaskHandle = NULL;

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

void MTR_task(void* pvParameters)
{
    (void)pvParameters;
    LOG_INFO("Started Motor Task");
    mtrTaskHandle = xTaskGetCurrentTaskHandle();
    for (;;)
    {
        MTRi_process();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

status_t MTR_init(void)
{
    for (uint8_t i = 0; i < MTR_MAX_MOTORS; i++)
    {
        motorHandles[i].used = 0;
    }

    mtrCmdQueue = xQueueCreate(MTR_CMD_QUEUE_SIZE, sizeof(MTR_CmdQueueItem_t));
    if (mtrCmdQueue == NULL)
    {
        return kStatus_Fail;
    }
    vQueueAddToRegistry(mtrCmdQueue, "MTR Command Queue");

    CHD_init_cmd_handles(mtrCmdHandles, MTR_MAX_CMD_HANDLE_COUNT);
    emergencyStopFlag = 0;
    return kStatus_Success;
}

status_t MTR_init_handle_async(const MTR_MotorConfig_t config, TickType_t deadline,
                               CHD_CmdHandle_t* cmdHandle)
{
    if (cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.handle                = NULL;
    queueItem.type                  = MTR_CMD_INIT_MOTOR;
    queueItem.data.initMotor.config = config;
    queueItem.deadline              = deadline;

    return MTRi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

void MTR_get_motor_by_label(const char* label, MTR_MotorHandle_t* handle)
{
    if (label == NULL || handle == NULL)
    {
        return;
    }

    for (uint8_t i = 0; i < MTR_MAX_MOTORS; i++)
    {
        if (motorHandles[i].used)
        {
            if (strncmp(label, motorHandles[i].handle.label,
                        strlen(motorHandles[i].handle.label)) == 0)
            {
                *handle = &motorHandles[i].handle;
                return;
            }
        }
    }

    *handle = NULL;
}

status_t MTR_move_angle_async(MTR_MotorHandle_t handle, double angle, TickType_t deadline,
                              CHD_CmdHandle_t* cmdHandle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                 = MTR_CMD_MOVE_ANGLE;
    queueItem.handle               = handle;
    queueItem.data.moveAngle.angle = angle;
    queueItem.deadline             = deadline;

    return MTRi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t MTR_move_absolute_angle_async(MTR_MotorHandle_t handle, double angle, TickType_t deadline,
                                       CHD_CmdHandle_t* cmdHandle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                 = MTR_CMD_MOVE_ABSOLUTE_ANGLE;
    queueItem.handle               = handle;
    queueItem.data.moveAngle.angle = angle;
    queueItem.deadline             = deadline;

    return MTRi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t MTR_move_revolutions_async(MTR_MotorHandle_t handle, double revolutions,
                                    TickType_t deadline, CHD_CmdHandle_t* cmdHandle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                             = MTR_CMD_MOVE_REVOLUTIONS;
    queueItem.handle                           = handle;
    queueItem.data.moveRevolutions.revolutions = revolutions;
    queueItem.deadline                         = deadline;

    return MTRi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t MTR_set_velocity_async(MTR_MotorHandle_t handle, double velocity_deg_per_sec,
                                TickType_t deadline, CHD_CmdHandle_t* cmdHandle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                                  = MTR_CMD_SET_VELOCITY;
    queueItem.handle                                = handle;
    queueItem.data.setVelocity.velocity_deg_per_sec = velocity_deg_per_sec;
    queueItem.deadline                              = deadline;

    return MTRi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t MTR_set_acceleration_async(MTR_MotorHandle_t handle, double acceleration_deg_per_sec2,
                                    TickType_t deadline, CHD_CmdHandle_t* cmdHandle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                                           = MTR_CMD_SET_ACCELERATION;
    queueItem.handle                                         = handle;
    queueItem.data.setAcceleration.acceleration_deg_per_sec2 = acceleration_deg_per_sec2;
    queueItem.deadline                                       = deadline;

    return MTRi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t MTR_stop_async(MTR_MotorHandle_t handle, bool decelerate, TickType_t deadline,
                        CHD_CmdHandle_t* cmdHandle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                 = MTR_CMD_STOP;
    queueItem.handle               = handle;
    queueItem.data.stop.decelerate = decelerate;
    queueItem.deadline             = deadline;

    return MTRi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t MTR_get_current_angle_async(MTR_MotorHandle_t handle, double* angle, TickType_t deadline,
                                     CHD_CmdHandle_t* cmdHandle)
{
    if (handle == NULL || angle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                       = MTR_CMD_GET_CURRENT_ANGLE;
    queueItem.handle                     = handle;
    queueItem.data.getCurrentAngle.angle = angle;
    queueItem.deadline                   = deadline;

    return MTRi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t MTR_get_movement_state_async(MTR_MotorHandle_t handle, STP_MovementState_t* state,
                                      CHD_CmdHandle_t* cmdHandle)
{
    if (handle == NULL || state == NULL)
    {
        return kStatus_Fail;
    }

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(1);

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                        = MTR_CMD_GET_MOVEMENT_STATE;
    queueItem.handle                      = handle;
    queueItem.data.getMovementState.state = state;
    queueItem.deadline                    = deadline;

    return MTRi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t MTR_set_home_position(MTR_MotorHandle_t handle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    return MTRi_set_home_position(handle);
}

status_t MTR_synchronized_move_async(MTR_MotorHandle_t* handles, double* angles, uint8_t count,
                                     TickType_t deadline, CHD_CmdHandle_t* cmdHandle)
{
    if (handles == NULL || angles == NULL || count == 0)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                          = MTR_CMD_SYNCHRONIZED_MOVE;
    queueItem.handle                        = NULL;
    queueItem.data.synchronizedMove.handles = handles;
    queueItem.data.synchronizedMove.angles  = angles;
    queueItem.data.synchronizedMove.count   = count;
    queueItem.deadline                      = deadline;

    return MTRi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t MTR_set_run_current_async(MTR_MotorHandle_t handle, double current_a, TickType_t deadline,
                                   CHD_CmdHandle_t* cmdHandle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                         = MTR_CMD_SET_RUN_CURRENT;
    queueItem.handle                       = handle;
    queueItem.data.setRunCurrent.current_a = current_a;
    queueItem.deadline                     = deadline;

    return MTRi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t MTR_set_hold_current_async(MTR_MotorHandle_t handle, double current_a, TickType_t deadline,
                                    CHD_CmdHandle_t* cmdHandle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                          = MTR_CMD_SET_HOLD_CURRENT;
    queueItem.handle                        = handle;
    queueItem.data.setHoldCurrent.current_a = current_a;
    queueItem.deadline                      = deadline;

    return MTRi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t MTR_enable_freewheeling_async(MTR_MotorHandle_t handle, TickType_t deadline,
                                       CHD_CmdHandle_t* cmdHandle)
{
    if (handle == NULL || cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type     = MTR_CMD_ENABLE_FREEWHEELING;
    queueItem.handle   = handle;
    queueItem.deadline = deadline;

    return MTRi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t MTR_disable_freewheeling_async(MTR_MotorHandle_t handle, TickType_t deadline,
                                        CHD_CmdHandle_t* cmdHandle)
{
    if (handle == NULL || cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type     = MTR_CMD_DISABLE_FREEWHEELING;
    queueItem.handle   = handle;
    queueItem.deadline = deadline;

    return MTRi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t MTR_emergency_stop(MTR_MotorHandle_t handle)
{
    static char logMsg[100];
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    emergencyStopFlag = 1;
    snprintf(logMsg, sizeof(logMsg),
             "[%s] Emergency stop triggered - all motors will stop immediately", handle->label);
    LOG_WARN(logMsg);

    return kStatus_Success;
}

status_t MTR_clear_emergency_stop(void)
{
    emergencyStopFlag = 0;
    LOG_INFO("Emergency stop flag cleared");
    return kStatus_Success;
}

uint8_t MTR_is_emergency_stop_active(void)
{
    return emergencyStopFlag;
}
