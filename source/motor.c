/************************************************************
 * @file    motor.c
 * @brief   Task-safe motor control module using FreeRTOS queues
 * @author  dg
 * @date    2 Mar 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include "motor.h"
#include "log.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "math.h"
#include "stdio.h"
#include "task_helpers.h"
#include "step.h"

/************************************
 *     Private Macros / Defines    *
 ************************************/

#define MAX_MOTORS 5
#define CMD_QUEUE_SIZE 20

/***************************
 *     Private Typedefs     *
 ***************************/

typedef enum rounding_method
{
    ROUND_DOWN,
    ROUND_UP,
    ROUND_NEAREST
} MTR_roundingMethod_t;

typedef enum _MTR_CmdType
{
    MTR_CMD_MOVE_ANGLE,
    MTR_CMD_MOVE_ABSOLUTE_ANGLE,
    MTR_CMD_MOVE_REVOLUTIONS,
    MTR_CMD_SET_VELOCITY,
    MTR_CMD_SET_ACCELERATION,
    MTR_CMD_STOP,
    MTR_CMD_GET_CURRENT_ANGLE,
    MTR_CMD_GET_MOVEMENT_STATE,
    MTR_CMD_WAIT_UNTIL_STOPPED,
    MTR_CMD_SET_HOME_POSITION,
    MTR_CMD_SYNCHRONIZED_MOVE,
    MTR_CMD_SET_RUN_CURRENT,
    MTR_CMD_SET_HOLD_CURRENT,
    MTR_CMD_INIT_MOTOR
} MTR_CmdType_t;

typedef struct _MTR_MoveAngleCmdData
{
    double angle;
} MTR_MoveAngleCmdData_t;

typedef struct _MTR_MoveRevolutionsCmdData
{
    double revolutions;
} MTR_MoveRevolutionsCmdData_t;

typedef struct _MTR_SetVelocityCmdData
{
    double velocity_deg_per_sec;
} MTR_SetVelocityCmdData_t;

typedef struct _MTR_SetAccelerationCmdData
{
    double acceleration_deg_per_sec2;
} MTR_SetAccelerationCmdData_t;

typedef struct _MTR_StopCmdData
{
    bool decelerate;
} MTR_StopCmdData_t;

typedef struct _MTR_GetCurrentAngleCmdData
{
    double* angle;
} MTR_GetCurrentAngleCmdData_t;

typedef struct _MTR_GetMovementStateCmdData
{
    STP_MovementState_t* state;
} MTR_GetMovementStateCmdData_t;

typedef struct _MTR_WaitUntilStoppedCmdData
{
} MTR_WaitUntilStoppedCmdData_t;

typedef struct _MTR_SynchronizedMoveCmdData
{
    MTR_MotorHandle_t* handles;
    double*            angles;
    uint8_t            count;
} MTR_SynchronizedMoveCmdData_t;

typedef struct _MTR_SetRunCurrentCmdData
{
    double current_a;
} MTR_SetRunCurrentCmdData_t;

typedef struct _MTR_SetHoldCurrentCmdData
{
    double current_a;
} MTR_SetHoldCurrentCmdData_t;

typedef struct _MTR_InitMotorCmdData
{
    MTR_MotorConfig_t config;
} MTR_InitMotorCmdData_t;

typedef struct _MTR_CmdQueueItem
{
    MTR_CmdType_t     type;
    MTR_MotorHandle_t handle;     /**< Pointer to target motor handle */
    TaskHandle_t      taskHandle; /**< Task handle for sending notifications back to caller */
    TickType_t        deadline;   /**< Absolute tick count by which command should complete */
    union
    {
        MTR_MoveAngleCmdData_t        moveAngle;
        MTR_MoveRevolutionsCmdData_t  moveRevolutions;
        MTR_SetVelocityCmdData_t      setVelocity;
        MTR_SetAccelerationCmdData_t  setAcceleration;
        MTR_StopCmdData_t             stop;
        MTR_GetCurrentAngleCmdData_t  getCurrentAngle;
        MTR_GetMovementStateCmdData_t getMovementState;
        MTR_WaitUntilStoppedCmdData_t waitUntilStopped;
        MTR_SynchronizedMoveCmdData_t synchronizedMove;
        MTR_SetRunCurrentCmdData_t    setRunCurrent;
        MTR_SetHoldCurrentCmdData_t   setHoldCurrent;
        MTR_InitMotorCmdData_t        initMotor;
    } data;
} MTR_CmdQueueItem_t;

typedef struct _MTR_MotorHandleImpl
{
    STP_Handle_t stepperHandle;
    TMC_Handle_t tmcHandle;
    double       stepAngle;
    uint8_t      microstep;
    double       reductionFactor;
    char*        label;
} MTR_MotorHandleImpl_t;

typedef struct _MTR_HandlesArrayItem
{
    MTR_MotorHandleImpl_t handle;
    uint8_t               used;
} MTR_HandlesArrayItem_t;

/*****************************************
 *     Private Function Declarations     *
 *****************************************/

static double  steps_to_angle(MTR_MotorHandle_t handle, int32_t steps);
static int32_t angle_to_steps(MTR_MotorHandle_t handle, double angle, MTR_roundingMethod_t method);

static status_t move_angle(MTR_MotorHandle_t handle, double angle, TickType_t deadline);
static status_t move_absolute_angle(MTR_MotorHandle_t handle, double angle, TickType_t deadline);
static status_t move_revolutions(MTR_MotorHandle_t handle, double revolutions, TickType_t deadline);
static status_t set_velocity(MTR_MotorHandle_t handle, double velocity_deg_per_sec,
                             TickType_t deadline);
static status_t set_acceleration(MTR_MotorHandle_t handle, double acceleration_deg_per_sec2,
                                 TickType_t deadline);
static status_t stop_motor(MTR_MotorHandle_t handle, bool decelerate, TickType_t deadline);
static status_t get_current_angle(MTR_MotorHandle_t handle, double* angle, TickType_t deadline);
static status_t get_movement_state(MTR_MotorHandle_t handle, STP_MovementState_t* state,
                                   TickType_t deadline);
static status_t wait_until_stopped(MTR_MotorHandle_t handle, TickType_t deadline);
static status_t set_home_position(MTR_MotorHandle_t handle, TickType_t deadline);
static status_t synchronized_move(MTR_MotorHandle_t* handles, double* angles, uint8_t count,
                                  TickType_t deadline);
static status_t set_run_current(MTR_MotorHandle_t handle, double current_a, TickType_t deadline);
static status_t set_hold_current(MTR_MotorHandle_t handle, double current_a, TickType_t deadline);
static inline status_t calculate_sync_motion_parameters(MTR_MotorHandle_t* handles, double* angles,
                                                        uint8_t count, int32_t* stepCounts,
                                                        double* originalVelocities,
                                                        double* maxDuration, TickType_t deadline);
static inline status_t scale_velocities_for_synchronization(MTR_MotorHandle_t* handles,
                                                            uint8_t count, int32_t* stepCounts,
                                                            double*    originalVelocities,
                                                            double     maxDuration,
                                                            TickType_t deadline);
static inline status_t prepare_synchronized_movements(MTR_MotorHandle_t* handles, double* angles,
                                                      uint8_t count, int32_t* stepCounts,
                                                      double*    originalVelocities,
                                                      TickType_t deadline);
static inline status_t restore_original_velocities(MTR_MotorHandle_t* handles, uint8_t count,
                                                   double* originalVelocities, TickType_t deadline);
static status_t        init_motor(MTR_MotorConfig_t config, TickType_t deadline);
static void            process(void);

/****************************
 *     Public Variables     *
 ****************************/

/*****************************
 *     Private Variables     *
 *****************************/

static MTR_HandlesArrayItem_t motorHandles[MAX_MOTORS];
static QueueHandle_t          cmdQueue = NULL;

static volatile uint8_t emergencyStopFlag = 0;

TaskHandle_t mtrTaskHandle = NULL;

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

void MTR_task(void* pvParameters)
{
    LOG_INFO("Started Motor Task");
    mtrTaskHandle = xTaskGetCurrentTaskHandle();
    for (;;)
    {
        process();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

status_t MTR_init(void)
{

    for (uint8_t i = 0; i < MAX_MOTORS; i++)
    {
        motorHandles[i].used = 0;
    }

    cmdQueue = xQueueCreate(CMD_QUEUE_SIZE, sizeof(MTR_CmdQueueItem_t));
    if (cmdQueue == NULL)
    {
        return kStatus_Fail;
    }

    emergencyStopFlag = 0;
    return kStatus_Success;
}

// TODO add timeout to all queue Send operations
status_t MTR_init_handle(const MTR_MotorConfig_t config, TickType_t deadline)
{
    THE_TASK_SAFE_BEGIN();

    MTR_CmdQueueItem_t queueItem;
    queueItem.handle                = NULL;
    queueItem.type                  = MTR_CMD_INIT_MOTOR;
    queueItem.data.initMotor.config = config;
    queueItem.taskHandle            = callerTaskHandle;
    queueItem.deadline              = deadline;

    TickType_t currentTick = xTaskGetTickCount();
    if (deadline <= currentTick)
    {
        LOG_ERROR("Deadline for motor initialization has already passed");
        return kStatus_Fail;
    }
    TickType_t ticksUntilDeadline = deadline - currentTick;
    if (xQueueSend(cmdQueue, (void*)&queueItem, ticksUntilDeadline) != pdTRUE)
    {
        LOG_WARN("Failed to queue stop command");
        return kStatus_Fail;
    }

    THE_TASK_SAFE_WAIT(deadline);

    return kStatus_Success;
}

void MTR_get_motor_by_label(const char* label, MTR_MotorHandle_t* handle)
{
    if (label == NULL || handle == NULL)
    {
        return;
    }

    for (uint8_t i = 0; i < MAX_MOTORS; i++)
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

status_t MTR_move_angle(MTR_MotorHandle_t handle, double angle, TickType_t deadline)
{
    THE_TASK_SAFE_BEGIN();

    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                 = MTR_CMD_MOVE_ANGLE;
    queueItem.handle               = handle;
    queueItem.data.moveAngle.angle = angle;
    queueItem.taskHandle           = callerTaskHandle;
    queueItem.deadline             = deadline;

    TickType_t currentTick = xTaskGetTickCount();
    if (deadline <= currentTick)
    {
        LOG_ERROR("Deadline for move angle command has already passed");
        return kStatus_Fail;
    }
    TickType_t ticksUntilDeadline = deadline - currentTick;
    if (xQueueSend(cmdQueue, (void*)&queueItem, ticksUntilDeadline) != pdTRUE)
    {
        LOG_WARN("Failed to queue move angle command");
        return kStatus_Fail;
    }

    THE_TASK_SAFE_WAIT(deadline);

    return kStatus_Success;
}

status_t MTR_move_absolute_angle(MTR_MotorHandle_t handle, double angle, TickType_t deadline)
{
    THE_TASK_SAFE_BEGIN();

    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                 = MTR_CMD_MOVE_ABSOLUTE_ANGLE;
    queueItem.handle               = handle;
    queueItem.data.moveAngle.angle = angle;
    queueItem.taskHandle           = callerTaskHandle;
    queueItem.deadline             = deadline;

    TickType_t currentTick = xTaskGetTickCount();
    if (deadline <= currentTick)
    {
        LOG_ERROR("Deadline for move absolute angle command has already passed");
        return kStatus_Fail;
    }
    TickType_t ticksUntilDeadline = deadline - currentTick;
    if (xQueueSend(cmdQueue, (void*)&queueItem, ticksUntilDeadline) != pdTRUE)
    {
        LOG_WARN("Failed to queue move absolute angle command");
        return kStatus_Fail;
    }

    THE_TASK_SAFE_WAIT(deadline);

    return kStatus_Success;
}

status_t MTR_move_revolutions(MTR_MotorHandle_t handle, double revolutions, TickType_t deadline)
{
    THE_TASK_SAFE_BEGIN();

    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                             = MTR_CMD_MOVE_REVOLUTIONS;
    queueItem.handle                           = handle;
    queueItem.data.moveRevolutions.revolutions = revolutions;
    queueItem.taskHandle                       = callerTaskHandle;
    queueItem.deadline                         = deadline;

    TickType_t currentTick = xTaskGetTickCount();
    if (deadline <= currentTick)
    {
        LOG_ERROR("Deadline for move revolutions command has already passed");
        return kStatus_Fail;
    }
    TickType_t ticksUntilDeadline = deadline - currentTick;
    if (xQueueSend(cmdQueue, (void*)&queueItem, ticksUntilDeadline) != pdTRUE)
    {
        LOG_WARN("Failed to queue move revolutions command");
        return kStatus_Fail;
    }

    THE_TASK_SAFE_WAIT(deadline);

    return kStatus_Success;
}

status_t MTR_set_velocity(MTR_MotorHandle_t handle, double velocity_deg_per_sec,
                          TickType_t deadline)
{
    THE_TASK_SAFE_BEGIN()

    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                                  = MTR_CMD_SET_VELOCITY;
    queueItem.handle                                = handle;
    queueItem.data.setVelocity.velocity_deg_per_sec = velocity_deg_per_sec;
    queueItem.taskHandle                            = callerTaskHandle;
    queueItem.deadline                              = deadline;

    TickType_t currentTick = xTaskGetTickCount();
    if (deadline <= currentTick)
    {
        LOG_ERROR("Deadline for set velocity command has already passed");
        return kStatus_Fail;
    }
    TickType_t ticksUntilDeadline = deadline - currentTick;
    if (xQueueSend(cmdQueue, (void*)&queueItem, ticksUntilDeadline) != pdTRUE)
    {
        LOG_WARN("Failed to queue set velocity command");
        return kStatus_Fail;
    }

    THE_TASK_SAFE_WAIT(deadline);

    return kStatus_Success;
}

status_t MTR_set_acceleration(MTR_MotorHandle_t handle, double acceleration_deg_per_sec2,
                              TickType_t deadline)
{
    THE_TASK_SAFE_BEGIN();

    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                                           = MTR_CMD_SET_ACCELERATION;
    queueItem.handle                                         = handle;
    queueItem.data.setAcceleration.acceleration_deg_per_sec2 = acceleration_deg_per_sec2;
    queueItem.taskHandle                                     = callerTaskHandle;
    queueItem.deadline                                       = deadline;

    TickType_t currentTick = xTaskGetTickCount();
    if (deadline <= currentTick)
    {
        LOG_ERROR("Deadline for set acceleration command has already passed");
        return kStatus_Fail;
    }
    TickType_t ticksUntilDeadline = deadline - currentTick;
    if (xQueueSend(cmdQueue, (void*)&queueItem, ticksUntilDeadline) != pdTRUE)
    {
        LOG_WARN("Failed to queue set acceleration command");
        return kStatus_Fail;
    }

    THE_TASK_SAFE_WAIT(deadline);

    return kStatus_Success;
}

status_t MTR_stop(MTR_MotorHandle_t handle, bool decelerate, TickType_t deadline)
{
    THE_TASK_SAFE_BEGIN();

    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                 = MTR_CMD_STOP;
    queueItem.handle               = handle;
    queueItem.data.stop.decelerate = decelerate;
    queueItem.taskHandle           = callerTaskHandle;
    queueItem.deadline             = deadline;

    TickType_t currentTick = xTaskGetTickCount();
    if (deadline <= currentTick)
    {
        LOG_ERROR("Deadline for stop command has already passed");
        return kStatus_Fail;
    }
    TickType_t ticksUntilDeadline = deadline - currentTick;
    if (xQueueSend(cmdQueue, (void*)&queueItem, ticksUntilDeadline) != pdTRUE)
    {
        LOG_WARN("Failed to queue stop command");
        return kStatus_Fail;
    }

    THE_TASK_SAFE_WAIT(deadline);

    return kStatus_Success;
}

status_t MTR_emergency_stop(MTR_MotorHandle_t handle)
{
    static char logMsg[100];
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    // Set emergency stop flag directly (no queue)
    emergencyStopFlag = 1;

    snprintf(logMsg, sizeof(logMsg), "[%s] Emergency stop triggered", handle->label);
    LOG_ERROR(logMsg);

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

status_t MTR_get_current_angle(MTR_MotorHandle_t handle, double* angle, TickType_t deadline)
{
    THE_TASK_SAFE_BEGIN();

    if (handle == NULL || angle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                       = MTR_CMD_GET_CURRENT_ANGLE;
    queueItem.handle                     = handle;
    queueItem.data.getCurrentAngle.angle = angle;
    queueItem.taskHandle                 = callerTaskHandle;
    queueItem.deadline                   = deadline;

    TickType_t currentTick = xTaskGetTickCount();
    if (deadline <= currentTick)
    {
        LOG_ERROR("Deadline for get current angle command has already passed");
        return kStatus_Fail;
    }
    TickType_t ticksUntilDeadline = deadline - currentTick;
    if (xQueueSend(cmdQueue, (void*)&queueItem, ticksUntilDeadline) != pdTRUE)
    {
        LOG_WARN("Failed to queue get current angle command");
        return kStatus_Fail;
    }

    THE_TASK_SAFE_WAIT(deadline);

    return kStatus_Success;
}

status_t MTR_get_movement_state(MTR_MotorHandle_t handle, STP_MovementState_t* state,
                                TickType_t deadline)
{
    THE_TASK_SAFE_BEGIN();

    if (handle == NULL || state == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                        = MTR_CMD_GET_MOVEMENT_STATE;
    queueItem.handle                      = handle;
    queueItem.data.getMovementState.state = state;
    queueItem.taskHandle                  = callerTaskHandle;
    queueItem.deadline                    = deadline;

    TickType_t currentTick = xTaskGetTickCount();
    if (deadline <= currentTick)
    {
        LOG_ERROR("Deadline for get movement state command has already passed");
        return kStatus_Fail;
    }
    TickType_t ticksUntilDeadline = deadline - currentTick;
    if (xQueueSend(cmdQueue, (void*)&queueItem, ticksUntilDeadline) != pdTRUE)
    {
        LOG_WARN("Failed to queue get movement state command");
        return kStatus_Fail;
    }

    THE_TASK_SAFE_WAIT(deadline);

    return kStatus_Success;
}

status_t MTR_wait_until_stopped(MTR_MotorHandle_t handle, TickType_t deadline)
{
    THE_TASK_SAFE_BEGIN();

    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type       = MTR_CMD_WAIT_UNTIL_STOPPED;
    queueItem.handle     = handle;
    queueItem.taskHandle = callerTaskHandle;
    queueItem.deadline   = deadline;

    TickType_t currentTick = xTaskGetTickCount();
    if (deadline <= currentTick)
    {
        LOG_ERROR("Deadline for wait until stopped command has already passed");
        return kStatus_Fail;
    }
    TickType_t ticksUntilDeadline = deadline - currentTick;
    if (xQueueSend(cmdQueue, (void*)&queueItem, ticksUntilDeadline) != pdTRUE)
    {
        LOG_WARN("Failed to queue wait until stopped command");
        return kStatus_Fail;
    }

    THE_TASK_SAFE_WAIT(deadline);

    return kStatus_Success;
}

status_t MTR_set_home_position(MTR_MotorHandle_t handle, TickType_t deadline)
{
    THE_TASK_SAFE_BEGIN();

    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type       = MTR_CMD_SET_HOME_POSITION;
    queueItem.handle     = handle;
    queueItem.deadline   = deadline;
    queueItem.taskHandle = callerTaskHandle;

    TickType_t currentTick = xTaskGetTickCount();
    if (deadline <= currentTick)
    {
        LOG_ERROR("Deadline for set home position command has already passed");
        return kStatus_Fail;
    }
    TickType_t ticksUntilDeadline = deadline - currentTick;
    if (xQueueSend(cmdQueue, (void*)&queueItem, ticksUntilDeadline) != pdTRUE)
    {
        LOG_WARN("Failed to queue set home position command");
        return kStatus_Fail;
    }

    THE_TASK_SAFE_WAIT(deadline);

    return kStatus_Success;
}

status_t MTR_synchronized_move(MTR_MotorHandle_t* handles, double* angles, uint8_t count,
                               TickType_t deadline)
{
    THE_TASK_SAFE_BEGIN();

    if (handles == NULL || angles == NULL || count == 0)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                          = MTR_CMD_SYNCHRONIZED_MOVE;
    queueItem.handle                        = NULL; // No single handle for synchronized move
    queueItem.data.synchronizedMove.handles = handles;
    queueItem.data.synchronizedMove.angles  = angles;
    queueItem.data.synchronizedMove.count   = count;
    queueItem.taskHandle                    = xTaskGetCurrentTaskHandle();
    queueItem.deadline                      = deadline;

    TickType_t currentTick = xTaskGetTickCount();
    if (deadline <= currentTick)
    {
        LOG_ERROR("Deadline for synchronized move command has already passed");
        return kStatus_Fail;
    }
    TickType_t ticksUntilDeadline = deadline - currentTick;
    if (xQueueSend(cmdQueue, (void*)&queueItem, ticksUntilDeadline) != pdTRUE)
    {
        LOG_WARN("Failed to queue synchronized move command");
        return kStatus_Fail;
    }

    THE_TASK_SAFE_WAIT(deadline);

    return kStatus_Success;
}

status_t MTR_set_run_current(MTR_MotorHandle_t handle, double current_a, TickType_t deadline)
{
    THE_TASK_SAFE_BEGIN();

    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                         = MTR_CMD_SET_RUN_CURRENT;
    queueItem.handle                       = handle;
    queueItem.data.setRunCurrent.current_a = current_a;
    queueItem.taskHandle                   = callerTaskHandle;
    queueItem.deadline                     = deadline;

    TickType_t currentTick = xTaskGetTickCount();
    if (deadline <= currentTick)
    {
        LOG_ERROR("Deadline for set run current command has already passed");
        return kStatus_Fail;
    }
    TickType_t ticksUntilDeadline = deadline - currentTick;
    if (xQueueSend(cmdQueue, (void*)&queueItem, ticksUntilDeadline) != pdTRUE)
    {
        LOG_WARN("Failed to queue set run current command");
        return kStatus_Fail;
    }

    THE_TASK_SAFE_WAIT(deadline);

    return kStatus_Success;
}

status_t MTR_set_hold_current(MTR_MotorHandle_t handle, double current_a, TickType_t deadline)
{
    THE_TASK_SAFE_BEGIN();

    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                          = MTR_CMD_SET_HOLD_CURRENT;
    queueItem.handle                        = handle;
    queueItem.data.setHoldCurrent.current_a = current_a;
    queueItem.taskHandle                    = callerTaskHandle;
    queueItem.deadline                      = deadline;

    TickType_t currentTick = xTaskGetTickCount();
    if (deadline <= currentTick)
    {
        LOG_ERROR("Deadline for set hold current command has already passed");
        return kStatus_Fail;
    }
    TickType_t ticksUntilDeadline = deadline - currentTick;
    if (xQueueSend(cmdQueue, (void*)&queueItem, ticksUntilDeadline) != pdTRUE)
    {
        LOG_WARN("Failed to queue set hold current command");
        return kStatus_Fail;
    }

    THE_TASK_SAFE_WAIT(deadline);

    return kStatus_Success;
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/

static void process(void)
{
    MTR_CmdQueueItem_t queueItem;
    BaseType_t         status;

    status = xQueueReceive(cmdQueue, &queueItem, pdMS_TO_TICKS(10));
    if (status != pdPASS)
    {
        return;
    }

    // Check emergency stop flag - if set, don't execute any commands
    if (emergencyStopFlag)
    {
        LOG_WARN("Command ignored due to emergency stop");
        return;
    }
    status_t cmdStatus;
    // Execute command based on type
    switch (queueItem.type)
    {
        case MTR_CMD_MOVE_ANGLE:
            cmdStatus =
                move_angle(queueItem.handle, queueItem.data.moveAngle.angle, queueItem.deadline);
            break;

        case MTR_CMD_MOVE_ABSOLUTE_ANGLE:
            cmdStatus = move_absolute_angle(queueItem.handle, queueItem.data.moveAngle.angle,
                                            queueItem.deadline);
            break;

        case MTR_CMD_MOVE_REVOLUTIONS:
            cmdStatus = move_revolutions(
                queueItem.handle, queueItem.data.moveRevolutions.revolutions, queueItem.deadline);
            break;

        case MTR_CMD_SET_VELOCITY:
            cmdStatus =
                set_velocity(queueItem.handle, queueItem.data.setVelocity.velocity_deg_per_sec,
                             queueItem.deadline);
            break;

        case MTR_CMD_SET_ACCELERATION:
            cmdStatus = set_acceleration(queueItem.handle,
                                         queueItem.data.setAcceleration.acceleration_deg_per_sec2,
                                         queueItem.deadline);
            break;

        case MTR_CMD_STOP:
            cmdStatus =
                stop_motor(queueItem.handle, queueItem.data.stop.decelerate, queueItem.deadline);
            break;

        case MTR_CMD_GET_CURRENT_ANGLE:
            cmdStatus = get_current_angle(queueItem.handle, queueItem.data.getCurrentAngle.angle,
                                          queueItem.deadline);
            break;

        case MTR_CMD_GET_MOVEMENT_STATE:
            cmdStatus = get_movement_state(queueItem.handle, queueItem.data.getMovementState.state,
                                           queueItem.deadline);
            break;

        case MTR_CMD_WAIT_UNTIL_STOPPED:
            cmdStatus = wait_until_stopped(queueItem.handle, queueItem.deadline);
            break;

        case MTR_CMD_SET_HOME_POSITION:
            cmdStatus = set_home_position(queueItem.handle, queueItem.deadline);
            break;

        case MTR_CMD_SYNCHRONIZED_MOVE:
            cmdStatus = synchronized_move(
                queueItem.data.synchronizedMove.handles, queueItem.data.synchronizedMove.angles,
                queueItem.data.synchronizedMove.count, queueItem.deadline);
            break;

        case MTR_CMD_SET_RUN_CURRENT:
            cmdStatus = set_run_current(queueItem.handle, queueItem.data.setRunCurrent.current_a,
                                        queueItem.deadline);
            break;

        case MTR_CMD_SET_HOLD_CURRENT:
            cmdStatus = set_hold_current(queueItem.handle, queueItem.data.setHoldCurrent.current_a,
                                         queueItem.deadline);
            break;
        case MTR_CMD_INIT_MOTOR:
            cmdStatus = init_motor(queueItem.data.initMotor.config, queueItem.deadline);
            break;
        default:
            LOG_WARN("Unknown motor command type");
            break;
    }
    if (queueItem.deadline > xTaskGetTickCount() && cmdStatus == kStatus_Success)
    {
        THE_notify_task_success(queueItem.taskHandle);
    }
    else
    {
        THE_notify_task_failure(queueItem.taskHandle);
    }
}

static status_t init_motor(MTR_MotorConfig_t config, TickType_t deadline)
{
    static char logMsg[100];

    if (config.stepAngle <= 0.0 || config.microstep == 0 || config.reductionFactor <= 0.0)
    {
        LOG_ERROR(
            "Invalid motor configuration: stepAngle, microstep, or reductionFactor out of range");
        return kStatus_Fail;
    }

    STP_StepperConfig_t stepperConfig;
    STP_get_default_config(&stepperConfig);

    stepperConfig.ftmBase               = config.stepperConfig.ftmBase;
    stepperConfig.ftmChannel            = config.stepperConfig.ftmChannel;
    stepperConfig.stepPort              = config.stepperConfig.stepPort;
    stepperConfig.stepGPIO              = config.stepperConfig.stepGPIO;
    stepperConfig.stepPin               = config.stepperConfig.stepPin;
    stepperConfig.stepMuxFTM            = config.stepperConfig.stepMuxFTM;
    stepperConfig.stepMuxGPIO           = config.stepperConfig.stepMuxGPIO;
    stepperConfig.dirPort               = config.stepperConfig.dirPort;
    stepperConfig.dirGPIO               = config.stepperConfig.dirGPIO;
    stepperConfig.dirPin                = config.stepperConfig.dirPin;
    stepperConfig.dirMux                = config.stepperConfig.dirMux;
    stepperConfig.dirLogicHighClockwise = config.stepperConfig.dirLogicHighClockwise;
    stepperConfig.label                 = config.label;

    stepperConfig.acceleration =
        config.acceleration * config.reductionFactor * config.microstep / config.stepAngle;
    stepperConfig.endVelocity =
        config.endVelocity * config.reductionFactor * config.microstep / config.stepAngle;

    if (STP_init_handle(stepperConfig, deadline) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    // Get the stepper handle that was just created
    STP_Handle_t stepperHandle = NULL;
    if (STP_get_handle_by_label(config.label, &stepperHandle) != kStatus_Success ||
        stepperHandle == NULL)
    {
        LOG_ERROR("Failed to get stepper handle after initialization");
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

    if (TMC_init_handle(tmcConfig, deadline) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    TMC_Handle_t tmcHandle = NULL;
    if (TMC_get_handle_by_label(config.label, &tmcHandle) != kStatus_Success || tmcHandle == NULL)
    {
        LOG_ERROR("Failed to get tmc handle after initialization");
        return kStatus_Fail;
    }

    // Find free motor handle slot
    uint8_t handleIndex = MAX_MOTORS;
    for (uint8_t i = 0; i < MAX_MOTORS; i++)
    {
        if (motorHandles[i].used == 0)
        {
            motorHandles[i].used = 1;
            handleIndex          = i;
            break;
        }
    }

    if (handleIndex == MAX_MOTORS)
    {
        LOG_ERROR("No free motor handle slots available");
        return kStatus_Fail;
    }

    MTR_MotorHandle_t newHandle = &motorHandles[handleIndex].handle;
    newHandle->stepperHandle    = stepperHandle;
    newHandle->tmcHandle        = tmcHandle;
    newHandle->stepAngle        = config.stepAngle;
    newHandle->microstep        = config.microstep;
    newHandle->reductionFactor  = config.reductionFactor;
    newHandle->label            = config.label;

    snprintf(logMsg, sizeof(logMsg), "[%s] Motor handle initialized", config.label);
    LOG_INFO(logMsg);

    return kStatus_Success;
}

static double steps_to_angle(MTR_MotorHandle_t handle, int32_t steps)
{
    if (handle == NULL)
        return 0.0;

    if (handle->stepAngle <= 0.0 || handle->microstep == 0 || handle->reductionFactor <= 0.0)
        return 0.0;

    double angle = (double)steps * handle->stepAngle;
    angle        = angle / (double)handle->microstep;
    angle        = angle / handle->reductionFactor;
    return angle;
}

static int32_t angle_to_steps(MTR_MotorHandle_t handle, double angle, MTR_roundingMethod_t method)
{
    if (handle == NULL)
        return 0;

    if (handle->stepAngle <= 0.0 || handle->microstep == 0 || handle->reductionFactor <= 0.0)
        return 0;

    double steps = angle * handle->reductionFactor;
    steps        = steps * (double)handle->microstep;
    steps        = steps / handle->stepAngle;

    // Clamp to int32_t range
    if (steps < (double)INT32_MIN)
        steps = (double)INT32_MIN;
    if (steps > (double)INT32_MAX)
        steps = (double)INT32_MAX;

    int32_t result;
    switch (method)
    {
        case ROUND_DOWN:
            result = (int32_t)floor(steps);
            break;
        case ROUND_UP:
            result = (int32_t)ceil(steps);
            break;
        case ROUND_NEAREST:
            result = (int32_t)round(steps);
            break;
        default:
            result = (int32_t)round(steps);
            break;
    }
    return result;
}

static status_t move_angle(MTR_MotorHandle_t handle, double angle, TickType_t deadline)
{
    static char logMsg[100];
    if (emergencyStopFlag)
        return kStatus_Fail;
    if (handle == NULL)
        return kStatus_Fail;
    snprintf(logMsg, sizeof(logMsg), "[%s] Moving by %.2f degrees", handle->label, angle);
    LOG_DEBUG(logMsg);

    // Convert angle to steps (sign is preserved)
    int32_t steps = angle_to_steps(handle, angle, ROUND_NEAREST);

    if (STP_move_relative(handle->stepperHandle, steps, deadline) != kStatus_Success)
    {
        LOG_ERROR("Failed to move stepper motor");
        return kStatus_Fail;
    }

    return kStatus_Success;
}

static status_t move_absolute_angle(MTR_MotorHandle_t handle, double angle, TickType_t deadline)
{
    static char logMsg[100];
    if (emergencyStopFlag)
        return kStatus_Fail;
    if (handle == NULL)
        return kStatus_Fail;
    snprintf(logMsg, sizeof(logMsg), "[%s] Moving to absolute angle %.2f degrees", handle->label,
             angle);
    LOG_DEBUG(logMsg);

    // Get current position, calculate relative movement, and execute
    int32_t currentSteps;
    STP_get_absolute_steps(handle->stepperHandle, &currentSteps, deadline);
    int32_t targetSteps = angle_to_steps(handle, angle, ROUND_NEAREST);
    int32_t stepsToMove = targetSteps - currentSteps;

    if (STP_move_relative(handle->stepperHandle, stepsToMove, deadline) != kStatus_Success)
    {
        LOG_ERROR("Failed to move stepper motor");
        return kStatus_Fail;
    }

    return kStatus_Success;
}

static status_t move_revolutions(MTR_MotorHandle_t handle, double revolutions, TickType_t deadline)
{
    static char logMsg[100];
    if (emergencyStopFlag)
        return kStatus_Fail;
    if (handle == NULL)
        return kStatus_Fail;
    double angle = revolutions * 360.0;
    snprintf(logMsg, sizeof(logMsg), "[%s] Moving %.2f revolutions (%.2f degrees)", handle->label,
             revolutions, angle);
    LOG_DEBUG(logMsg);

    return move_angle(handle, angle, deadline);
}

static status_t set_velocity(MTR_MotorHandle_t handle, double velocity_deg_per_sec,
                             TickType_t deadline)
{
    static char logMsg[100];
    if (emergencyStopFlag)
        return kStatus_Fail;
    if (handle == NULL)
        return kStatus_Fail;
    snprintf(logMsg, sizeof(logMsg), "[%s] Setting velocity to %.2f deg/sec", handle->label,
             velocity_deg_per_sec);
    LOG_DEBUG(logMsg);

    STP_set_end_velocity(handle->stepperHandle,
                         angle_to_steps(handle, fabs(velocity_deg_per_sec), ROUND_NEAREST),
                         deadline);

    return kStatus_Success;
}

static status_t set_acceleration(MTR_MotorHandle_t handle, double acceleration_deg_per_sec2,
                                 TickType_t deadline)
{
    static char logMsg[100];
    if (emergencyStopFlag)
        return kStatus_Fail;
    if (handle == NULL)
        return kStatus_Fail;
    snprintf(logMsg, sizeof(logMsg), "[%s] Setting acceleration to %.2f deg/sec²", handle->label,
             acceleration_deg_per_sec2);
    LOG_DEBUG(logMsg);

    STP_set_acceleration(handle->stepperHandle,
                         angle_to_steps(handle, fabs(acceleration_deg_per_sec2), ROUND_NEAREST),
                         deadline);

    return kStatus_Success;
}

static status_t stop_motor(MTR_MotorHandle_t handle, bool decelerate, TickType_t deadline)
{
    static char logMsg[100];
    if (emergencyStopFlag)
        return kStatus_Fail;
    if (handle == NULL)
        return kStatus_Fail;
    snprintf(logMsg, sizeof(logMsg), "[%s] Stopping motor (decelerate=%d)", handle->label,
             decelerate);
    LOG_DEBUG(logMsg);

    if (STP_stop(handle->stepperHandle, decelerate, deadline) != kStatus_Success)
    {
        LOG_ERROR("Failed to stop stepper motor");
        return kStatus_Fail;
    }

    return kStatus_Success;
}

static status_t get_current_angle(MTR_MotorHandle_t handle, double* angle, TickType_t deadline)
{
    if (emergencyStopFlag)
        return kStatus_Fail;
    if (handle == NULL || angle == NULL)
        return kStatus_Fail;

    int32_t absoluteSteps;
    if (STP_get_absolute_steps(handle->stepperHandle, &absoluteSteps, deadline) != kStatus_Success)
        return kStatus_Fail;
    *angle = steps_to_angle(handle, absoluteSteps);

    return kStatus_Success;
}

static status_t get_movement_state(MTR_MotorHandle_t handle, STP_MovementState_t* state,
                                   TickType_t deadline)
{
    if (emergencyStopFlag)
        return kStatus_Fail;
    if (handle == NULL || state == NULL)
        return kStatus_Fail;

    return STP_get_movement_state(handle->stepperHandle, state, deadline);
}

static status_t wait_until_stopped(MTR_MotorHandle_t handle, TickType_t deadline)
{
    if (emergencyStopFlag)
        return kStatus_Fail;
    if (handle == NULL)
        return kStatus_Fail;

    // TODO: Implement wait until stopped

    while (deadline > xTaskGetTickCount())
    {
        STP_MovementState_t state;
        if (get_movement_state(handle, &state, deadline) != kStatus_Success)
        {
            LOG_ERROR("Failed to get movement state");
            return kStatus_Fail;
        }

        if (state == STP_MOVEMENT_IDLE || state == STP_MOVEMENT_STOPPED ||
            state == STP_MOVEMENT_FINISHED)
        {
            return kStatus_Success;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    LOG_ERROR("Timeout waiting for motor to stop");
    return kStatus_Fail;
}

static status_t set_home_position(MTR_MotorHandle_t handle, TickType_t deadline)
{
    static char logMsg[100];
    if (emergencyStopFlag)
        return kStatus_Fail;
    if (handle == NULL)
        return kStatus_Fail;
    snprintf(logMsg, sizeof(logMsg), "[%s] Setting home position", handle->label);
    LOG_DEBUG(logMsg);

    STP_reset_absolute_position(handle->stepperHandle, deadline);

    return kStatus_Success;
}

static inline status_t calculate_sync_motion_parameters(MTR_MotorHandle_t* handles, double* angles,
                                                        uint8_t count, int32_t* stepCounts,
                                                        double* originalVelocities,
                                                        double* maxDuration, TickType_t deadline)
{
    *maxDuration = 0.0;

    for (uint8_t i = 0; i < count; i++)
    {
        if (handles[i] == NULL)
            return kStatus_Fail;

        stepCounts[i] = angle_to_steps(handles[i], angles[i], ROUND_NEAREST);
        if (STP_get_end_velocity(handles[i]->stepperHandle, &originalVelocities[i], deadline) !=
            kStatus_Success)
            return kStatus_Fail;

        if (originalVelocities[i] <= 0)
            return kStatus_Fail;

        // Use absolute value of steps for duration calculation
        uint32_t absSteps     = (stepCounts[i] >= 0) ? stepCounts[i] : -stepCounts[i];
        double   moveDuration = (double)absSteps / originalVelocities[i];
        if (moveDuration > *maxDuration)
            *maxDuration = moveDuration;
    }

    return kStatus_Success;
}

static inline status_t scale_velocities_for_synchronization(MTR_MotorHandle_t* handles,
                                                            uint8_t count, int32_t* stepCounts,
                                                            double* originalVelocities,
                                                            double maxDuration, TickType_t deadline)
{
    for (uint8_t i = 0; i < count; i++)
    {
        if (stepCounts[i] != 0)
        {
            uint32_t absSteps    = (stepCounts[i] >= 0) ? stepCounts[i] : -stepCounts[i];
            double   newVelocity = (double)absSteps / maxDuration;

            if (newVelocity > originalVelocities[i])
                newVelocity = originalVelocities[i];

            if (STP_set_end_velocity(handles[i]->stepperHandle, newVelocity, deadline) !=
                kStatus_Success)
                return kStatus_Fail;
        }
    }
    return kStatus_Success;
}

static inline status_t prepare_synchronized_movements(MTR_MotorHandle_t* handles, double* angles,
                                                      uint8_t count, int32_t* stepCounts,
                                                      double*    originalVelocities,
                                                      TickType_t deadline)
{
    for (uint8_t i = 0; i < count; i++)
    {
        if (stepCounts[i] != 0)
        {
            status_t result =
                STP_move_relative_prepare(handles[i]->stepperHandle, stepCounts[i], deadline);

            if (result != kStatus_Success)
            {
                restore_original_velocities(handles, count, originalVelocities, deadline);
                return kStatus_Fail;
            }
        }
    }

    return kStatus_Success;
}

static inline status_t restore_original_velocities(MTR_MotorHandle_t* handles, uint8_t count,
                                                   double* originalVelocities, TickType_t deadline)
{
    for (uint8_t i = 0; i < count; i++)
    {
        if (STP_set_end_velocity(handles[i]->stepperHandle, originalVelocities[i], deadline) !=
            kStatus_Success)
            return kStatus_Fail;
    }
    return kStatus_Success;
}

static status_t synchronized_move(MTR_MotorHandle_t* handles, double* angles, uint8_t count,
                                  TickType_t deadline)
{
    static char logMsg[100];
    if (emergencyStopFlag)
        return kStatus_Fail;
    if (handles == NULL || angles == NULL || count == 0)
        return kStatus_Fail;

    int32_t stepCounts[count];
    double  originalVelocities[count];
    double  maxDuration;

    if (calculate_sync_motion_parameters(handles, angles, count, stepCounts, originalVelocities,
                                         &maxDuration, deadline) != kStatus_Success)
        return kStatus_Fail;

    scale_velocities_for_synchronization(handles, count, stepCounts, originalVelocities,
                                         maxDuration, deadline);

    snprintf(logMsg, sizeof(logMsg), "Synchronized move: %d motors, duration=%.3fs", count,
             maxDuration);
    LOG_DEBUG(logMsg);

    if (prepare_synchronized_movements(handles, angles, count, stepCounts, originalVelocities,
                                       deadline) != kStatus_Success)
        return kStatus_Fail;

    status_t triggerResult = STP_trigger_prepared_moves(deadline);
    if (triggerResult != kStatus_Success)
    {
        restore_original_velocities(handles, count, originalVelocities, deadline);
        return kStatus_Fail;
    }

    restore_original_velocities(handles, count, originalVelocities, deadline);

    return kStatus_Success;
}

static status_t set_run_current(MTR_MotorHandle_t handle, double current_a, TickType_t deadline)
{
    static char logMsg[100];
    if (emergencyStopFlag)
        return kStatus_Fail;
    if (handle == NULL)
        return kStatus_Fail;

    uint8_t divider = 0;
    if (TMC_current_to_divider((float)current_a, TMC_ROUND_NEAREST, &divider) != kStatus_Success)
    {
        LOG_ERROR("Failed to convert run current to TMC divider");
        return kStatus_Fail;
    }

    if (TMC_set_irun_divider(handle->tmcHandle, divider, deadline) != kStatus_Success)
    {
        LOG_ERROR("Failed to queue TMC set irun divider command");
        return kStatus_Fail;
    }

    snprintf(logMsg, sizeof(logMsg), "[%s] Set run current requested: %.3f A", handle->label,
             current_a);
    LOG_DEBUG(logMsg);

    return kStatus_Success;
}

static status_t set_hold_current(MTR_MotorHandle_t handle, double current_a, TickType_t deadline)
{
    static char logMsg[100];
    if (emergencyStopFlag)
        return kStatus_Fail;
    if (handle == NULL)
        return kStatus_Fail;

    uint8_t divider = 0;
    if (TMC_current_to_divider((float)current_a, TMC_ROUND_NEAREST, &divider) != kStatus_Success)
    {
        LOG_ERROR("Failed to convert hold current to TMC divider");
        return kStatus_Fail;
    }

    if (TMC_set_ihold_divider(handle->tmcHandle, divider, deadline) != kStatus_Success)
    {
        LOG_ERROR("Failed to queue TMC set ihold divider command");
        return kStatus_Fail;
    }

    snprintf(logMsg, sizeof(logMsg), "[%s] Set hold current requested: %.3f A", handle->label,
             current_a);
    LOG_DEBUG(logMsg);

    return kStatus_Success;
}
