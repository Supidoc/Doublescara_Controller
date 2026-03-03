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
} rounding_method_t;

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
    MTR_CMD_SYNCHRONIZED_MOVE
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
    uint32_t timeout_ms;
} MTR_WaitUntilStoppedCmdData_t;

typedef struct _MTR_SynchronizedMoveCmdData
{
    MTR_MotorHandle_t** handles;
    double*             angles;
    uint8_t             count;
} MTR_SynchronizedMoveCmdData_t;

typedef struct _MTR_HandlesArrayItem
{
    MTR_MotorHandle_t handle;
    uint8_t           used;
} MTR_HandlesArrayItem_t;

typedef struct _MTR_CmdQueueItem
{
    MTR_CmdType_t      type;
    MTR_MotorHandle_t* handle; /**< Pointer to target motor handle */
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
    } data;
} MTR_CmdQueueItem_t;

/*****************************************
 *     Private Function Declarations     *
 *****************************************/

static double  steps_to_angle(MTR_MotorHandle_t* handle, int32_t steps);
static int32_t angle_to_steps(MTR_MotorHandle_t* handle, double angle, rounding_method_t method);

static status_t move_angle(MTR_MotorHandle_t* handle, double angle);
static status_t move_absolute_angle(MTR_MotorHandle_t* handle, double angle);
static status_t move_revolutions(MTR_MotorHandle_t* handle, double revolutions);
static status_t set_velocity(MTR_MotorHandle_t* handle, double velocity_deg_per_sec);
static status_t set_acceleration(MTR_MotorHandle_t* handle, double acceleration_deg_per_sec2);
static status_t stop_motor(MTR_MotorHandle_t* handle, bool decelerate);
static status_t get_current_angle(MTR_MotorHandle_t* handle, double* angle);
static status_t get_movement_state(MTR_MotorHandle_t* handle, STP_MovementState_t* state);
static status_t wait_until_stopped(MTR_MotorHandle_t* handle, uint32_t timeout_ms);
static status_t set_home_position(MTR_MotorHandle_t* handle);
static status_t synchronized_move(MTR_MotorHandle_t** handles, double* angles, uint8_t count);
static inline status_t calculate_sync_motion_parameters(MTR_MotorHandle_t** handles, double* angles,
                                                        uint8_t count, int32_t* stepCounts,
                                                        double* originalVelocities,
                                                        double* maxDuration);
static inline void scale_velocities_for_synchronization(MTR_MotorHandle_t** handles, uint8_t count,
                                                        int32_t* stepCounts,
                                                        double*  originalVelocities,
                                                        double   maxDuration);
static inline status_t prepare_synchronized_movements(MTR_MotorHandle_t** handles, double* angles,
                                                      uint8_t count, int32_t* stepCounts,
                                                      double* originalVelocities);
static inline void     restore_original_velocities(MTR_MotorHandle_t** handles, uint8_t count,
                                                   double* originalVelocities);

static void process_command_queue(void);

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
        process_command_queue();
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
        LOG_ERROR("Failed to create motor command queue");
        return kStatus_Fail;
    }

    emergencyStopFlag = 0;
    LOG_INFO("Motor module initialized");
    return kStatus_Success;
}

status_t MTR_init_motor(const MTR_MotorConfig_t* config, MTR_MotorHandle_t** handle)
{
    static char logMsg[100];
    if (config == NULL || handle == NULL)
    {
        LOG_ERROR("Invalid parameters for motor initialization");
        return kStatus_Fail;
    }

    if (config->stepAngle <= 0.0 || config->microstep == 0 || config->reductionFactor <= 0.0)
    {
        LOG_ERROR(
            "Invalid motor configuration: stepAngle, microstep, or reductionFactor out of range");
        return kStatus_Fail;
    }

    MTR_MotorHandle_t* newHandle   = NULL;
    int8_t             handleIndex = -1;
    for (uint8_t i = 0; i < MAX_MOTORS; i++)
    {
        if (motorHandles[i].used == 0)
        {
            motorHandles[i].used = 1;
            newHandle            = &motorHandles[i].handle;
            handleIndex          = i;
            break;
        }
    }

    if (newHandle == NULL)
    {
        LOG_ERROR("Failed to allocate motor handle - too many motors");
        return kStatus_Fail;
    }

    // Copy configuration to handle
    // Initialize the stepper motor with its configuration
    status_t stepperStatus = STP_init_stepper(config->stepperHandle);
    if (stepperStatus != kStatus_Success)
    {
        LOG_ERROR("Failed to initialize stepper motor");
        motorHandles[handleIndex].used = 0;
        return kStatus_Fail;
    }

    // Get the stepper handle that was just created
    STP_StepperHandle_t* stepperHandle = NULL;
    status_t getHandleStatus = STP_get_handle_by_label(config->stepperHandle.label, &stepperHandle);
    if (getHandleStatus != kStatus_Success || stepperHandle == NULL)
    {
        LOG_ERROR("Failed to get stepper handle after initialization");
        motorHandles[handleIndex].used = 0;
        return kStatus_Fail;
    }

    newHandle->stepperHandle   = *stepperHandle;
    newHandle->tmcHandle       = config->tmcHandle;
    newHandle->stepAngle       = config->stepAngle;
    newHandle->microstep       = config->microstep;
    newHandle->reductionFactor = config->reductionFactor;
    newHandle->label           = config->label;

    if (TMC_init_default(&newHandle->tmcHandle, mtrTaskHandle) != kStatus_Success)
    {
        motorHandles[handleIndex].used = 0;
        LOG_ERROR("Failed to initialize TMC handle");
        return kStatus_Fail;
    }

    uint32_t notification = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000));
    if (notification == 0)
    {
        LOG_ERROR("TMC init timed out");
        return kStatus_Fail;
    }
    else
    {
        LOG_INFO("TMC init completed successfully");
    }

    TMC_MICROSTEPPING_t microstepping;
    TMC_microstepping_value_to_enum(config->microstep, &microstepping);

    if (TMC_set_microstepping(&newHandle->tmcHandle, microstepping, mtrTaskHandle) !=
        kStatus_Success)
    {
        motorHandles[handleIndex].used = 0;
        LOG_ERROR("Failed to set TMC microstepping");
        return kStatus_Fail;
    }

    notification = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000));
    if (notification == 0)
    {
        LOG_ERROR("TMC set microstepping timed out");
        return kStatus_Fail;
    }
    else
    {
        LOG_INFO("TMC set microstepping completed successfully");
    }

    *handle = newHandle;

    snprintf(logMsg, sizeof(logMsg), "[%s] Motor handle initialized", config->label);
    LOG_INFO(logMsg);

    return kStatus_Success;
}

void MTR_get_motor_by_label(const char* label, MTR_MotorHandle_t** handle)
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

status_t MTR_move_angle(MTR_MotorHandle_t* handle, double angle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                 = MTR_CMD_MOVE_ANGLE;
    queueItem.handle               = handle;
    queueItem.data.moveAngle.angle = angle;

    if (xQueueSend(cmdQueue, (void*)&queueItem, portMAX_DELAY) != pdTRUE)
    {
        LOG_WARN("Failed to queue move angle command");
        return kStatus_Fail;
    }
    return kStatus_Success;
}

status_t MTR_move_absolute_angle(MTR_MotorHandle_t* handle, double angle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                 = MTR_CMD_MOVE_ABSOLUTE_ANGLE;
    queueItem.handle               = handle;
    queueItem.data.moveAngle.angle = angle;

    if (xQueueSend(cmdQueue, (void*)&queueItem, portMAX_DELAY) != pdTRUE)
    {
        LOG_WARN("Failed to queue move absolute angle command");
        return kStatus_Fail;
    }
    return kStatus_Success;
}

status_t MTR_move_revolutions(MTR_MotorHandle_t* handle, double revolutions)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                             = MTR_CMD_MOVE_REVOLUTIONS;
    queueItem.handle                           = handle;
    queueItem.data.moveRevolutions.revolutions = revolutions;

    if (xQueueSend(cmdQueue, (void*)&queueItem, portMAX_DELAY) != pdTRUE)
    {
        LOG_WARN("Failed to queue move revolutions command");
        return kStatus_Fail;
    }
    return kStatus_Success;
}

status_t MTR_set_velocity(MTR_MotorHandle_t* handle, double velocity_deg_per_sec)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                                  = MTR_CMD_SET_VELOCITY;
    queueItem.handle                                = handle;
    queueItem.data.setVelocity.velocity_deg_per_sec = velocity_deg_per_sec;

    if (xQueueSend(cmdQueue, (void*)&queueItem, portMAX_DELAY) != pdTRUE)
    {
        LOG_WARN("Failed to queue set velocity command");
        return kStatus_Fail;
    }
    return kStatus_Success;
}

status_t MTR_set_acceleration(MTR_MotorHandle_t* handle, double acceleration_deg_per_sec2)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                                           = MTR_CMD_SET_ACCELERATION;
    queueItem.handle                                         = handle;
    queueItem.data.setAcceleration.acceleration_deg_per_sec2 = acceleration_deg_per_sec2;

    if (xQueueSend(cmdQueue, (void*)&queueItem, portMAX_DELAY) != pdTRUE)
    {
        LOG_WARN("Failed to queue set acceleration command");
        return kStatus_Fail;
    }
    return kStatus_Success;
}

status_t MTR_stop(MTR_MotorHandle_t* handle, bool decelerate)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                 = MTR_CMD_STOP;
    queueItem.handle               = handle;
    queueItem.data.stop.decelerate = decelerate;

    if (xQueueSend(cmdQueue, (void*)&queueItem, portMAX_DELAY) != pdTRUE)
    {
        LOG_WARN("Failed to queue stop command");
        return kStatus_Fail;
    }
    return kStatus_Success;
}

status_t MTR_emergency_stop(MTR_MotorHandle_t* handle)
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

status_t MTR_get_current_angle(MTR_MotorHandle_t* handle, double* angle)
{
    if (handle == NULL || angle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                       = MTR_CMD_GET_CURRENT_ANGLE;
    queueItem.handle                     = handle;
    queueItem.data.getCurrentAngle.angle = angle;

    if (xQueueSend(cmdQueue, (void*)&queueItem, portMAX_DELAY) != pdTRUE)
    {
        LOG_WARN("Failed to queue get current angle command");
        return kStatus_Fail;
    }
    return kStatus_Success;
}

status_t MTR_get_movement_state(MTR_MotorHandle_t* handle, STP_MovementState_t* state)
{
    if (handle == NULL || state == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                        = MTR_CMD_GET_MOVEMENT_STATE;
    queueItem.handle                      = handle;
    queueItem.data.getMovementState.state = state;

    if (xQueueSend(cmdQueue, (void*)&queueItem, portMAX_DELAY) != pdTRUE)
    {
        LOG_WARN("Failed to queue get movement state command");
        return kStatus_Fail;
    }
    return kStatus_Success;
}

status_t MTR_wait_until_stopped(MTR_MotorHandle_t* handle, uint32_t timeout_ms)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type                             = MTR_CMD_WAIT_UNTIL_STOPPED;
    queueItem.handle                           = handle;
    queueItem.data.waitUntilStopped.timeout_ms = timeout_ms;

    if (xQueueSend(cmdQueue, (void*)&queueItem, portMAX_DELAY) != pdTRUE)
    {
        LOG_WARN("Failed to queue wait until stopped command");
        return kStatus_Fail;
    }
    return kStatus_Success;
}

status_t MTR_set_home_position(MTR_MotorHandle_t* handle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    MTR_CmdQueueItem_t queueItem;
    queueItem.type   = MTR_CMD_SET_HOME_POSITION;
    queueItem.handle = handle;

    if (xQueueSend(cmdQueue, (void*)&queueItem, portMAX_DELAY) != pdTRUE)
    {
        LOG_WARN("Failed to queue set home position command");
        return kStatus_Fail;
    }
    return kStatus_Success;
}

status_t MTR_synchronized_move(MTR_MotorHandle_t** handles, double* angles, uint8_t count)
{
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

    if (xQueueSend(cmdQueue, (void*)&queueItem, portMAX_DELAY) != pdTRUE)
    {
        LOG_WARN("Failed to queue synchronized move command");
        return kStatus_Fail;
    }
    return kStatus_Success;
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/

static void process_command_queue(void)
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

    // Execute command based on type
    switch (queueItem.type)
    {
        case MTR_CMD_MOVE_ANGLE:
            move_angle(queueItem.handle, queueItem.data.moveAngle.angle);
            break;

        case MTR_CMD_MOVE_ABSOLUTE_ANGLE:
            move_absolute_angle(queueItem.handle, queueItem.data.moveAngle.angle);
            break;

        case MTR_CMD_MOVE_REVOLUTIONS:
            move_revolutions(queueItem.handle, queueItem.data.moveRevolutions.revolutions);
            break;

        case MTR_CMD_SET_VELOCITY:
            set_velocity(queueItem.handle, queueItem.data.setVelocity.velocity_deg_per_sec);
            break;

        case MTR_CMD_SET_ACCELERATION:
            set_acceleration(queueItem.handle,
                             queueItem.data.setAcceleration.acceleration_deg_per_sec2);
            break;

        case MTR_CMD_STOP:
            stop_motor(queueItem.handle, queueItem.data.stop.decelerate);
            break;

        case MTR_CMD_GET_CURRENT_ANGLE:
            get_current_angle(queueItem.handle, queueItem.data.getCurrentAngle.angle);
            break;

        case MTR_CMD_GET_MOVEMENT_STATE:
            get_movement_state(queueItem.handle, queueItem.data.getMovementState.state);
            break;

        case MTR_CMD_WAIT_UNTIL_STOPPED:
            wait_until_stopped(queueItem.handle, queueItem.data.waitUntilStopped.timeout_ms);
            break;

        case MTR_CMD_SET_HOME_POSITION:
            set_home_position(queueItem.handle);
            break;

        case MTR_CMD_SYNCHRONIZED_MOVE:
            synchronized_move(queueItem.data.synchronizedMove.handles,
                              queueItem.data.synchronizedMove.angles,
                              queueItem.data.synchronizedMove.count);
            break;

        default:
            LOG_WARN("Unknown motor command type");
            break;
    }
}

static double steps_to_angle(MTR_MotorHandle_t* handle, int32_t steps)
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

static int32_t angle_to_steps(MTR_MotorHandle_t* handle, double angle, rounding_method_t method)
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

static status_t move_angle(MTR_MotorHandle_t* handle, double angle)
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

    if (STP_move_relative(&handle->stepperHandle, steps) != kStatus_Success)
    {
        LOG_ERROR("Failed to move stepper motor");
        return kStatus_Fail;
    }

    return kStatus_Success;
}

static status_t move_absolute_angle(MTR_MotorHandle_t* handle, double angle)
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
    int32_t currentSteps = handle->stepperHandle.absolutePosition;
    int32_t targetSteps  = angle_to_steps(handle, angle, ROUND_NEAREST);
    int32_t stepsToMove  = targetSteps - currentSteps;

    if (STP_move_relative(&handle->stepperHandle, stepsToMove) != kStatus_Success)
    {
        LOG_ERROR("Failed to move stepper motor");
        return kStatus_Fail;
    }

    return kStatus_Success;
}

static status_t move_revolutions(MTR_MotorHandle_t* handle, double revolutions)
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

    return move_angle(handle, angle);
}

static status_t set_velocity(MTR_MotorHandle_t* handle, double velocity_deg_per_sec)
{
    static char logMsg[100];
    if (emergencyStopFlag)
        return kStatus_Fail;
    if (handle == NULL)
        return kStatus_Fail;
    snprintf(logMsg, sizeof(logMsg), "[%s] Setting velocity to %.2f deg/sec", handle->label,
             velocity_deg_per_sec);
    LOG_DEBUG(logMsg);

    handle->stepperHandle.endVelocity =
        angle_to_steps(handle, fabs(velocity_deg_per_sec), ROUND_NEAREST);

    return kStatus_Success;
}

static status_t set_acceleration(MTR_MotorHandle_t* handle, double acceleration_deg_per_sec2)
{
    static char logMsg[100];
    if (emergencyStopFlag)
        return kStatus_Fail;
    if (handle == NULL)
        return kStatus_Fail;
    snprintf(logMsg, sizeof(logMsg), "[%s] Setting acceleration to %.2f deg/sec²", handle->label,
             acceleration_deg_per_sec2);
    LOG_DEBUG(logMsg);

    handle->stepperHandle.acceleration =
        angle_to_steps(handle, fabs(acceleration_deg_per_sec2), ROUND_NEAREST);

    return kStatus_Success;
}

static status_t stop_motor(MTR_MotorHandle_t* handle, bool decelerate)
{
    static char logMsg[100];
    if (emergencyStopFlag)
        return kStatus_Fail;
    if (handle == NULL)
        return kStatus_Fail;
    snprintf(logMsg, sizeof(logMsg), "[%s] Stopping motor (decelerate=%d)", handle->label,
             decelerate);
    LOG_DEBUG(logMsg);

    if (STP_stop(&handle->stepperHandle, decelerate) != kStatus_Success)
    {
        LOG_ERROR("Failed to stop stepper motor");
        return kStatus_Fail;
    }

    return kStatus_Success;
}

static status_t get_current_angle(MTR_MotorHandle_t* handle, double* angle)
{
    if (emergencyStopFlag)
        return kStatus_Fail;
    if (handle == NULL || angle == NULL)
        return kStatus_Fail;

    *angle = steps_to_angle(handle, handle->stepperHandle.absolutePosition);

    return kStatus_Success;
}

static status_t get_movement_state(MTR_MotorHandle_t* handle, STP_MovementState_t* state)
{
    if (emergencyStopFlag)
        return kStatus_Fail;
    if (handle == NULL || state == NULL)
        return kStatus_Fail;

    *state = handle->stepperHandle.movementHandle.state;

    return kStatus_Success;
}

static status_t wait_until_stopped(MTR_MotorHandle_t* handle, uint32_t timeout_ms)
{
    static char logMsg[100];
    if (emergencyStopFlag)
        return kStatus_Fail;
    if (handle == NULL)
        return kStatus_Fail;
    snprintf(logMsg, sizeof(logMsg), "[%s] Waiting for motor to stop (timeout=%lu ms)",
             handle->label, timeout_ms);
    LOG_DEBUG(logMsg);

    // TODO: Implement wait until stopped

    while (timeout_ms > 0)
    {
        STP_MovementState_t state;
        if (get_movement_state(handle, &state) != kStatus_Success)
        {
            LOG_ERROR("Failed to get movement state");
            return kStatus_Fail;
        }

        if (state == STP_MOVEMENT_IDLE || state == STP_MOVEMENT_STOPPED ||
            state == STP_MOVEMENT_FINISHED)
        {
            return kStatus_Success;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
        timeout_ms -= 100;
    }

    LOG_ERROR("Timeout waiting for motor to stop");
    return kStatus_Fail;
}

static status_t set_home_position(MTR_MotorHandle_t* handle)
{
    static char logMsg[100];
    if (emergencyStopFlag)
        return kStatus_Fail;
    if (handle == NULL)
        return kStatus_Fail;
    snprintf(logMsg, sizeof(logMsg), "[%s] Setting home position", handle->label);
    LOG_DEBUG(logMsg);

    handle->stepperHandle.absolutePosition = 0;

    return kStatus_Success;
}

static inline status_t calculate_sync_motion_parameters(MTR_MotorHandle_t** handles, double* angles,
                                                        uint8_t count, int32_t* stepCounts,
                                                        double* originalVelocities,
                                                        double* maxDuration)
{
    *maxDuration = 0.0;

    for (uint8_t i = 0; i < count; i++)
    {
        if (handles[i] == NULL)
            return kStatus_Fail;

        stepCounts[i]         = angle_to_steps(handles[i], angles[i], ROUND_NEAREST);
        originalVelocities[i] = handles[i]->stepperHandle.endVelocity;

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

static inline void scale_velocities_for_synchronization(MTR_MotorHandle_t** handles, uint8_t count,
                                                        int32_t* stepCounts,
                                                        double*  originalVelocities,
                                                        double   maxDuration)
{
    for (uint8_t i = 0; i < count; i++)
    {
        if (stepCounts[i] != 0)
        {
            uint32_t absSteps    = (stepCounts[i] >= 0) ? stepCounts[i] : -stepCounts[i];
            double   newVelocity = (double)absSteps / maxDuration;

            if (newVelocity > originalVelocities[i])
                newVelocity = originalVelocities[i];

            handles[i]->stepperHandle.endVelocity = newVelocity;
        }
    }
}

static inline status_t prepare_synchronized_movements(MTR_MotorHandle_t** handles, double* angles,
                                                      uint8_t count, int32_t* stepCounts,
                                                      double* originalVelocities)
{
    for (uint8_t i = 0; i < count; i++)
    {
        if (stepCounts[i] != 0)
        {
            status_t result = STP_move_relative_prepare(&handles[i]->stepperHandle, stepCounts[i]);

            if (result != kStatus_Success)
            {
                restore_original_velocities(handles, count, originalVelocities);
                return kStatus_Fail;
            }
        }
    }

    return kStatus_Success;
}

static inline void restore_original_velocities(MTR_MotorHandle_t** handles, uint8_t count,
                                               double* originalVelocities)
{
    for (uint8_t i = 0; i < count; i++)
    {
        handles[i]->stepperHandle.endVelocity = originalVelocities[i];
    }
}

static status_t synchronized_move(MTR_MotorHandle_t** handles, double* angles, uint8_t count)
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
                                         &maxDuration) != kStatus_Success)
        return kStatus_Fail;

    scale_velocities_for_synchronization(handles, count, stepCounts, originalVelocities,
                                         maxDuration);

    snprintf(logMsg, sizeof(logMsg), "Synchronized move: %d motors, duration=%.3fs", count,
             maxDuration);
    LOG_DEBUG(logMsg);

    if (prepare_synchronized_movements(handles, angles, count, stepCounts, originalVelocities) !=
        kStatus_Success)
        return kStatus_Fail;

    status_t triggerResult = STP_trigger_prepared_moves();
    if (triggerResult != kStatus_Success)
    {
        restore_original_velocities(handles, count, originalVelocities);
        return kStatus_Fail;
    }

    restore_original_velocities(handles, count, originalVelocities);

    return kStatus_Success;
}
