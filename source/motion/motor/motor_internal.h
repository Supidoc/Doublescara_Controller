/************************************************************
 * @file    motor_internal.h
 * @brief   Internal shared types and state for motor submodules.
 * @author  dg
 * @date    15 Apr 2026
 ************************************************************/

/**
 * @defgroup MOTOR_Internal Motor Internal
 * @brief Internal command types, queue payloads, and shared subsystem state.
 * @ingroup MOTOR_Facade_Module
 * @{
 */

#ifndef MOTOR_INTERNAL_H_
#define MOTOR_INTERNAL_H_

/********************
 *     Includes    *
 ********************/
#include "motor_core.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "cmd_handle.h"

/***********************************
 *     Public Macros / Defines     *
 ***********************************/

#define MTR_MAX_MOTORS 5
#define MTR_CMD_QUEUE_SIZE 20
#define MTR_MAX_NEEDED_CMD_HANDLES 5
#define MTR_MAX_PARALLEL_TASKS 20

/***************************
 *     Public Typedefs     *
 ***************************/

typedef struct _MTR_MotorHandleImpl
{
    STP_Handle_t stepperHandle;
    TMC_Handle_t tmcHandle;
    double       stepAngle;
    uint8_t      microstep;
    double       reductionFactor;
    char*        label;
    uint8_t      freewheeling;        /**< 1 if motor is in freewheeling mode, 0 otherwise */
    double       previousHoldCurrent; /**< Hold current before freewheeling was enabled */
} MTR_MotorHandleImpl_t;

typedef struct _MTR_HandlesArrayItem
{
    MTR_MotorHandleImpl_t handle;
    uint8_t               used;
} MTR_HandlesArrayItem_t;

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
    MTR_CMD_SET_HOME_POSITION,
    MTR_CMD_SYNCHRONIZED_MOVE,
    MTR_CMD_SET_RUN_CURRENT,
    MTR_CMD_SET_HOLD_CURRENT,
    MTR_CMD_INIT_MOTOR,
    MTR_CMD_ENABLE_FREEWHEELING,
    MTR_CMD_DISABLE_FREEWHEELING
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

typedef struct _MTR_EnableFreewheellingCmdData
{
} MTR_EnableFreewheellingCmdData_t;

typedef struct _MTR_DisableFreewheellingCmdData
{
} MTR_DisableFreewheellingCmdData_t;

typedef struct _MTR_CmdQueueItem
{
    MTR_CmdType_t     type;
    MTR_MotorHandle_t handle;
    CHD_CmdHandle_t   cmdHandle;
    TickType_t        deadline;
    union
    {
        MTR_MoveAngleCmdData_t            moveAngle;
        MTR_MoveRevolutionsCmdData_t      moveRevolutions;
        MTR_SetVelocityCmdData_t          setVelocity;
        MTR_SetAccelerationCmdData_t      setAcceleration;
        MTR_StopCmdData_t                 stop;
        MTR_GetCurrentAngleCmdData_t      getCurrentAngle;
        MTR_GetMovementStateCmdData_t     getMovementState;
        MTR_SynchronizedMoveCmdData_t     synchronizedMove;
        MTR_SetRunCurrentCmdData_t        setRunCurrent;
        MTR_SetHoldCurrentCmdData_t       setHoldCurrent;
        MTR_InitMotorCmdData_t            initMotor;
        MTR_EnableFreewheellingCmdData_t  enableFreewheeling;
        MTR_DisableFreewheellingCmdData_t disableFreewheeling;
    } data;
} MTR_CmdQueueItem_t;

typedef struct _MTR_ParallelTaskItem
{
    CHD_CmdHandle_t cmdHandles[MTR_MAX_NEEDED_CMD_HANDLES];
    CHD_CmdHandle_t returnHandle;
    uint8_t         count;
    uint8_t         used;
} MTR_ParallelTaskItem;

/****************************
 *     Public Variables     *
 ****************************/

extern QueueHandle_t          mtrCmdQueue;
extern CHD_CmdHandleImpl_t    mtrCmdHandles[MTR_MAX_CMD_HANDLE_COUNT];
extern MTR_HandlesArrayItem_t motorHandles[MTR_MAX_MOTORS];
extern MTR_ParallelTaskItem   parallelTasks[MTR_MAX_PARALLEL_TASKS];

extern volatile uint8_t emergencyStopFlag;

/**************************************
 *     Public Function Prototypes    *
 **************************************/

/**
 * @brief Internal command dispatcher entry point.
 *
 * @param[in] queueItem Dequeued motor command item.
 * @param[in,out] taskItem Parallel-task tracking record for async sub-commands.
 * @return kStatus_Success, kStatus_Fail, or kStatus_Timeout depending on execution result.
 */
status_t MTRi_process_cmd(MTR_CmdQueueItem_t queueItem, MTR_ParallelTaskItem* taskItem);

#endif // MOTOR_INTERNAL_H_

/** @} */