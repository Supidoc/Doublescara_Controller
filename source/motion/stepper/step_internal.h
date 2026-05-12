/************************************************************
 * @file    step_internal.h
 * @brief   Internal shared types, queues, and state for stepper submodules.
 * @author  dg
 * @date    13 Apr 2026
 ************************************************************/

/**
 * @defgroup STEPPER_Internal Stepper Internal
 * @brief Internal command payloads and shared subsystem state.
 * @ingroup STEPPER_Module
 * @{
 */

#ifndef STEP_INTERNAL_H_
#define STEP_INTERNAL_H_

#ifdef __cplusplus
extern "C"
{
#endif

/********************
 *     Includes    *
 ********************/
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "cmd_handle.h"
#include "step_shared.h"

/***********************************
 *     Public Macros / Defines     *
 ***********************************/

/**
 * @brief Timer frequency used for step generation (in Hz).
 **/
/** @brief Timer frequency used for step generation in Hz. */
#define STP_TIMER_FREQ_HZ 250000

/**
 * @brief Fixed-point scale factor for delay calculations
 * Using 16 fractional bits (scale by 65536) to maintain precision
 * This provides sufficient accuracy for high-speed applications
 * Based on principles from "Generate stepper-motor speed profiles in real time" by David Austin
 */
/** @brief Number of fractional bits used for delay scaling. */
#define STP_DELAY_SCALE_BITS 16
/** @brief Fixed-point scale factor for delay calculations. */
#define STP_DELAY_SCALE (1 << STP_DELAY_SCALE_BITS)

    /***************************
     *     Public Typedefs     *
     ***************************/

    typedef enum _STP_CmdType
    {
        STP_CMD_MOVE,
        STP_CMD_STOP,
        STP_CMD_MOVE_PREPARE,
        STP_CMD_TRIGGER_START,
        STP_CMD_INIT_HANDLE,
        STP_CMD_SET_ACCELERATION,
        STP_CMD_SET_END_VELOCITY
    } STP_CmdType_t;

    typedef struct _STP_MoveCmdData
    {
        int32_t steps; /**< Number of steps to execute (positive=CCW, negative=CW) */
    } STP_MoveCmdData_t;

    typedef struct _STP_StopCmdData
    {
        uint8_t doDeceleration;
    } STP_StopCmdData_t;

    typedef struct _STP_InitHandleCmdData
    {
        STP_StepperConfig_t config;
    } STP_InitHandleCmdData_t;

    typedef struct _STP_SetAccelerationCmdData
    {
        double acceleration;
    } STP_SetAccelerationCmdData_t;

    typedef struct _STP_SetEndVelocityCmdData
    {
        double endVelocity;
    } STP_SetEndVelocityCmdData_t;

    typedef struct _STP_CmdQueueItem
    {
        STP_CmdType_t type;
        STP_Handle_t  handle;   /**< Pointer to target stepper handle */
        TickType_t    deadline; /**< Deadline for command completion (0 for no deadline) */
        CHD_CmdHandle_t
            cmdHandle; /**< Command handle for task synchronization (NULL for no synchronization) */
        union
        {
            STP_MoveCmdData_t            move;
            STP_StopCmdData_t            stop;
            STP_InitHandleCmdData_t      config;
            STP_SetAccelerationCmdData_t setAcceleration;
            STP_SetEndVelocityCmdData_t  setEndVelocity;
        } data;
    } STP_CmdQueueItem_t;

    typedef struct _STP_AccelTablePoolItem
    {
        uint16_t table[STP_MAX_ACCEL_TABLE_SIZE]; /**< Pre-calculated delay values */
        uint8_t  used;                            /**< Flag: 1 if in use, 0 if free */
        uint32_t tableSize;                       /**< Number of valid entries in table */
    } STP_AccelTablePoolItem_t;

    /**
     * @brief Structure for tracking stepper motor movement state and parameters.
     *
     * Contains all necessary information for executing a stepper motor movement,
     * including acceleration/deceleration profile, current step count, and motor state.
     */
    typedef struct _STP_StepperMovement
    {
        STP_MovementState_t state;            /**< Current movement state */
        uint32_t            totalSteps;       /**< Total steps to execute */
        uint32_t            accelSteps;       /**< Steps during acceleration phase */
        uint32_t            endVelocitySteps; /**< Steps at constant velocity */
        uint32_t            decelSteps;       /**< Steps during deceleration phase */
        STP_Direction_t     direction;        /**< Movement direction */
        uint8_t             isTrapezoidal; /**< Flag: 1 if trapezoidal profile, 0 if triangular */
        uint32_t            currStepCount; /**< Current step count during movement */
        uint32_t phaseStepCount;           /**< Step count within current phase (for accel/decel) */
        uint16_t endVelocityDelay;         /**< Delay for constant velocity phase */
        int8_t   accelTablePoolIndex;      /**< Index into static pool (-1 if no table allocated) */
        uint32_t accelTableSize;           /**< Actual number of entries used in the table */
        uint16_t accelInterpFactor;        /**< Number of steps to repeat each table entry */
        uint16_t accelInterpCounter; /**< Counter for interpolation factor (0 to interpFactor-1) */
        uint32_t accelTableIndex;    /**< Current index into acceleration table */
        uint8_t waitForStart; /**< Flag: 1 if movement is planned but awaiting sync start trigger */
        CHD_CmdHandle_t
            cmdHandle; /**< Command handle for synchronizing movement start (NULL if no sync) */
    } STP_StepperMovement_t;

    /**
     * @brief Handle structure for a stepper motor instance.
     *
     * Contains hardware configuration and control parameters for a single stepper motor,
     * including FTM timer settings, GPIO pins, and movement parameters.
     */
    typedef struct _STP_StepperHandleImpl
    {
        FTM_Type*  ftmBase;    /**< Pointer to FTM module base address */
        ftm_chnl_t ftmChannel; /**< FTM channel used for step output */
        PORT_Type* stepPort;   /**< PORT module for step pin mux control */
        GPIO_Type* stepGPIO;   /**< GPIO module for step pin */
        uint8_t    stepPin;    /**< Step output pin number */
        PCA_Port_t dirPort;    /**< PORT for direction pin control */
        uint8_t    dirPin;     /**< Direction output pin number */
        uint8_t
            dirLogicHighClockwise; /**< Flag: 1 if high = clockwise, 0 if high = counterclockwise */
        int32_t absolutePosition;  /**< The absolute Position of the StepperMotor in steps. Positive
                                      Values are counterclockwise*/
        double                acceleration;   /**< Acceleration in steps/s² */
        double                endVelocity;    /**< Final velocity in steps/s */
        const char*           label;          /**< Identifier label for logging (e.g., "Motor_X") */
        STP_StepperMovement_t movementHandle; /**< Current movement state and parameters */
    } STP_StepperHandleImpl_t;

    typedef struct _STP_HandlesArrayItem
    {
        STP_StepperHandleImpl_t handle; /**< Stepper motor handle */
        uint8_t                 used;   /**< Flag: 1 if handle is allocated, 0 if free */
    } STP_HandlesArrayItem_t;

    /****************************
     *     Public Variables     *
     ****************************/

    extern STP_HandlesArrayItem_t stpHandles[STP_MAX_HANDLE_COUNT];

    extern CHD_CmdHandleImpl_t stpCmdHandles[STP_MAX_CMD_HANDLE_COUNT];

    extern QueueHandle_t stpCmdQueue;

    extern STP_Handle_t FTM3_ISR_handle_cache[8];

    extern STP_AccelTablePoolItem_t accelTablePool[STP_ACCEL_TABLE_POOL_SIZE];

    /**************************************
     *     Public Function Prototypes    *
     **************************************/

#ifdef __cplusplus
}
#endif

#endif // STEP_INTERNAL_H_

/** @} */