/************************************************************
 * @file    step.h
 * @brief   Module for controlling stepper motors with FTM
 *
 * This module provides functionality for controlling stepper motors using
 * the FTM peripheral. It supports acceleration
 * and deceleration profiles with trapezoidal velocity profiles for smooth
 * motor control. The module uses FreeRTOS queues for task-safe movement
 * commands.
 *
 * @note    Ensure the FTM peripherals and GPIO are initialized before using this module.
 * @author  dg
 * @date    4 Jan 2026
 ************************************************************/

/**
 * @defgroup STEP_Module Stepper Motor Control Module
 * @brief   Functions for controlling stepper motors using FTM with acceleration/deceleration
 * @{
 */

#ifndef STEP_H_
#define STEP_H_

/********************
 *     Includes		*
 ********************/

#include "fsl_ftm.h"
#include "fsl_gpio.h"
#include "fsl_port.h"

/***********************************
 *     Public Macros / Defines	   *
 ***********************************/

/**
 * @brief Maximum number of stepper motor handles that can be created.
 */
#define STP_MAX_HANDLE_COUNT 6

/**
 * @brief Minimum acceleration for stepper motors (in degrees/s²).
 */
#define STP_MIN_STEP_ACCELERATION 10

/**
 * @brief Maximum step frequency for stepper motors (in steps/s).
 */
#define STP_MAX_STEP_FREQUENCY 100

/**
 * @brief Size of the movement command queue.
 */
#define STP_MOVEMENT_QUEUE_SIZE 10

/**
 * @brief Size of the stepper configuration queue.
 */
#define STP_CONFIG_QUEUE_SIZE 3

/**
 * @brief Maximum number of steps in the acceleration/deceleration lookup table.
 * This limits the size of pre-calculated delay profiles to conserve memory.
 * Movements with more acceleration steps will be clamped to this value.
 */
#define STP_MAX_ACCEL_TABLE_SIZE 3000

/**
 * @brief Interpolation factor between lookup table entries.
 * Each table entry spans this many acceleration steps.
 * A value of 1 means no interpolation.
 */
#define STP_ACCEL_TABLE_INTERP_FACTOR 1

/**
 * @brief Number of acceleration lookup tables in the static pool.
 * Each stepper movement requires one table from the pool.
 * If all tables are in use, new movement commands will fail until a table is freed.
 */
#define STP_ACCEL_TABLE_POOL_SIZE 8

/***************************
 *     Public Typedefs	   *
 ***************************/

/**
 * @brief Enumeration of stepper motor movement states.
 *
 * Represents the current state of a stepper motor movement operation.
 */
typedef enum _STP_MovementState
{
    STP_MOVEMENT_IDLE,           /**< Stepper motor is idling */
    STP_MOVEMENT_CALLED,         /**< Movement command received */
    STP_MOVEMENT_PLANNED,        /**< Movement profile calculated */
    STP_MOVEMENT_STARTED,        /**< Movement started */
    STP_MOVEMENT_ACCELERATING,   /**< Motor is currently accelerating */
    STP_MOVEMENT_CONST_VELOCITY, /**< Motor is currently running with constant velocity */
    STP_MOVEMENT_DECELRATING,    /**< Motor is currently decelerating */
    STP_MOVEMENT_STOPPED,        /**< Movement stopped before completion */
    STP_MOVEMENT_FINISHED,       /**< Movement completed */
    STP_MOVEMENT_SUCCESSFUL,     /**< Movement completed successfully */
    STP_MOVEMENT_FAILED          /**< Movement failed */
} STP_MovementState_t;

/**
 * @brief Enumeration of stepper motor rotation directions.
 */
typedef enum _STP_Direction
{
    STP_CLOCKWISE,       /**< Rotate clockwise */
    STP_COUNTERCLOCKWISE /**< Rotate counterclockwise */
} STP_Direction_t;

/**
 * @brief Structure for tracking stepper motor movement state and parameters.
 *
 * Contains all necessary information for executing a stepper motor movement,
 * including acceleration/deceleration profile, current step count, and motor state.
 */
typedef struct _STP_StepperMovementHandle
{
    STP_MovementState_t state;            /**< Current movement state */
    uint32_t            totalSteps;       /**< Total steps to execute */
    uint32_t            accelSteps;       /**< Steps during acceleration phase */
    uint32_t            endVelocitySteps; /**< Steps at constant velocity */
    uint32_t            decelSteps;       /**< Steps during deceleration phase */
    STP_Direction_t     direction;        /**< Movement direction */
    uint8_t             isTrapezoidal;    /**< Flag: 1 if trapezoidal profile, 0 if triangular */
    uint32_t            currStepCount;    /**< Current step count during movement */
    uint32_t            phaseStepCount;   /**< Step count within current phase (for accel/decel) */
    uint16_t            endVelocityDelay; /**< Delay for constant velocity phase */
    int8_t   accelTablePoolIndex;         /**< Index into static pool (-1 if no table allocated) */
    uint32_t accelTableSize;              /**< Actual number of entries used in the table */
    uint16_t accelInterpFactor;           /**< Number of steps to repeat each table entry */
    uint16_t accelInterpCounter; /**< Counter for interpolation factor (0 to interpFactor-1) */
    uint32_t accelTableIndex;    /**< Current index into acceleration table */
    uint8_t  waitForStart; /**< Flag: 1 if movement is planned but awaiting sync start trigger */
} STP_StepperMovementHandle_t;

/**
 * @brief Handle structure for a stepper motor instance.
 *
 * Contains hardware configuration and control parameters for a single stepper motor,
 * including FTM timer settings, GPIO pins, and movement parameters.
 */
typedef struct _STP_StepperHandle
{
    FTM_Type*  ftmBase;            /**< Pointer to FTM module base address */
    ftm_chnl_t ftmChannel;         /**< FTM channel used for step output */
    PORT_Type* stepPort;           /**< PORT module for step pin mux control */
    GPIO_Type* stepGPIO;           /**< GPIO module for step pin */
    uint8_t    stepPin;            /**< Step output pin number */
    port_mux_t stepMuxFTM;         /**< Pin mux value for FTM mode */
    port_mux_t stepMuxGPIO;        /**< Pin mux value for GPIO mode */
    PORT_Type* dirPort;            /**< PORT module for direction pin mux control */
    GPIO_Type* dirGPIO;            /**< GPIO module for direction pin */
    uint8_t    dirPin;             /**< Direction output pin number */
    port_mux_t dirMux;             /**< Pin mux value for dir pin */
    uint8_t dirLogicHighClockwise; /**< Flag: 1 if high = clockwise, 0 if high = counterclockwise */
    int32_t absolutePosition;      /**< The absolute Position of the StepperMotor in steps. Positive
                                      Values are counterclockwise*/
    double                      acceleration; /**< Acceleration in steps/s² */
    double                      endVelocity;  /**< Final velocity in steps/s */
    const char*                 label;        /**< Identifier label for logging (e.g., "Motor_X") */
    STP_StepperMovementHandle_t movementHandle; /**< Current movement state and parameters */
} STP_StepperHandle_t;

/**
 * @brief Configuration structure for initializing a stepper motor.
 *
 * Contains all necessary configuration parameters to initialize and set up
 * a new stepper motor control instance.
 */
typedef struct _STP_StepperConfig
{
    FTM_Type*  ftmBase;            /**< Pointer to FTM module base address */
    ftm_chnl_t ftmChannel;         /**< FTM channel for step output */
    PORT_Type* stepPort;           /**< PORT module for step pin mux control */
    GPIO_Type* stepGPIO;           /**< GPIO module for step pin */
    uint8_t    stepPin;            /**< Step output pin number */
    port_mux_t stepMuxFTM;         /**< Pin mux value for FTM mode */
    port_mux_t stepMuxGPIO;        /**< Pin mux value for GPIO mode */
    PORT_Type* dirPort;            /**< PORT module for direction pin mux control */
    GPIO_Type* dirGPIO;            /**< GPIO module for direction pin */
    uint8_t    dirPin;             /**< Direction output pin number */
    port_mux_t dirMux;             /**< Pin mux value for dir pin */
    uint8_t dirLogicHighClockwise; /**< Flag: 1 if high = clockwise, 0 if high = counterclockwise */
    double  acceleration;          /**< Acceleration in steps/s² */
    double  endVelocity;           /**< Final velocity in steps/s */
    const char* label;             /**< Identifier label for logging (e.g., "Motor_X") */
} STP_StepperConfig_t;

/****************************
 *     Public Variables	    *
 ****************************/

/**************************************
 *     Public Function Prototypes	  *
 **************************************/

/**
 * @brief Initializes the stepper motor control module.
 *
 * This function initializes the movement and configuration queues used for
 * task-safe communication. It must be called before using any other functions
 * in this module.
 *
 * @note This function is NOT task-safe and must be called during system initialization.
 * @note The STP_task() should be started after this function completes successfully.
 * @note Ensure FTM timer and GPIO peripherals are initialized before calling this.
 *
 * @return kStatus_Success if initialization is successful.
 *         kStatus_Fail if queue creation fails.
 *
 * @see STP_task()
 */
status_t STP_init(void);

/**
 * @brief Task function for processing stepper motor commands.
 *
 * This function is intended to be run as a FreeRTOS task. It continuously
 * processes configuration and movement commands from the queues and manages
 * the stepper motor state machine.
 *
 * @param[in] pvParameters Pointer to task parameters (not used in this implementation).
 *
 * @note This task should be started after STP_init() completes successfully.
 * @note This task is blocked on command queues and wakes when commands are available.
 * @warning This function does not return; it is expected to run as a perpetual FreeRTOS task.
 *
 * @see STP_init()
 */
void STP_task(void* pvParameters);

/**
 * @brief Sends a relative movement command to a stepper motor.
 *
 * This function adds a movement command to the queue for the specified stepper.
 * The movement will be executed asynchronously by the STP_task function.
 * The direction is encoded in the sign of the steps parameter:
 * positive values move counter-clockwise (CCW), negative values move clockwise (CW).
 *
 * @param[in] handle Pointer to the stepper motor handle.
 * @param[in] steps Number of steps to move (positive=CCW, negative=CW).
 *
 * @note This function is TASK-SAFE and can be called from any task context.
 * @note The actual movement occurs in the STP_task() and is non-blocking.
 * @warning Multiple calls will queue movements to execute sequentially.
 *
 * @return kStatus_Success if the command is successfully queued.
 *         kStatus_Fail if the queue is full or an error occurs.
 *
 * @see STP_Direction_t
 */
status_t STP_move_relative(STP_StepperHandle_t* handle, int32_t steps);

/**
 * @brief Initializes a stepper motor with the provided configuration.
 *
 * This function sends a configuration command to the stepper control task.
 * The stepper motor will be configured according to the provided parameters.
 *
 * @param[in] config Configuration structure containing stepper parameters.
 *
 * @note This function is TASK-SAFE and can be called after STP_init().
 * @note Configuration is applied asynchronously by the STP_task().
 * @warning Do not modify the config structure after calling this function.
 *
 * @return kStatus_Success if the configuration is successfully queued.
 *         kStatus_Fail if the queue is full or an error occurs.
 *
 * @see STP_StepperConfig_t
 * @see STP_task()
 */
status_t STP_init_stepper(STP_StepperConfig_t config);

/**
 * @brief Retrieves the stepper motor handle by its label identifier.
 *
 * This function searches for a stepper motor by its label and returns a pointer
 * to its handle structure if found.
 *
 * @param[in] label The label identifier of the stepper motor to find.
 * @param[out] handle Pointer to store the found stepper motor handle.
 *
 * @note This function is TASK-SAFE and can be called from any task context.
 * @note The label must match exactly as configured during STP_init_stepper().
 *
 * @return kStatus_Success if the stepper motor is found.
 *         kStatus_Fail if the stepper motor with the given label is not found.
 *
 * @see STP_init_stepper()
 * @see STP_StepperHandle_t
 */
status_t STP_get_handle_by_label(const char* label, STP_StepperHandle_t** handle);

/**
 * @brief Stops a stepper motor movement.
 *
 * This function sends a command to stop the specified stepper motor. The motor
 * can be stopped immediately or with a deceleration phase to smoothly bring
 * it to rest, depending on the doDeceleration parameter.
 *
 * @param[in] handle Pointer to the stepper motor handle.
 * @param[in] doDeceleration If 1, the motor will decelerate to a stop. If 0, the motor
 *                           will stop immediately.
 *
 * @note This function is TASK-SAFE and can be called from any task context.
 * @note The stop command is queued and executed by the STP_task().
 *
 * @return kStatus_Success if the stop command is successfully queued.
 *         kStatus_Fail if the queue is full or an error occurs.
 *
 * @see STP_move_relative()
 */
status_t STP_stop(STP_StepperHandle_t* handle, uint8_t doDeceleration);

status_t STP_reset_absolute_position(STP_StepperHandle_t* handle);

status_t STP_move_relative_prepare(STP_StepperHandle_t* handle, int32_t steps);

status_t STP_trigger_prepared_moves(void);

/** @} */ // End of STEP_Module

#endif /* STEP_H_ */
