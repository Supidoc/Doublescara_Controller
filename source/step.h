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
 * @defgroup STEP_Module Stepper Generator Module
 * @brief   Functions for controlling stepper motors using FTM with constant
 * acceleration/deceleration
 *
 * This module provides an interface for initializing stepper generatordox instances, configuring
 * movement parameters, and executing stepper movements with acceleration and deceleration profiles.
 * It uses FreeRTOS queues to allow for task-safe command submission and asynchronous execution in a
 * dedicated stepper control task.
 *
 * @{
 */

#ifndef STEP_H_
#define STEP_H_

/********************
 *     Includes		*
 ********************/
#include "FreeRTOS.h"
#include "task.h"
#include "fsl_common.h"
#include "fsl_ftm.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "task_helpers.h"
#include "pca9555a.h"

/***********************************
 *     Public Macros / Defines	   *
 ***********************************/

/**
 * @brief Maximum number of stepper motor handles that can be created.
 */
#define STP_MAX_HANDLE_COUNT 6

/**
 * @brief Minimum step acceleration for stepper motors (in steps/s²).
 *
 */
#define STP_MIN_STEP_ACCELERATION 10.0

/**
 * @brief Maximum step frequency for stepper motors (in steps/s).
 */
#define STP_MAX_STEP_FREQUENCY 100

/**
 * @brief Size of the command queue.
 */
#define STP_CMD_QUEUE_SIZE 20

#define STP_MAX_CMD_HANDLE_COUNT 30

/**
 * @brief Maximum number of steps in the acceleration/deceleration lookup table.
 * This limits the size of pre-calculated delay profiles to conserve memory.
 * Movements with more acceleration steps will be clamped to this value.
 */
#define STP_MAX_ACCEL_TABLE_SIZE 1000

/**
 * @brief Interpolation factor between lookup table entries.
 * Each table entry spans this many acceleration steps.
 * A value of 1 means no interpolation.
 */
#define STP_ACCEL_TABLE_INTERP_FACTOR 4

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
    /** @brief Motor is idling */
    STP_MOVEMENT_IDLE,
    /** @brief Movement command received, but profile not yet calculated */
    STP_MOVEMENT_CALLED,
    /** @brief Movement profile calculated and ready to start */
    STP_MOVEMENT_PLANNED,
    /** @brief Movement is waiting for synchronized start trigger */
    STP_MOVEMENT_STARTED,
    /** @brief Motor is currently accelerating */
    STP_MOVEMENT_ACCELERATING,
    /** @brief Motor is currently running with constant velocity */
    STP_MOVEMENT_CONST_VELOCITY,
    /** @brief Motor is currently decelerating */
    STP_MOVEMENT_DECELRATING,
    /** @brief Movement stopped before completion */
    STP_MOVEMENT_STOPPED,
    /** @brief Movement completed */
    STP_MOVEMENT_FINISHED,
    /** @brief Movement completed successfully */
    STP_MOVEMENT_SUCCESSFUL,
    /** @brief Movement failed due to an error */
    STP_MOVEMENT_FAILED
} STP_MovementState_t;

/**
 * @brief Enumeration of stepper motor rotation directions.
 */
typedef enum _STP_Direction
{
    /** @brief Rotate clockwise */
    STP_CLOCKWISE,
    /** @brief Rotate counterclockwise */
    STP_COUNTERCLOCKWISE
} STP_Direction_t;

/**
 * @brief Opaque handle type for stepper motor control instances.
 *
 */
typedef struct _STP_StepperHandleImpl* STP_Handle_t;

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
    PCA_Port_t dirPort;            /**< PORT for direction pin control */
    uint8_t    dirPin;             /**< Direction output pin number */
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
 * task-safe communication. It also initialzes the timer and gpio peripheral for step generation.
 * It must be called before using any other functions in this module.
 *
 * @note This function is NOT task-safe and must be called during system initialization.
 * @note The STP_task() should be started after this function completes successfully.
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
 * @brief Retrieves the default configuration for a stepper motor handle.
 *
 * This function populates a STP_StepperConfig_t structure with default values for initializing a
 * stepper motor control instance.
 *
 * @param[out] config Pointer to a STP_StepperConfig_t structure to be filled with default
 * configuration values. Must not be NULL.
 *
 * @return kStatus_Success if the default configuration is successfully retrieved.
 *         kStatus_Fail if the config pointer is NULL.
 *
 */
status_t STP_get_default_config(STP_StepperConfig_t* config);

/**
 * @brief Initializes a stepper motor with the provided configuration.
 *
 * This function sends a configuration command to the stepper control task.
 * The stepper motor will be configured according to the provided parameters.
 *
 * @param[in] config Configuration structure containing stepper parameters.
 * @param[in] deadline Deadline for the initialization process.
 * @param[out] Command Handle to await for completion
 *
 * @note This function is TASK-SAFE and can be called after STP_init().
 * @note Configuration is applied asynchronously by the STP_task().
 *
 * @return kStatus_Success if the configuration is successfully queued.
 *         kStatus_Fail if the queue is full or an error occurs.
 *
 * @see STP_StepperConfig_t
 * @see STP_task()
 */
status_t STP_init_handle_async(STP_StepperConfig_t config, TickType_t deadline,
                               THE_CmdHandle_t* cmdHandle);

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
status_t STP_get_handle_by_label(const char* label, STP_Handle_t* handle);

/**
 * @brief Sends a relative movement command to a stepper motor.
 *
 * This function adds a movement command to the queue for the specified stepper.
 * The movement will be executed asynchronously by the STP_task function.
 * The direction is encoded in the sign of the steps parameter:
 * positive values move counter-clockwise (CCW), negative values move clockwise (CW).
 *
 * @param[in] handle Stepper motor handle to move
 * @param[in] steps Number of steps to move (positive=CCW, negative=CW).
 * @param[out] Command Handle to await for completion
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
status_t STP_move_relative_async(STP_Handle_t handle, int32_t steps, TickType_t deadline,
                                 THE_CmdHandle_t* cmdHandle);

/**
 * @brief Stops a stepper motor movement.
 *
 * This function sends a command to stop the specified stepper motor. The motor
 * can be stopped immediately or with a deceleration phase to smoothly bring
 * it to rest, depending on the doDeceleration parameter.
 *
 * @param[in] handle Stepper motor handle to stop
 * @param[in] doDeceleration If 1, the motor will decelerate to a stop. If 0, the motor
 *                           will stop immediately.
 * @param[out] Command Handle to await for completion
 *
 * @note This function is TASK-SAFE and can be called from any task context.
 * @note The stop command is queued and executed by the STP_task().
 *
 * @return kStatus_Success if the stop command is successfully queued.
 *         kStatus_Fail if the queue is full or an error occurs.
 *
 * @see STP_move_relative()
 */
status_t STP_stop_async(STP_Handle_t handle, uint8_t doDeceleration, TickType_t deadline,
                        THE_CmdHandle_t* cmdHandle);

/**
 * @brief Resets the tracked absolute position of a stepper motor to zero.
 *
 * This function queues a command that clears the software-maintained
 * absolute step counter for the selected motor handle.
 *
 * @param[in] handle Stepper motor handle whose absolute position is reset.
 *
 * @return kStatus_Success if the command is accepted and completed.
 *         kStatus_Fail if the command cannot be queued.
 */
status_t STP_reset_absolute_position(STP_Handle_t handle);

/**
 * @brief Prepares a relative movement command without starting motion immediately.
 *
 * This function stores a movement command in the prepared state so multiple
 * motors can be started in a synchronized way with STP_trigger_prepared_moves().
 *
 * @param[in] handle Stepper motor handle to prepare.
 * @param[in] steps Relative step count to prepare (positive=CCW, negative=CW).
 * @param[in] deadline Maximum time to wait for command completion.
 * * @param[out] Command Handle to await for completion
 *
 * @return kStatus_Success if the prepare command is accepted.
 *         kStatus_Fail if the command cannot be queued or completed before deadline.
 *
 * @see STP_trigger_prepared_moves()
 */
status_t STP_move_relative_prepare_async(STP_Handle_t handle, int32_t steps, TickType_t deadline,
                                         THE_CmdHandle_t* cmdHandle);

/**
 * @brief Starts all previously prepared movements.
 *
 * This function triggers queued prepared movement commands, allowing
 * synchronized motion start across multiple stepper handles.
 *
 * @param[in] deadline Maximum time to wait for trigger command completion.
 *
 * @return kStatus_Success if prepared movements are triggered successfully.
 *         kStatus_Fail if the trigger command fails or the deadline is exceeded.
 */
status_t STP_trigger_prepared_moves_async(TickType_t deadline, THE_CmdHandle_t* cmdHandle);

/**
 * @brief Reads the current absolute step position of a motor.
 *
 * @param[in] handle Stepper motor handle to query.
 * @param[out] absoluteSteps Pointer receiving the absolute position in steps.
 *
 * @return kStatus_Success if the value is returned successfully.
 *         kStatus_Fail if parameters are invalid, queueing fails..
 */
status_t STP_get_absolute_steps(STP_Handle_t handle, int32_t* absoluteSteps);

/**
 * @brief Gets the configured acceleration of a motor.
 *
 * @param[in] handle Stepper motor handle to query.
 * @param[out] acceleration Pointer receiving acceleration in steps/s^2.
 *
 * @return kStatus_Success if the acceleration is returned successfully.
 *         kStatus_Fail if parameters are invalid, queueing fails.
 */
status_t STP_get_acceleration(STP_Handle_t handle, double* acceleration);

/**
 * @brief Gets the configured target end velocity of a motor.
 *
 * @param[in] handle Stepper motor handle to query.
 * @param[out] endVelocity Pointer receiving end velocity in steps/s.
 *
 * @return kStatus_Success if the end velocity is returned successfully.
 *         kStatus_Fail if parameters are invalid, queueing fails.
 */
status_t STP_get_end_velocity(STP_Handle_t handle, double* endVelocity);

/**
 * @brief Retrieves the current movement state of a motor.
 *
 * @param[in] handle Stepper motor handle to query.
 * @param[out] state Pointer receiving the current state.
 *
 * @return kStatus_Success if the state is returned successfully.
 *         kStatus_Fail if parameters are invalid, queueing fails.
 */
status_t STP_get_movement_state(STP_Handle_t handle, STP_MovementState_t* state);

/**
 * @brief Sets the acceleration parameter for a motor.
 *
 * @param[in] handle Stepper motor handle to configure.
 * @param[in] acceleration New acceleration in steps/s^2.
 * @param[in] deadline Maximum time to wait for command completion.
 * @param[out] Command Handle to await for completion
 *
 * @return kStatus_Success if the acceleration is updated successfully.
 *         kStatus_Fail if parameters are invalid, queueing fails, or deadline expires.
 */
status_t STP_set_acceleration_async(STP_Handle_t handle, double acceleration, TickType_t deadline,
                                    THE_CmdHandle_t* cmdHandle);

/**
 * @brief Sets the target end velocity parameter for a motor.
 *
 * @param[in] handle Stepper motor handle to configure.
 * @param[in] endVelocity New end velocity in steps/s.
 * @param[in] deadline Maximum time to wait for command completion.
 * @param[out] Command Handle to await for completion
 *
 * @return kStatus_Success if the end velocity is updated successfully.
 *         kStatus_Fail if parameters are invalid, queueing fails, or deadline expires.
 */
status_t STP_set_end_velocity_async(STP_Handle_t handle, double endVelocity, TickType_t deadline,
                                    THE_CmdHandle_t* cmdHandle);

/** @} */ // End of STEP_Module

#endif /* STEP_H_ */
