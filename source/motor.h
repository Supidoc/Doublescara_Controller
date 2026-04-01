/************************************************************
 * @file    motor.h
 * @brief   High-level motor control interface
 *
 * This module provides a high-level API for controlling stepper-driven motors
 * in mechanical units (angle and revolutions). It wraps the low-level step
 * generation and TMC2209 driver configuration modules and exposes task-safe
 * operations for movement, state monitoring, and safety handling.
 * @author  dg
 * @date    2 Mar 2026
 ************************************************************/

/**
 * @defgroup MOTOR_Module Motor Control Module
 * @brief   High-level API for angle-based motor control and safety handling
 *
 * The motor module combines stepper movement control and TMC2209 configuration
 * into one abstraction per motor. All kinematic commands are expressed in
 * mechanical units (degree, degree/s, degree/s^2, revolutions) and translated
 * internally to step-domain operations.
 *
 * @{
 */

#ifndef MOTOR_H_
#define MOTOR_H_

/********************
 *     Includes    *
 ********************/
#include "step.h"
#include "tmc2209.h"
#include "fsl_ftm.h"
#include "fsl_gpio.h"
#include "fsl_port.h"

/***********************************
 *     Public Macros / Defines     *
 ***********************************/

#define MTR_MAX_CMD_HANDLE_COUNT 20

/***************************
 *     Public Typedefs     *
 ***************************/

/**
 * @brief TMC2209-related configuration values for one motor.
 */
typedef struct _MTR_MotorTmcConfig
{
    uart_rtos_handle_t* uartRTOSHandle; /**< RTOS UART handle used for TMC communication. */
    uart_handle_t*      uartHandle;     /**< Low-level UART handle for TMC communication. */
    TMC_SerialAddress_t serialAdress;   /**< TMC2209 slave address on UART bus. */
    double              iHoldCurrentA;  /**< Hold current in ampere. */
    double              iRunCurrentA;   /**< Run current in ampere. */
} MTR_MotorTmcConfig_t;

/**
 * @brief Step generation hardware configuration for one motor.
 */
typedef struct _MTR_MotorStepperConfig
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
} MTR_MotorStepperConfig_t;

/**
 * @brief Configuration structure for initializing one motor instance.
 *
 * This structure groups mechanical conversion parameters and the underlying
 * low-level driver configurations required to build one motor handle.
 */
typedef struct _MTR_MotorConfig
{
    double  stepAngle;       /**< Full-step angle in degree (e.g., 1.8). */
    uint8_t microstep;       /**< Microstep factor (e.g., 1, 2, 4, ..., 256). */
    double  reductionFactor; /**< Mechanical transmission ratio from motor to output. */
    double  acceleration;    /**< Acceleration in degree/s² */
    double  endVelocity;     /**< Final velocity in degree/s */

    MTR_MotorTmcConfig_t     tmcConfig;     /**< TMC2209 communication and current settings. */
    MTR_MotorStepperConfig_t stepperConfig; /**< Step-generation hardware configuration. */

    char* label; /**< Human-readable motor identifier, must be unique. */
} MTR_MotorConfig_t;

typedef enum _MTR_roundingMethod
{
    ROUND_DOWN,
    ROUND_UP,
    ROUND_NEAREST
} MTR_roundingMethod_t;

/**
 * @brief Opaque handle type for motor control instances.
 */
typedef struct _MTR_MotorHandleImpl* MTR_MotorHandle_t;

/****************************
 *     Public Variables     *
 ****************************/

/**************************************
 *     Public Function Prototypes    *
 **************************************/

/**
 * @brief FreeRTOS task entry for the motor control module.
 *
 * This task processes queued motor commands and coordinates calls to the
 * stepper and TMC submodules.
 *
 * @param[in] pvParameters Task parameter pointer (unused).
 */
void MTR_task(void* pvParameters);

/**
 * @brief Initializes the motor module infrastructure.
 *
 * Must be called before creating motor handles or issuing motor commands.
 *
 * @return kStatus_Success if initialization succeeded.
 *         kStatus_Fail if initialization failed.
 */
status_t MTR_init(void);

/**
 * @brief Creates and initializes a motor handle from configuration.
 *
 * @param[in] config Complete motor configuration.
 * @param[in] deadline Maximum time to wait for command completion.
 * @param[out] cmdHandle Command handle for waiting/checking command completion.
 *
 * @return kStatus_Success if the handle was created successfully.
 *         kStatus_Fail on invalid parameters, timeout, or initialization failure.
 */
status_t MTR_init_handle_async(const MTR_MotorConfig_t config, TickType_t deadline,
                               THE_CmdHandle_t* cmdHandle);

/**
 * @brief Retrieves a motor handle by its label.
 *
 * @param[in] label Label string of the motor.
 * @param[out] handle Pointer receiving the matching motor handle.
 */
void MTR_get_motor_by_label(const char* label, MTR_MotorHandle_t* handle);

/**
 * @brief Commands a relative angular movement.
 *
 * @param[in] handle Target motor handle.
 * @param[in] angle Relative angle in degree (signed).
 * @param[in] deadline Maximum time to wait for command completion.
 * @param[out] cmdHandle Command handle for waiting/checking command completion.
 *
 * @return kStatus_Success if the command is accepted.
 *         kStatus_Fail on invalid parameters, queue errors, or timeout.
 */
status_t MTR_move_angle_async(MTR_MotorHandle_t handle, double angle, TickType_t deadline,
                              THE_CmdHandle_t* cmdHandle);

/**
 * @brief Commands movement to an absolute output angle.
 *
 * @param[in] handle Target motor handle.
 * @param[in] angle Absolute destination angle in degree.
 * @param[in] deadline Maximum time to wait for command completion.
 * @param[out] cmdHandle Command handle for waiting/checking command completion.
 *
 * @return kStatus_Success if the command is accepted.
 *         kStatus_Fail on invalid parameters, queue errors, or timeout.
 */
status_t MTR_move_absolute_angle_async(MTR_MotorHandle_t handle, double angle, TickType_t deadline,
                                       THE_CmdHandle_t* cmdHandle);

/**
 * @brief Commands a relative movement in revolutions.
 *
 * @param[in] handle Target motor handle.
 * @param[in] revolutions Relative number of revolutions (signed).
 * @param[in] deadline Maximum time to wait for command completion.
 * @param[out] cmdHandle Command handle for waiting/checking command completion.
 *
 * @return kStatus_Success if the command is accepted.
 *         kStatus_Fail on invalid parameters, queue errors, or timeout.
 */
status_t MTR_move_revolutions_async(MTR_MotorHandle_t handle, double revolutions,
                                    TickType_t deadline, THE_CmdHandle_t* cmdHandle);

/**
 * @brief Sets motor target velocity.
 *
 * @param[in] handle Target motor handle.
 * @param[in] velocity_deg_per_sec Target velocity in degree/s.
 * @param[in] deadline Maximum time to wait for command completion.
 * @param[out] cmdHandle Command handle for waiting/checking command completion.
 *
 * @return kStatus_Success if velocity was applied.
 *         kStatus_Fail on invalid parameters, queue errors, or timeout.
 */
status_t MTR_set_velocity_async(MTR_MotorHandle_t handle, double velocity_deg_per_sec,
                                TickType_t deadline, THE_CmdHandle_t* cmdHandle);

/**
 * @brief Sets motor acceleration.
 *
 * @param[in] handle Target motor handle.
 * @param[in] acceleration_deg_per_sec2 Target acceleration in degree/s^2.
 * @param[in] deadline Maximum time to wait for command completion.
 * @param[out] cmdHandle Command handle for waiting/checking command completion.
 *
 * @return kStatus_Success if acceleration was applied.
 *         kStatus_Fail on invalid parameters, queue errors, or timeout.
 */
status_t MTR_set_acceleration_async(MTR_MotorHandle_t handle, double acceleration_deg_per_sec2,
                                    TickType_t deadline, THE_CmdHandle_t* cmdHandle);

/**
 * @brief Stops ongoing motion.
 *
 * @param[in] handle Target motor handle.
 * @param[in] decelerate If true, perform controlled deceleration; otherwise stop immediately.
 * @param[in] deadline Maximum time to wait for command completion.
 * @param[out] cmdHandle Command handle for waiting/checking command completion.
 *
 * @return kStatus_Success if stop command was accepted.
 *         kStatus_Fail on invalid parameters, queue errors, or timeout.
 */
status_t MTR_stop_async(MTR_MotorHandle_t handle, bool decelerate, TickType_t deadline,
                        THE_CmdHandle_t* cmdHandle);

/**
 * @brief Triggers emergency stop for the motor subsystem.
 *
 * @param[in] handle Motor handle associated with the emergency stop command.
 *
 * @return kStatus_Success if emergency stop was triggered.
 *         kStatus_Fail if the command could not be processed.
 */
status_t MTR_emergency_stop(MTR_MotorHandle_t handle);

/**
 * @brief Clears the global emergency-stop latch.
 *
 * @return kStatus_Success if emergency stop was cleared.
 *         kStatus_Fail if clearing failed.
 */
status_t MTR_clear_emergency_stop(void);

/**
 * @brief Indicates whether emergency stop is currently active.
 *
 * @return Non-zero if emergency stop is active, otherwise 0.
 */
uint8_t MTR_is_emergency_stop_active(void);

/**
 * @brief Reads the current mechanical output angle.
 *
 * @param[in] handle Target motor handle.
 * @param[out] angle Pointer receiving the current angle in degree.
 * @param[in] deadline Maximum time to wait for query completion.
 * @param[out] cmdHandle Command handle for waiting/checking command completion.
 *
 * @return kStatus_Success if angle was read successfully.
 *         kStatus_Fail on invalid parameters, queue errors, or timeout.
 */
status_t MTR_get_current_angle_async(MTR_MotorHandle_t handle, double* angle, TickType_t deadline,
                                     THE_CmdHandle_t* cmdHandle);

/**
 * @brief Gets the current low-level movement state.
 *
 * @param[in] handle Target motor handle.
 * @param[out] state Pointer receiving current movement state.
 * @param[out] cmdHandle Command handle for waiting/checking command completion.
 *
 * @return kStatus_Success if state was read successfully.
 *         kStatus_Fail on invalid parameters, queue errors, or timeout.
 */
status_t MTR_get_movement_state_async(MTR_MotorHandle_t handle, STP_MovementState_t* state,
                                      THE_CmdHandle_t* cmdHandle);

/**
 * @brief Sets current position as home reference.
 *
 * @param[in] handle Target motor handle.
 * @param[out] cmdHandle Command handle for waiting/checking command completion.
 *
 * @return kStatus_Success if home position is set.
 *         kStatus_Fail on invalid parameters, queue errors, or timeout.
 */
status_t MTR_set_home_position_async(MTR_MotorHandle_t handle, THE_CmdHandle_t* cmdHandle);

/**
 * @brief Executes a synchronized multi-motor move.
 *
 * @param[in] handles Array of motor handles.
 * @param[in] angles Array of relative target angles in degree.
 * @param[in] count Number of elements in handles/angles.
 * @param[in] deadline Matypedef enum rounding_method
{
    ROUND_DOWN,
    ROUND_UP,
    ROUND_NEAREST
} MTR_roundingMethod_t;ximum time to wait for synchronization trigger.
 * @param[out] cmdHandle Command handle for waiting/checking command completion.
 *
 * @return kStatus_Success if all moves are prepared and triggered successfully.
 *         kStatus_Fail if any preparation/trigger step fails or deadline expires.
 */
status_t MTR_synchronized_move_async(MTR_MotorHandle_t* handles, double* angles, uint8_t count,
                                     TickType_t deadline, THE_CmdHandle_t* cmdHandle);

/**
 * @brief Sets TMC2209 run current for a motor.
 *
 * @param[in] handle Target motor handle.
 * @param[in] current_a Run current in ampere.
 * @param[in] deadline Maximum time to wait for command completion.
 * @param[out] cmdHandle Command handle for waiting/checking command completion.
 *
 * @return kStatus_Success if current was updated successfully.
 *         kStatus_Fail on invalid parameters, queue errors, or timeout.
 */
status_t MTR_set_run_current_async(MTR_MotorHandle_t handle, double current_a, TickType_t deadline,
                                   THE_CmdHandle_t* cmdHandle);

/**
 * @brief Sets TMC2209 hold current for a motor.
 *
 * @param[in] handle Target motor handle.
 * @param[in] current_a Hold current in ampere.
 * @param[in] deadline Maximum time to wait for command completion.
 * @param[out] cmdHandle Command handle for waiting/checking command completion.
 *
 * @return kStatus_Success if current was updated successfully.
 *         kStatus_Fail on invalid parameters, queue errors, or timeout.
 */
status_t MTR_set_hold_current_async(MTR_MotorHandle_t handle, double current_a, TickType_t deadline,
                                    THE_CmdHandle_t* cmdHandle);

double MTR_steps_to_angle(MTR_MotorHandle_t handle, int32_t steps);

int32_t MTR_angle_to_steps(MTR_MotorHandle_t handle, double angle, MTR_roundingMethod_t method);

/** @} */ /* End of MOTOR_Module */

#endif /* MOTOR_H_ */
