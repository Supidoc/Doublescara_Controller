/************************************************************
 * @file    motor_core.h
 * @brief   Public motor API declarations.
 * @author  dg
 * @date    15 Apr 2026
 ************************************************************/

/**
 * @defgroup MOTOR_Facade_Module Motor Module
 * @brief Public task-safe motor API for queued motion and driver commands.
 *
 * This module exposes the externally visible motor control API that other
 * application modules should call.
 *
 * @{
 */

#ifndef MOTOR_CORE_H_
#define MOTOR_CORE_H_

/********************
 *     Includes    *
 ********************/
#include "motor_internal.h"

/**************************************
 *     Public Function Prototypes    *
 **************************************/

/**
 * @brief Task entry point for the motor subsystem.
 *
 * Runs the motor command processing loop and should be created as a FreeRTOS task
 * after successful module initialization.
 *
 * @param[in] pvParameters FreeRTOS task parameter pointer (unused).
 *
 * @note This function does not return.
 */
void MTR_task(void* pvParameters);

/**
 * @brief Initializes the motor facade module.
 *
 * Resets internal handle pools and creates required command queue resources.
 *
 * @return kStatus_Success if initialization completed.
 *         kStatus_Fail if required resources could not be created.
 */
status_t MTR_init(void);

/**
 * @brief Queues asynchronous initialization of one motor handle.
 *
 * @param[in] config Motor configuration used to create and initialize the handle.
 * @param[in] deadline Timeout/deadline for queueing and dependent operations.
 * @param[out] cmdHandle Optional command handle used to wait for completion.
 *
 * @return kStatus_Success if command was queued.
 *         kStatus_Fail if validation or queueing failed.
 */
status_t MTR_init_handle_async(const MTR_MotorConfig_t config, TickType_t deadline,
                               CHD_CmdHandle_t* cmdHandle);

/**
 * @brief Resolves a motor handle by its configured label.
 *
 * @param[in] label Null-terminated label string to match.
 * @param[out] handle Destination pointer receiving the matched handle or NULL.
 */
void MTR_get_motor_by_label(const char* label, MTR_MotorHandle_t* handle);

/**
 * @brief Queues relative movement by angle in degrees.
 *
 * @param[in] handle Target motor handle.
 * @param[in] angle Relative angle in degrees.
 * @param[in] deadline Timeout/deadline for command.
 * @param[out] cmdHandle Optional command handle for completion wait.
 * @return kStatus_Success if queued, otherwise kStatus_Fail.
 */
status_t MTR_move_angle_async(MTR_MotorHandle_t handle, double angle, TickType_t deadline,
                              CHD_CmdHandle_t* cmdHandle);

/**
 * @brief Queues move to an absolute mechanical angle.
 *
 * @param[in] handle Target motor handle.
 * @param[in] angle Absolute target angle in degrees.
 * @param[in] deadline Timeout/deadline for command.
 * @param[out] cmdHandle Optional command handle for completion wait.
 * @return kStatus_Success if queued, otherwise kStatus_Fail.
 */
status_t MTR_move_absolute_angle_async(MTR_MotorHandle_t handle, double angle, TickType_t deadline,
                                       CHD_CmdHandle_t* cmdHandle);

/**
 * @brief Queues relative movement in revolutions.
 *
 * @param[in] handle Target motor handle.
 * @param[in] revolutions Relative movement in revolutions.
 * @param[in] deadline Timeout/deadline for command.
 * @param[out] cmdHandle Optional command handle for completion wait.
 * @return kStatus_Success if queued, otherwise kStatus_Fail.
 */
status_t MTR_move_revolutions_async(MTR_MotorHandle_t handle, double revolutions,
                                    TickType_t deadline, CHD_CmdHandle_t* cmdHandle);

/**
 * @brief Queues update of target velocity.
 *
 * @param[in] handle Target motor handle.
 * @param[in] velocity_deg_per_sec Velocity in degrees per second.
 * @param[in] deadline Timeout/deadline for command.
 * @param[out] cmdHandle Optional command handle for completion wait.
 * @return kStatus_Success if queued, otherwise kStatus_Fail.
 */
status_t MTR_set_velocity_async(MTR_MotorHandle_t handle, double velocity_deg_per_sec,
                                TickType_t deadline, CHD_CmdHandle_t* cmdHandle);

/**
 * @brief Queues update of target acceleration.
 *
 * @param[in] handle Target motor handle.
 * @param[in] acceleration_deg_per_sec2 Acceleration in degrees per second squared.
 * @param[in] deadline Timeout/deadline for command.
 * @param[out] cmdHandle Optional command handle for completion wait.
 * @return kStatus_Success if queued, otherwise kStatus_Fail.
 */
status_t MTR_set_acceleration_async(MTR_MotorHandle_t handle, double acceleration_deg_per_sec2,
                                    TickType_t deadline, CHD_CmdHandle_t* cmdHandle);

/**
 * @brief Queues stop command for a motor.
 *
 * @param[in] handle Target motor handle.
 * @param[in] decelerate If true, stop with deceleration; otherwise immediate stop.
 * @param[in] deadline Timeout/deadline for command.
 * @param[out] cmdHandle Optional command handle for completion wait.
 * @return kStatus_Success if queued, otherwise kStatus_Fail.
 */
status_t MTR_stop_async(MTR_MotorHandle_t handle, bool decelerate, TickType_t deadline,
                        CHD_CmdHandle_t* cmdHandle);

/**
 * @brief Queues query for current motor angle.
 *
 * @param[in] handle Target motor handle.
 * @param[out] angle Pointer receiving current angle in degrees.
 * @param[in] deadline Timeout/deadline for command.
 * @param[out] cmdHandle Optional command handle for completion wait.
 * @return kStatus_Success if queued, otherwise kStatus_Fail.
 */
status_t MTR_get_current_angle_async(MTR_MotorHandle_t handle, double* angle, TickType_t deadline,
                                     CHD_CmdHandle_t* cmdHandle);

/**
 * @brief Queues query for current movement state.
 *
 * @param[in] handle Target motor handle.
 * @param[out] state Pointer receiving movement state.
 * @param[out] cmdHandle Optional command handle for completion wait.
 * @return kStatus_Success if queued, otherwise kStatus_Fail.
 */
status_t MTR_get_movement_state_async(MTR_MotorHandle_t handle, STP_MovementState_t* state,
                                      CHD_CmdHandle_t* cmdHandle);

/**
 * @brief Queues command to set current motor position as home.
 *
 * @param[in] handle Target motor handle.
 * @param[out] cmdHandle Optional command handle for completion wait.
 * @return kStatus_Success if queued, otherwise kStatus_Fail.
 */
status_t MTR_set_home_position_async(MTR_MotorHandle_t handle, CHD_CmdHandle_t* cmdHandle);

/**
 * @brief Queues synchronized movement command for multiple motors.
 *
 * @param[in] handles Array of motor handles.
 * @param[in] angles Array of relative angles in degrees.
 * @param[in] count Number of motors in arrays.
 * @param[in] deadline Timeout/deadline for command.
 * @param[out] cmdHandle Optional command handle for completion wait.
 * @return kStatus_Success if queued, otherwise kStatus_Fail.
 */
status_t MTR_synchronized_move_async(MTR_MotorHandle_t* handles, double* angles, uint8_t count,
                                     TickType_t deadline, CHD_CmdHandle_t* cmdHandle);

/**
 * @brief Queues update of TMC run current.
 *
 * @param[in] handle Target motor handle.
 * @param[in] current_a Run current in ampere.
 * @param[in] deadline Timeout/deadline for command.
 * @param[out] cmdHandle Optional command handle for completion wait.
 * @return kStatus_Success if queued, otherwise kStatus_Fail.
 */
status_t MTR_set_run_current_async(MTR_MotorHandle_t handle, double current_a, TickType_t deadline,
                                   CHD_CmdHandle_t* cmdHandle);

/**
 * @brief Queues update of TMC hold current.
 *
 * @param[in] handle Target motor handle.
 * @param[in] current_a Hold current in ampere.
 * @param[in] deadline Timeout/deadline for command.
 * @param[out] cmdHandle Optional command handle for completion wait.
 * @return kStatus_Success if queued, otherwise kStatus_Fail.
 */
status_t MTR_set_hold_current_async(MTR_MotorHandle_t handle, double current_a, TickType_t deadline,
                                    CHD_CmdHandle_t* cmdHandle);

/**
 * @brief Queues command to enable freewheeling mode.
 *
 * @param[in] handle Target motor handle.
 * @param[in] deadline Timeout/deadline for command.
 * @param[out] cmdHandle Optional command handle for completion wait.
 * @return kStatus_Success if queued, otherwise kStatus_Fail.
 */
status_t MTR_enable_freewheeling_async(MTR_MotorHandle_t handle, TickType_t deadline,
                                       CHD_CmdHandle_t* cmdHandle);

/**
 * @brief Queues command to disable freewheeling mode.
 *
 * @param[in] handle Target motor handle.
 * @param[in] deadline Timeout/deadline for command.
 * @param[out] cmdHandle Optional command handle for completion wait.
 * @return kStatus_Success if queued, otherwise kStatus_Fail.
 */
status_t MTR_disable_freewheeling_async(MTR_MotorHandle_t handle, TickType_t deadline,
                                        CHD_CmdHandle_t* cmdHandle);

/**
 * @brief Activates emergency stop latch.
 *
 * @param[in] handle Motor handle used for immediate stop action.
 * @return kStatus_Success if emergency stop applied, otherwise kStatus_Fail.
 */
status_t MTR_emergency_stop(MTR_MotorHandle_t handle);

/**
 * @brief Clears emergency stop latch and allows new motion commands.
 *
 * @return kStatus_Success if latch was cleared.
 */
status_t MTR_clear_emergency_stop(void);

/**
 * @brief Returns emergency-stop latch state.
 *
 * @return 1 if emergency stop is active, otherwise 0.
 */
uint8_t MTR_is_emergency_stop_active(void);

#endif // MOTOR_CORE_H_

/** @} */
