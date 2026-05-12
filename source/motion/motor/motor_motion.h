#/************************************************************
 * @file motor_motion.h
 * @brief Internal motor motion command interfaces.
 *
 * This header exposes the internal movement-related command APIs used by
 * the motor facade. Functions here enqueue or query stepper operations and
 * are intended for use inside the motor subsystem (not part of the public
 * external API).
 *
 * @author dg
 * @date 15 Apr 2026
 */

/**
 * @defgroup MOTOR_Motion_Internal Motor Motion Internal
 * @brief Internal handlers for movement-related motor commands.
 * @ingroup MOTOR_Facade_Module
 * @{
 */

#ifndef MOTOR_MOTION_H_
#define MOTOR_MOTION_H_

#ifdef __cplusplus
extern "C"
{
#endif

/********************
 *     Includes    *
 ********************/
#include "motor_internal.h"

#include <stdbool.h>

    /***********************************
     *     Public Macros / Defines     *
     ***********************************/

    /***************************
     *     Public Typedefs     *
     ***************************/

    /****************************
     *     Public Variables     *
     ****************************/

    /**************************************
     *     Public Function Prototypes    *
     **************************************/

    /**
     * @brief Executes relative motor movement by angle.
     *
     * @param[in] handle Target motor handle.
     * @param[in] angle Relative angle in degrees.
     * @param[in] deadline Timeout/deadline for dependent async commands.
     * @param[in,out] taskItem Parallel task item collecting sub-command handles.
     * @return kStatus_Success on success, kStatus_Fail otherwise.
     */
    /**
     * @brief Execute a relative rotation on a motor.
     *
     * Enqueues an asynchronous relative move on the motor's stepper. The
     * movement is specified in degrees and converted to step counts by the
     * motor conversion helpers.
     *
     * @param[in] handle Target motor handle (must not be NULL).
     * @param[in] angle Relative angle in degrees (positive = forward).
     * @param[in] deadline Tick-based timeout/deadline for dependent async commands.
     * @param[in,out] taskItem Parallel task item receiving sub-command handle(s).
     * @retval kStatus_Success Command successfully queued.
     * @retval kStatus_Fail On error (NULL pointers, emergency stop or queue failure).
     */
    status_t MTRi_move_angle(MTR_MotorHandle_t handle, double angle, TickType_t deadline,
                             MTR_ParallelTaskItem* taskItem);

    /**
     * @brief Executes movement to absolute mechanical angle.
     *
     * @param[in] handle Target motor handle.
     * @param[in] angle Absolute angle in degrees.
     * @param[in] deadline Timeout/deadline for dependent async commands.
     * @param[in,out] taskItem Parallel task item collecting sub-command handles.
     * @return kStatus_Success on success, kStatus_Fail otherwise.
     */
    /**
     * @brief Move motor to an absolute mechanical angle.
     *
     * Calculates the difference between the requested absolute angle and the
     * current position, then enqueues a relative move. Angle units are degrees.
     *
     * @param[in] handle Target motor handle (must not be NULL).
     * @param[in] angle Absolute target angle in degrees.
     * @param[in] deadline Tick-based timeout/deadline for dependent async commands.
     * @param[in,out] taskItem Parallel task item receiving sub-command handle(s).
     * @retval kStatus_Success Command successfully queued.
     * @retval kStatus_Fail On error (NULL pointers, emergency stop, sensor read or queue failure).
     */
    status_t MTRi_move_absolute_angle(MTR_MotorHandle_t handle, double angle, TickType_t deadline,
                                      MTR_ParallelTaskItem* taskItem);

    /**
     * @brief Executes relative motor movement in revolutions.
     *
     * @param[in] handle Target motor handle.
     * @param[in] revolutions Relative revolutions.
     * @param[in] deadline Timeout/deadline for dependent async commands.
     * @param[in,out] taskItem Parallel task item collecting sub-command handles.
     * @return kStatus_Success on success, kStatus_Fail otherwise.
     */
    /**
     * @brief Move motor by a number of revolutions.
     *
     * Convenience wrapper that converts revolutions to degrees and calls
     * `MTRi_move_angle`.
     *
     * @param[in] handle Target motor handle.
     * @param[in] revolutions Relative revolutions (can be fractional).
     * @param[in] deadline Tick-based timeout/deadline for dependent async commands.
     * @param[in,out] taskItem Parallel task item receiving sub-command handle(s).
     * @retval kStatus_Success Command successfully queued.
     * @retval kStatus_Fail On error.
     */
    status_t MTRi_move_revolutions(MTR_MotorHandle_t handle, double revolutions,
                                   TickType_t deadline, MTR_ParallelTaskItem* taskItem);

    /**
     * @brief Applies target velocity for a motor.
     *
     * @param[in] handle Target motor handle.
     * @param[in] velocity_deg_per_sec Velocity in degrees per second.
     * @param[in] deadline Timeout/deadline for dependent async commands.
     * @param[in,out] taskItem Parallel task item collecting sub-command handles.
     * @return kStatus_Success on success, kStatus_Fail otherwise.
     */
    /**
     * @brief Set target end velocity for the motor (deg/s).
     *
     * The velocity is applied as an end velocity for subsequent motion planning
     * and is converted into step units for the stepper layer.
     *
     * @param[in] handle Target motor handle.
     * @param[in] velocity_deg_per_sec Velocity in degrees per second.
     * @param[in] deadline Tick-based timeout/deadline for dependent async commands.
     * @param[in,out] taskItem Parallel task item receiving sub-command handle(s).
     * @retval kStatus_Success Command successfully queued.
     * @retval kStatus_Fail On error.
     */
    status_t MTRi_set_velocity(MTR_MotorHandle_t handle, double velocity_deg_per_sec,
                               TickType_t deadline, MTR_ParallelTaskItem* taskItem);

    /**
     * @brief Applies target acceleration for a motor.
     *
     * @param[in] handle Target motor handle.
     * @param[in] acceleration_deg_per_sec2 Acceleration in degrees per second squared.
     * @param[in] deadline Timeout/deadline for dependent async commands.
     * @param[in,out] taskItem Parallel task item collecting sub-command handles.
     * @return kStatus_Success on success, kStatus_Fail otherwise.
     */
    /**
     * @brief Set target acceleration for the motor (deg/s^2).
     *
     * @param[in] handle Target motor handle.
     * @param[in] acceleration_deg_per_sec2 Acceleration in degrees per second squared.
     * @param[in] deadline Tick-based timeout/deadline for dependent async commands.
     * @param[in,out] taskItem Parallel task item receiving sub-command handle(s).
     * @retval kStatus_Success Command successfully queued.
     * @retval kStatus_Fail On error.
     */
    status_t MTRi_set_acceleration(MTR_MotorHandle_t handle, double acceleration_deg_per_sec2,
                                   TickType_t deadline, MTR_ParallelTaskItem* taskItem);

    /**
     * @brief Stops an active motor movement.
     *
     * @param[in] handle Target motor handle.
     * @param[in] decelerate True to decelerate to stop, false for immediate stop.
     * @param[in] deadline Timeout/deadline for dependent async commands.
     * @param[in,out] taskItem Parallel task item collecting sub-command handles.
     * @return kStatus_Success on success, kStatus_Fail otherwise.
     */
    /**
     * @brief Request motor stop.
     *
     * Enqueues a stop command. When `decelerate` is true the motor will perform a
     * controlled deceleration, otherwise an immediate stop is requested.
     *
     * @param[in] handle Target motor handle.
     * @param[in] decelerate True to request controlled deceleration, false for immediate.
     * @param[in] deadline Tick-based timeout/deadline for dependent async commands.
     * @param[in,out] taskItem Parallel task item receiving sub-command handle(s).
     * @retval kStatus_Success Command successfully queued.
     * @retval kStatus_Fail On error.
     */
    status_t MTRi_stop_motor(MTR_MotorHandle_t handle, bool decelerate, TickType_t deadline,
                             MTR_ParallelTaskItem* taskItem);

    /**
     * @brief Reads current motor angle.
     *
     * @param[in] handle Target motor handle.
     * @param[out] angle Pointer receiving angle in degrees.
     * @return kStatus_Success on success, kStatus_Fail otherwise.
     */
    /**
     * @brief Read the current motor angle in degrees.
     *
     * @param[in] handle Target motor handle.
     * @param[out] angle Pointer to receive angle in degrees (must not be NULL).
     * @retval kStatus_Success Angle successfully read and converted.
     * @retval kStatus_Fail On error.
     */
    status_t MTRi_get_current_angle(MTR_MotorHandle_t handle, double* angle);

    /**
     * @brief Reads current movement state for a motor.
     *
     * @param[in] handle Target motor handle.
     * @param[out] state Pointer receiving movement state.
     * @return kStatus_Success on success, kStatus_Fail otherwise.
     */
    /**
     * @brief Query the current movement state from the stepper layer.
     *
     * @param[in] handle Target motor handle.
     * @param[out] state Pointer to receive movement state (must not be NULL).
     * @retval kStatus_Success State retrieved.
     * @retval kStatus_Fail On error.
     */
    status_t MTRi_get_movement_state(MTR_MotorHandle_t handle, STP_MovementState_t* state);

    /**
     * @brief Sets current motor position as home/zero.
     *
     * @param[in] handle Target motor handle.
     * @return kStatus_Success on success, kStatus_Fail otherwise.
     */
    /**
     * @brief Mark the current physical position as home (zero).
     *
     * This resets the stepper absolute position counters so the current
     * position becomes the logical zero/home.
     *
     * @param[in] handle Target motor handle.
     * @retval kStatus_Success Home set.
     * @retval kStatus_Fail On error.
     */
    status_t MTRi_set_home_position(MTR_MotorHandle_t handle);

    /**
     * @brief Sets current physical position to a specified angle offset from home.
     *
     * @param[in] handle Target motor handle.
     * @param[in] angle_offset_deg Desired angle at current position in degrees.
     * @return kStatus_Success on success, kStatus_Fail otherwise.
     */
    /**
     * @brief Set the motor's absolute position such that the current physical
     *        position maps to `angle_offset_deg`.
     *
     * Useful for applying an offset without moving the motor.
     *
     * @param[in] handle Target motor handle.
     * @param[in] angle_offset_deg Desired logical angle at the current position.
     * @retval kStatus_Success Offset applied.
     * @retval kStatus_Fail On error.
     */
    status_t MTRi_set_home_angle_offset(MTR_MotorHandle_t handle, double angle_offset_deg);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_MOTION_H_

/** @} */
