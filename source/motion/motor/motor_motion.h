/************************************************************
 * @file    motor_motion.h
 * @brief   Internal motor motion command implementations.
 * @author  dg
 * @date    15 Apr 2026
 ************************************************************/

/**
 * @defgroup MOTOR_Motion_Internal Motor Motion Internal
 * @brief Internal handlers for movement-related motor commands.
 * @ingroup MOTOR_Facade_Module
 * @{
 */

#ifndef MOTOR_MOTION_H_
#define MOTOR_MOTION_H_

/********************
 *     Includes    *
 ********************/
#include "motor_internal.h"

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
status_t MTRi_move_revolutions(MTR_MotorHandle_t handle, double revolutions, TickType_t deadline,
                               MTR_ParallelTaskItem* taskItem);

/**
 * @brief Applies target velocity for a motor.
 *
 * @param[in] handle Target motor handle.
 * @param[in] velocity_deg_per_sec Velocity in degrees per second.
 * @param[in] deadline Timeout/deadline for dependent async commands.
 * @param[in,out] taskItem Parallel task item collecting sub-command handles.
 * @return kStatus_Success on success, kStatus_Fail otherwise.
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
status_t MTRi_stop_motor(MTR_MotorHandle_t handle, bool decelerate, TickType_t deadline,
                         MTR_ParallelTaskItem* taskItem);

/**
 * @brief Reads current motor angle.
 *
 * @param[in] handle Target motor handle.
 * @param[out] angle Pointer receiving angle in degrees.
 * @param[in] deadline Timeout/deadline for dependent async commands.
 * @return kStatus_Success on success, kStatus_Fail otherwise.
 */
status_t MTRi_get_current_angle(MTR_MotorHandle_t handle, double* angle, TickType_t deadline);

/**
 * @brief Reads current movement state for a motor.
 *
 * @param[in] handle Target motor handle.
 * @param[out] state Pointer receiving movement state.
 * @return kStatus_Success on success, kStatus_Fail otherwise.
 */
status_t MTRi_get_movement_state(MTR_MotorHandle_t handle, STP_MovementState_t* state);

/**
 * @brief Sets current motor position as home/zero.
 *
 * @param[in] handle Target motor handle.
 * @return kStatus_Success on success, kStatus_Fail otherwise.
 */
status_t MTRi_set_home_position(MTR_MotorHandle_t handle);

#endif // MOTOR_MOTION_H_

/** @} */