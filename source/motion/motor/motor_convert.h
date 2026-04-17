/************************************************************
 * @file    motor_convert.h
 * @brief   Motor unit-conversion helper utilities.
 * @author  dg
 * @date    15 Apr 2026
 ************************************************************/

/**
 * @defgroup MOTOR_Convert_Internal Motor Convert Internal
 * @brief Internal helpers for converting between step and angle domains.
 * @ingroup MOTOR_Facade_Module
 * @{
 */

#ifndef MOTOR_CONVERT_H_
#define MOTOR_CONVERT_H_

/********************
 *     Includes    *
 ********************/
#include "motor_core.h"

/***********************************
 *     Public Macros / Defines     *
 ***********************************/

/***************************
 *     Public Typedefs     *
 ***************************/

typedef enum _MTR_roundingMethod
{
    ROUND_DOWN,
    ROUND_UP,
    ROUND_NEAREST
} MTR_roundingMethod_t;

/****************************
 *     Public Variables     *
 ****************************/

/**************************************
 *     Public Function Prototypes    *
 **************************************/

/**
 * @brief Converts step count to mechanical angle.
 *
 * @param[in] handle Motor handle containing step geometry.
 * @param[in] steps Signed step count.
 * @return Converted angle in degrees.
 */
double MTR_steps_to_angle(MTR_MotorHandle_t handle, int32_t steps);

/**
 * @brief Converts mechanical angle to step count.
 *
 * @param[in] handle Motor handle containing step geometry.
 * @param[in] angle Angle in degrees.
 * @param[in] method Rounding method for non-integer step results.
 * @return Signed step count.
 */
int32_t MTR_angle_to_steps(MTR_MotorHandle_t handle, double angle, MTR_roundingMethod_t method);

#endif // MOTOR_CONVERT_H_

/** @} */