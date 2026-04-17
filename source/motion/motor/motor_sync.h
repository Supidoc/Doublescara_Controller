/************************************************************
 * @file    motor_sync.h
 * @brief   Internal synchronized multi-motor movement helpers.
 * @author  dg
 * @date    15 Apr 2026
 ************************************************************/

/**
 * @defgroup MOTOR_Sync_Internal Motor Sync Internal
 * @brief Internal synchronized movement command handlers.
 * @ingroup MOTOR_Facade_Module
 * @{
 */

#ifndef MOTOR_SYNC_H_
#define MOTOR_SYNC_H_

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
 * @brief Executes synchronized movement for a set of motors.
 *
 * @param[in] handles Array of motor handles to command.
 * @param[in] angles Array of relative target angles in degrees.
 * @param[in] count Number of entries in @p handles and @p angles.
 * @param[in] deadline Timeout/deadline for synchronization sequence.
 * @return kStatus_Success on success, kStatus_Fail otherwise.
 */
status_t MTRi_synchronized_move(MTR_MotorHandle_t* handles, double* angles, uint8_t count,
                                TickType_t deadline);

#endif // MOTOR_SYNC_H_

/** @} */