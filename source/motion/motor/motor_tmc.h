/************************************************************
 * @file    motor_tmc.h
 * @brief   Internal TMC-specific motor command handlers.
 * @author  dg
 * @date    15 Apr 2026
 ************************************************************/

/**
 * @defgroup MOTOR_TMC_Internal Motor TMC Internal
 * @brief Internal handlers for TMC current and freewheeling commands.
 * @ingroup MOTOR_Facade_Module
 * @{
 */

#ifndef MOTOR_TMC_H_
#define MOTOR_TMC_H_

#ifdef __cplusplus
extern "C"
{
#endif

/********************
 *     Includes    *
 ********************/
#include "motor_internal.h"

#include <stdint.h>
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
     * @brief Sets TMC run current for a motor.
     *
     * @param[in] handle Target motor handle.
     * @param[in] current_a Run current in ampere.
     * @param[in] deadline Timeout/deadline for dependent async commands.
     * @return kStatus_Success on success, kStatus_Fail otherwise.
     */
    status_t MTRi_set_run_current(MTR_MotorHandle_t handle, double current_a, TickType_t deadline);

    /**
     * @brief Sets TMC hold current for a motor.
     *
     * @param[in] handle Target motor handle.
     * @param[in] current_a Hold current in ampere.
     * @param[in] deadline Timeout/deadline for dependent async commands.
     * @return kStatus_Success on success, kStatus_Fail otherwise.
     */
    status_t MTRi_set_hold_current(MTR_MotorHandle_t handle, double current_a, TickType_t deadline);

    /**
     * @brief Enables freewheeling mode for a motor.
     *
     * @param[in] handle Target motor handle.
     * @param[in] deadline Timeout/deadline for dependent async commands.
     * @return kStatus_Success on success, kStatus_Fail otherwise.
     */
    status_t MTRi_enable_freewheeling(MTR_MotorHandle_t handle, TickType_t deadline);

    /**
     * @brief Disables freewheeling mode for a motor.
     *
     * @param[in] handle Target motor handle.
     * @param[in] deadline Timeout/deadline for dependent async commands.
     * @return kStatus_Success on success, kStatus_Fail otherwise.
     */
    status_t MTRi_disable_freewheeling(MTR_MotorHandle_t handle, TickType_t deadline);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_TMC_H_

/** @} */