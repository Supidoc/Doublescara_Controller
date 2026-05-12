/************************************************************
 * @file    step_profile.h
 * @brief   Internal motion-profile planning helpers for steppers.
 * @author  dg
 * @date    13 Apr 2026
 ************************************************************/

/**
 * @defgroup STEPPER_Profile_Internal Stepper Profile Internal
 * @brief Internal acceleration/deceleration profile planning utilities.
 * @ingroup STEPPER_Module
 * @{
 */

#ifndef STEP_PROFILE_H_
#define STEP_PROFILE_H_

#ifdef __cplusplus
extern "C"
{
#endif

/********************
 *     Includes    *
 ********************/
#include "step_internal.h"

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
     * @brief Calculates and stores motion profile parameters for a handle.
     *
     * @param[in,out] handle Stepper handle whose movement profile is planned.
     * @return kStatus_Success on success, kStatus_Fail otherwise.
     */
    status_t STPi_plan_motion_profile(STP_Handle_t handle);

    /**
     * @brief Resets runtime movement bookkeeping fields.
     *
     * @param[in,out] handle Movement handle to reset.
     * @return kStatus_Success on success, kStatus_Fail otherwise.
     */
    status_t STPi_reset_movement_handle(STP_StepperMovement_t* handle);

#ifdef __cplusplus
}
#endif

#endif // STEP_PROFILE_H_

/** @} */