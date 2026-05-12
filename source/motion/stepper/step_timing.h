/************************************************************
 * @file    step_timing.h
 * @brief   Internal timing and ISR-cache helpers for step generation.
 * @author  dg
 * @date    13 Apr 2026
 ************************************************************/

/**
 * @defgroup STEPPER_Timing_Internal Stepper Timing Internal
 * @brief Internal helpers for movement completion checks and ISR handle cache.
 * @ingroup STEPPER_Module
 * @{
 */

#ifndef STEP_TIMING_H_
#define STEP_TIMING_H_

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
     * @brief Checks whether active movement completed and finalizes state.
     *
     * @return 1 if movement transitioned to complete state, otherwise 0.
     */
    uint8_t STPi_check_movement_completion(void);

    /**
     * @brief Refreshes ISR lookup cache for FTM channel to handle mapping.
     */
    void STPi_update_isr_handle_cache(void);

#ifdef __cplusplus
}
#endif

#endif // STEP_TIMING_H_

/** @} */