/************************************************************
 * @file    motor_init.h
 * @brief   Internal motor-handle initialization helpers.
 * @author  dg
 * @date    15 Apr 2026
 ************************************************************/

/**
 * @defgroup MOTOR_Init_Internal Motor Init Internal
 * @brief Internal helpers for creating and configuring motor handles.
 * @ingroup MOTOR_Facade_Module
 * @{
 */

#ifndef MOTOR_INIT_H_
#define MOTOR_INIT_H_

#ifdef __cplusplus
extern "C"
{
#endif

/********************
 *     Includes    *
 ********************/
#include "motor_core.h"
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
     * @brief Initializes one motor handle from config and stores it in handle pool.
     *
     * @param[in] config Motor configuration to apply.
     * @param[in] deadline Deadline used for dependent async commands.
     * @param[in,out] handles Handle array to allocate from.
     * @param[in] maxHandles Number of entries in @p handles.
     *
     * @note Intended for internal command dispatch path only.
     *
     * @return kStatus_Success on success, kStatus_Fail otherwise.
     */
    status_t MTR_init_motor(MTR_MotorConfig_t config, TickType_t deadline,
                            MTR_HandlesArrayItem_t* handles, uint8_t maxHandles);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_INIT_H_

/** @} */