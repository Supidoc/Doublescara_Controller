/************************************************************
 * @file    tmc2209_setup.h
 * @brief   Internal low-level setup helpers for TMC command handlers.
 * @author  dg
 * @date    14 Apr 2026
 ************************************************************/

/**
 * @defgroup TMC2209_Setup_Internal TMC2209 Setup Internal
 * @brief Internal helpers that directly apply configuration to TMC handles.
 * @ingroup TMC2209_Module
 * @{
 */

#ifndef TMC2209_SETUP_H_
#define TMC2209_SETUP_H_

#ifdef __cplusplus
extern "C"
{
#endif

/********************
 *     Includes    *
 ********************/
#include <stdint.h>
#include "FreeRTOS.h"
#include "tmc2209_shared.h"

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
     * @brief Allocates and initializes one TMC handle from config.
     * @param[in] config Handle configuration.
     * @param[in] deadline Timeout/deadline for UART operations.
     * @return kStatus_Success on success, kStatus_Fail otherwise.
     */
    status_t TMCi_init_handle(TMC_Config_t config, TickType_t deadline);

    /**
     * @brief Frees one previously allocated TMC handle.
     * @param[in] handle Handle to release.
     * @return kStatus_Success on success, kStatus_Fail otherwise.
     */
    status_t TMCi_free_handle(TMC_Handle_t handle);

    /**
     * @brief Writes IHOLD divider value to driver.
     * @param[in] handle Target handle.
     * @param[in] ihold IHOLD divider value.
     * @param[in] deadline Timeout/deadline for UART operations.
     * @return kStatus_Success on success, kStatus_Fail otherwise.
     */
    status_t TMCi_set_ihold_divider(TMC_Handle_t handle, uint8_t ihold, TickType_t deadline);

    /**
     * @brief Writes IRUN divider value to driver.
     * @param[in] handle Target handle.
     * @param[in] irun IRUN divider value.
     * @param[in] deadline Timeout/deadline for UART operations.
     * @return kStatus_Success on success, kStatus_Fail otherwise.
     */
    status_t TMCi_set_irun_divider(TMC_Handle_t handle, uint8_t irun, TickType_t deadline);

    /**
     * @brief Applies microstepping configuration.
     * @param[in] handle Target handle.
     * @param[in] microstepping Microstepping enum value.
     * @param[in] deadline Timeout/deadline for UART operations.
     * @return kStatus_Success on success, kStatus_Fail otherwise.
     */
    status_t TMCi_set_microstepping(TMC_Handle_t handle, TMC_MICROSTEPPING_t microstepping,
                                    TickType_t deadline);

    /**
     * @brief Enables or disables double-edge step mode.
     * @param[in] handle Target handle.
     * @param[in] enabled Non-zero to enable, zero to disable.
     * @param[in] deadline Timeout/deadline for UART operations.
     * @return kStatus_Success on success, kStatus_Fail otherwise.
     */
    status_t TMCi_set_double_edge_step_pulse(TMC_Handle_t handle, uint8_t enabled,
                                             TickType_t deadline);

    /**
     * @brief Enables or disables freewheeling mode.
     * @param[in] handle Target handle.
     * @param[in] enabled Non-zero to enable, zero to disable.
     * @param[in] deadline Timeout/deadline for UART operations.
     * @return kStatus_Success on success, kStatus_Fail otherwise.
     */
    status_t TMCi_set_freewheeling(TMC_Handle_t handle, uint8_t enabled, TickType_t deadline);

    /**
     * @brief Converts register microstepping value to enum representation.
     * @param[in] in Register/raw value.
     * @param[out] out Destination enum pointer.
     * @return kStatus_Success on success, kStatus_Fail otherwise.
     */
    status_t TMCi_microstepping_uint_to_enum_impl(uint16_t in, TMC_MICROSTEPPING_t* out);

    /**
     * @brief Converts enum microstepping value to register representation.
     * @param[in] in Enum microstepping value.
     * @param[out] out Destination raw value pointer.
     * @return kStatus_Success on success, kStatus_Fail otherwise.
     */
    status_t TMC_microstepping_enum_to_uint_impl(TMC_MICROSTEPPING_t in, uint16_t* out);

#ifdef __cplusplus
}
#endif

#endif // TMC2209_SETUP_H_

/** @} */
