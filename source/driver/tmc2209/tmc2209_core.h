/************************************************************
 * @file    tmc2209_core.h
 * @brief   Public TMC2209 facade API for task-safe driver control.
 * @author  dg
 * @date    14 Apr 2026
 ************************************************************/

/**
 * @defgroup TMC2209_Module TMC2209 Module
 * @brief Public task-safe API for controlling TMC2209 driver settings.
 *
 * This module exposes the external TMC driver interface used by motor control.
 *
 * @{
 */

#ifndef TMC2209_CORE_H_
#define TMC2209_CORE_H_

/********************
 *     Includes    *
 ********************/
#include "tmc2209_shared.h"
#include "fsl_common.h"
#include "cmd_handle.h"
#include "peripherals.h"

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
 * @brief Initializes the TMC2209 module.
 *
 * This function initializes the TMC2209 module. It must be called before using any other
 * functions in this module.
 *
 * @note This function is NOT task-safe and must be called during system initialization before any
 * tasks start.
 * @note The TMC_task() should be started after this function completes successfully.
 *
 * @return kStatus_Success if the initialization is successful.
 *         kStatus_Fail if the initialization fails.
 *
 * @see TMC_task()
 */
status_t TMC_init(void);

/**
 * @brief Task function for processing TMC2209 commands.
 *
 * This function is intended to be run as a FreeRTOS task. It continuously
 * processes commands from the TMC command queue and communicates with the
 * TMC2209 motor drivers.
 *
 * @param[in] pvParameters Pointer to task parameters (not used in this implementation).
 *
 * @note This function should be started after TMC_init() completes successfully.
 * @note This task is blocked on the command queue and will wake when commands are queued.
 * @warning This function does not return; it is expected to run as a perpetual FreeRTOS task.
 *
 * @see TMC_init()
 */
void TMC_task(void* pvParameters);

/**
 * @brief Retrieves the default configuration for a TMC2209 driver handle.
 *
 * This function populates a TMC_Config_t structure with default values for initializing a TMC2209
 * driver handle.
 *
 * @param[out] config Pointer to a TMC_Config_t structure to be filled with default configuration
 * values. Must not be NULL.
 *
 * @return kStatus_Success if the default configuration was successfully retrieved and stored in the
 * provided structure. kStatus_Fail if the provided pointer is NULL or an error occurs while
 * retrieving
 */
status_t TMC_get_default_config(TMC_Config_t* config);

/**
 * @brief Initializes a TMC2209 handle with default configuration asynchronously.
 *
 * This function queues a default initialization command for a TMC2209 driver handle.
 * The initialization is performed asynchronously by the TMC_task.
 * The caller can await completion using the returned command handle.
 *
 * @param[in] config  TMC config structure to initialize the handle with.
 * @param[in] deadline Deadline for the initialization command.
 * @param[out] cmdHandle Pointer to receive command handle for awaiting completion.
 *
 * @return kStatus_Success if the command is successfully queued.
 *         kStatus_Fail if the queue is full, cmdHandle is NULL, or an error occurs.
 *
 * @see SYW_cmd_wait_result()
 */
status_t TMC_init_handle_async(TMC_Config_t config, TickType_t deadline,
                               CHD_CmdHandle_t* cmdHandle);

/**
 * @brief Retrieves a TMC2209 handle by its label.
 *
 * This function searches for a TMC2209 handle with the specified label and returns it if found.
 *
 * @param[in] label The label of the TMC2209 handle to retrieve. Must not be NULL.
 * @param[out] handle Pointer to a TMC_Handle_t variable where the found handle will be stored. Must
 * not be NULL.
 *
 * @return kStatus_Success if a handle with the specified label is found and returned.
 *         kStatus_Fail if the label is NULL, the handle pointer is NULL, or no matching handle is
 * found.
 */
status_t TMC_get_handle_by_label(const char* label, TMC_Handle_t* handle);

/**
 * @brief Sets the microstepping resolution for a TMC2209 driver asynchronously.
 *
 * This function queues a microstepping configuration command for a TMC2209 driver handle.
 * The operation is performed asynchronously by the TMC_task.
 * The caller can await completion using the returned command handle.
 *
 * @param[in] handle Handle for the TMC2209 driver to configure.
 * @param[in] microstepping The microstepping resolution to set (TMC_MICROSTEPPING_t).
 * @param[in] deadline Deadline for the operation.
 * @param[out] cmdHandle Pointer to receive command handle for awaiting completion.
 *
 * @return kStatus_Success if the command is successfully queued.
 *         kStatus_Fail if the queue is full, handle is NULL, cmdHandle is NULL, or an error occurs.
 *
 * @see TMC_MICROSTEPPING_t
 */
status_t TMC_set_microstepping_async(TMC_Handle_t handle, TMC_MICROSTEPPING_t microstepping,
                                     TickType_t deadline, CHD_CmdHandle_t* cmdHandle);

/**
 * @brief Converts a uint8_t microstepping value to the corresponding enum member.
 *
 * This function converts a raw uint8_t microstepping value to the corresponding
 * TMC_MICROSTEPPING_t enum value.
 *
 * @param[in] value The uint16_t microstepping value to convert.
 * @param[out] microstepping Pointer to store the converted enum value. Must not be NULL.
 *
 * @return kStatus_Success if the value matches a valid enum member.
 *         kStatus_Fail if the value does not correspond to any enum member.
 *
 * @see TMC_MICROSTEPPING_t
 */
status_t TMC_microstepping_uint_to_enum(uint16_t value, TMC_MICROSTEPPING_t* microstepping);

/**
 * @brief Converts a TMC2209 microstepping enum value to its raw register representation.
 *
 * This function converts a TMC_MICROSTEPPING_t value into the corresponding numeric microstepping
 * resolution used by higher-level configuration code.
 *
 * @param[in] in  The enum value to convert.
 * @param[out] out Pointer to store the converted numeric value. Must not be NULL.
 *
 * @return kStatus_Success if the conversion succeeded.
 *         kStatus_Fail if the output pointer is NULL or the enum value is not recognized.
 *
 * @see TMC_MICROSTEPPING_t
 */
status_t TMC_microstepping_enum_to_uint(TMC_MICROSTEPPING_t in, uint16_t* out);

/**
 * @brief Sets the IHOLD current divider asynchronously.
 *
 * Queues a command to set the hold current divider value.
 * The operation is executed in the TMC task context.
 * The caller can await completion using the returned command handle.
 *
 * @param[in] handle Handle for the TMC2209 driver to configure.
 * @param[in] ihold The hold current divider value (0-31).
 * @param[in] deadline Deadline for the operation.
 * @param[out] cmdHandle Pointer to receive command handle for awaiting completion.
 *
 * @return kStatus_Success if command was successfully queued.
 *         kStatus_Fail if queue operation failed, handle is NULL, or cmdHandle is NULL.
 */
status_t TMC_set_ihold_divider_async(TMC_Handle_t handle, uint8_t ihold, TickType_t deadline,
                                     CHD_CmdHandle_t* cmdHandle);

/**
 * @brief Sets the IRUN current divider asynchronously.
 *
 * Queues a command to set the run current divider value.
 * The operation is executed in the TMC task context.
 * The caller can await completion using the returned command handle.
 *
 * @param[in] handle Handle for the TMC2209 driver to configure.
 * @param[in] irun The run current divider value (0-31).
 * @param[in] deadline Deadline for the operation.
 * @param[out] cmdHandle Pointer to receive command handle for awaiting completion.
 *
 * @return kStatus_Success if command was successfully queued.
 *         kStatus_Fail if queue operation failed, handle is NULL, or cmdHandle is NULL.
 */
status_t TMC_set_irun_divider_async(TMC_Handle_t handle, uint8_t irun, TickType_t deadline,
                                    CHD_CmdHandle_t* cmdHandle);

/**
 * @brief Converts a desired current to a TMC2209 current divider value.
 *
 * Calculates the appropriate current divider (0-31) that achieves a desired output current
 * based on a reference current. Uses the formula:
 *       divider = (desired_current * 32 / reference_current) - 1
 *
 * The forumla for the reference current is derived from the TMC2209 datasheet (Rev. 1.09 p. 53)
 * and results in a current of approximately 1.77A at 100% (divider=0).
 *
 * The result is rounded according to the specified rounding mode to ensure valid divider values.
 * Dividers are limited to the valid range [0, 31] as per TMC2209 register bit width.
 *
 * @param[in] desired_current The target output current in milliamps.
 * @param[in] rounding Rounding mode for the divider calculation (FLOOR, CEIL, or NEAREST).
 * @param[out] divider Pointer to store the calculated divider value (0-31). Must not be NULL.
 *
 * @return kStatus_Success if the conversion succeeded and divider is valid (0-31).
 *         kStatus_Fail if divider parameter is NULL, currents are invalid (zero or negative, or
 * exceed reference current), or the calculated divider exceeds valid range.
 *
 * @note Divider values outside the valid range [0, 31] will cause the function to return
 * kStatus_Fail.
 * @note Higher divider values result in lower output currents.
 *
 * @see TMC_RoundingMode_t, TMC_set_ihold_divider(), TMC_set_irun_divider()
 */
status_t TMC_current_to_divider(float desired_current, TMC_RoundingMode_t rounding,
                                uint8_t* divider);

/**
 * @brief Enables freewheeling mode for a TMC2209 driver asynchronously.
 *
 * When enabled, the hold current is minimized, allowing the motor to rotate freely.
 *
 * @param[in] handle Handle for the TMC2209 driver to configure.
 * @param[in] deadline Deadline for the operation.
 * @param[out] cmdHandle Pointer to receive command handle for awaiting completion.
 *
 * @return kStatus_Success if the command is successfully queued.
 *         kStatus_Fail if the queue is full, handle is NULL, cmdHandle is NULL, or an error occurs.
 */
status_t TMC_enable_freewheeling_async(TMC_Handle_t handle, TickType_t deadline,
                                       CHD_CmdHandle_t* cmdHandle);

/**
 * @brief Disables freewheeling mode for a TMC2209 driver asynchronously.
 *
 * When disabled, the hold current is restored to its previous value.
 *
 * @param[in] handle Handle for the TMC2209 driver to configure.
 * @param[in] deadline Deadline for the operation.
 * @param[out] cmdHandle Pointer to receive command handle for awaiting completion.
 *
 * @return kStatus_Success if the command is successfully queued.
 *         kStatus_Fail if the queue is full, handle is NULL, cmdHandle is NULL, or an error occurs.
 */
status_t TMC_disable_freewheeling_async(TMC_Handle_t handle, TickType_t deadline,
                                        CHD_CmdHandle_t* cmdHandle);

#endif // TMC2209_CORE_H_

/** @} */
