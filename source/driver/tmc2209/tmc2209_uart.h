/************************************************************
 * @file    tmc2209_uart.h
 * @brief   Internal UART transport helpers for TMC2209 communication.
 * @author  dg
 * @date    14 Apr 2026
 ************************************************************/

/**
 * @defgroup TMC2209_UART_Internal TMC2209 UART Internal
 * @brief Internal UART read/write primitives used by TMC setup/process layers.
 * @ingroup TMC2209_Module
 * @{
 */

#ifndef TMC2209_UART_H_
#define TMC2209_UART_H_

/********************
 *     Includes    *
 ********************/
#include "tmc2209_shared.h"
#include "tmc2209_internal.h"
#include "stdint.h"
#include "FreeRTOS.h"

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
 * @brief Reads a 32-bit register from a TMC2209 device.
 * @param[in] handle Target TMC handle.
 * @param[in] regAddr Register address.
 * @param[out] data Destination pointer for register value.
 * @param[in] deadline Timeout/deadline for UART transaction.
 * @return kStatus_Success on success, kStatus_Fail otherwise.
 */
status_t TMCi_read(TMC_Handle_t handle, uint8_t regAddr, uint32_t* data, TickType_t deadline);

/**
 * @brief Writes a 32-bit register to a TMC2209 device.
 * @param[in] handle Target TMC handle.
 * @param[in] regAddr Register address.
 * @param[in] data Pointer to value to write.
 * @param[in] deadline Timeout/deadline for UART transaction.
 * @return kStatus_Success on success, kStatus_Fail otherwise.
 */
status_t TMCi_write(TMC_Handle_t handle, uint8_t regAddr, uint32_t* data, TickType_t deadline);

/**
 * @brief Reads transmission counter from TMC interface.
 * @param[in] handle Target TMC handle.
 * @param[in] deadline Timeout/deadline for UART transaction.
 * @return kStatus_Success on success, kStatus_Fail otherwise.
 */
status_t TMCi_read_transmissioncount(TMC_Handle_t handle, TickType_t deadline);

#endif // TMC2209_UART_H_

/** @} */
