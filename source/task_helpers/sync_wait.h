/************************************************************
 * @file    sync_wait.h
 * @brief   Filedescription
 * @author  dg
 * @date    13 Apr 2026
 ************************************************************/

#ifndef SYNC_WAIT_H_
#define SYNC_WAIT_H_

/********************
 *     Includes    *
 ********************/
#include <stdint.h>
#include "FreeRTOS.h"
#include "event_groups.h"
#include "fsl_common.h"
#include "cmd_handle.h"

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

/** @brief Convert timeout in milliseconds to a deadline
 *
 * @param timeout_ms Timeout in milliseconds
 * @return Deadline in ticks
 */
static inline TickType_t SYW_deadline_from_timeout_ms(uint32_t timeout_ms)
{

    return xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
}

/**
 * @brief Wait for a single command to complete.
 *
 * Blocks until the command completes (success/failure/timeout) or the deadline is reached.
 *
 * @param[in] cmdHandle Command handle to wait for.
 * @param[in] deadline Deadline for waiting (in ticks).
 * @param[out] outBits Optional pointer to receive result bits (CHD_CMD_BIT_SUCCESS,
 *                      CHD_CMD_BIT_FAILURE, CHD_CMD_BIT_TIMEOUT). Can be NULL.
 *
 * @return kStatus_Success if command completed with CHD_CMD_BIT_SUCCESS bit set.
 *         kStatus_Fail if command has CHD_CMD_BIT_FAILURE bit set or deadline exceeded.
 *         kStatus_Timeout if CHD_CMD_BIT_TIMEOUT is set.
 *
 * @see CHD_CMD_BIT_SUCCESS
 * @see CHD_CMD_BIT_FAILURE
 * @see CHD_CMD_BIT_TIMEOUT
 */
status_t SYW_cmd_wait_result(CHD_CmdHandle_t cmdHandle, TickType_t deadline, EventBits_t* outBits);

/**
 * @brief Check a single command result without waiting.
 *
 * Polls the command result bits once and returns immediately.
 *
 * @param[in] cmdHandle Command handle to check.
 * @param[out] outBits Optional pointer to receive result bits (CHD_CMD_BIT_SUCCESS,
 *                      CHD_CMD_BIT_FAILURE, CHD_CMD_BIT_TIMEOUT). Can be NULL.
 *
 * @return kStatus_Success if command has CHD_CMD_BIT_SUCCESS bit set.
 *         kStatus_Fail if command has CHD_CMD_BIT_FAILURE bit set.
 *         kStatus_Timeout if CHD_CMD_BIT_TIMEOUT is set or command not completed yet.
 *
 * @see CHD_CMD_BIT_SUCCESS
 * @see CHD_CMD_BIT_FAILURE
 * @see CHD_CMD_BIT_TIMEOUT
 */
status_t SYW_cmd_check_result(CHD_CmdHandle_t cmdHandle, EventBits_t* outBits);

/**
 * @brief Wait for any of multiple commands to complete.
 *
 * Blocks until at least one command completes or the deadline is reached.
 *
 * @param[in] cmdHandles Array of command handles to wait for.
 * @param[in] count Number of handles in the array.
 * @param[in] deadline Deadline for waiting (in ticks).
 * @param[out] outCompletedIndex Optional pointer to receive index of first completed command.
 *                                Can be NULL.
 * @param[out] outBits Optional pointer to receive result bits (CHD_CMD_BIT_SUCCESS,
 *                      CHD_CMD_BIT_FAILURE, CHD_CMD_BIT_TIMEOUT) of the completed command.
 *                      Can be NULL.
 *
 * @return kStatus_Success if any command has CHD_CMD_BIT_SUCCESS bit set.
 *         kStatus_Fail if the first completed command has CHD_CMD_BIT_FAILURE bit set
 *                      or deadline exceeded.
 *         kStatus_Timeout if CHD_CMD_BIT_TIMEOUT is set on first completed command.
 *
 * @see CHD_CMD_BIT_SUCCESS
 * @see CHD_CMD_BIT_FAILURE
 * @see CHD_CMD_BIT_TIMEOUT
 */
status_t SYW_cmd_wait_any(CHD_CmdHandle_t* cmdHandles, size_t count, TickType_t deadline,
                          size_t* outCompletedIndex, EventBits_t* outBits);

/**
 * @brief Check whether any of multiple commands has completed, without waiting.
 *
 * Polls all command handles once and returns immediately.
 *
 * @param[in] cmdHandles Array of command handles to check.
 * @param[in] count Number of handles in the array.
 * @param[out] outCompletedIndex Optional pointer to receive index of first completed command.
 *                                Can be NULL.
 * @param[out] outBits Optional pointer to receive result bits (CHD_CMD_BIT_SUCCESS,
 *                      CHD_CMD_BIT_FAILURE, CHD_CMD_BIT_TIMEOUT) of the first completed command.
 *                      Can be NULL.
 *
 * @return kStatus_Success if any command has CHD_CMD_BIT_SUCCESS bit set.
 *         kStatus_Fail if the first completed command has CHD_CMD_BIT_FAILURE bit set.
 *         kStatus_Timeout if any command is still pending, or first completed has
 * CHD_CMD_BIT_TIMEOUT.
 *
 * @see CHD_CMD_BIT_SUCCESS
 * @see CHD_CMD_BIT_FAILURE
 * @see CHD_CMD_BIT_TIMEOUT
 */
status_t SYW_cmd_check_any(CHD_CmdHandle_t* cmdHandles, size_t count, size_t* outCompletedIndex,
                           EventBits_t* outBits);

/**
 * @brief Wait for all commands to complete.
 *
 * Blocks until all commands complete or the deadline is reached.
 *
 * @param[in] cmdHandles Array of command handles to wait for.
 * @param[in] count Number of handles in the array.
 * @param[in] deadline Deadline for waiting (in ticks).
 * @param[out] outResults Optional array to receive result bits (CHD_CMD_BIT_SUCCESS,
 *                         CHD_CMD_BIT_FAILURE, CHD_CMD_BIT_TIMEOUT) for each command.
 *                         Caller must allocate array of size count. Can be NULL.
 *
 * @return kStatus_Success if all commands have CHD_CMD_BIT_SUCCESS bit set.
 *         kStatus_Fail if any command has CHD_CMD_BIT_FAILURE bit set, deadline exceeded,
 *                      or not all commands completed.
 *         kStatus_Timeout if any command has CHD_CMD_BIT_TIMEOUT bit set.
 *
 * @see CHD_CMD_BIT_SUCCESS
 * @see CHD_CMD_BIT_FAILURE
 * @see CHD_CMD_BIT_TIMEOUT
 */
status_t SYW_cmd_wait_all(CHD_CmdHandle_t* cmdHandles, size_t count, TickType_t deadline,
                          EventBits_t* outResults);

/**
 * @brief Check whether all commands have completed, without waiting.
 *
 * Polls all command handles once and returns immediately.
 *
 * @param[in] cmdHandles Array of command handles to check.
 * @param[in] count Number of handles in the array.
 * @param[out] outResults Optional array to receive result bits (CHD_CMD_BIT_SUCCESS,
 *                         CHD_CMD_BIT_FAILURE, CHD_CMD_BIT_TIMEOUT) for each command.
 *                         Caller must allocate array of size count. Can be NULL.
 *
 * @return kStatus_Success if all commands have CHD_CMD_BIT_SUCCESS bit set.
 *         kStatus_Fail if any completed command has CHD_CMD_BIT_FAILURE bit set.
 *         kStatus_Timeout if any command is still pending, or any has CHD_CMD_BIT_TIMEOUT.
 *
 * @see CHD_CMD_BIT_SUCCESS
 * @see CHD_CMD_BIT_FAILURE
 * @see CHD_CMD_BIT_TIMEOUT
 */
status_t SYW_cmd_check_all(CHD_CmdHandle_t* cmdHandles, size_t count, EventBits_t* outResults);

#endif // SYNC_WAIT_H_