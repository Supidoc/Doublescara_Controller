/************************************************************
 * @file    task_helpers.h
 * @brief   Helper macros and functions for task-safe operations
 * @author  dg
 * @date    9 Mar 2026
 ************************************************************/

#ifndef TASK_HELPERS_H_
#define TASK_HELPERS_H_

/** @defgroup TaskHelpers_Module Task Helpers
 * @brief Helper macros and functions for task-safe operations
 * @{
 */

/********************
 *     Includes    *
 ********************/
#include "FreeRTOS.h"
#include "task.h"
#include "fsl_common.h"
#include "log.h"
#include "string.h"
#include "event_groups.h"
#include "queue.h"

/***********************************
 *     Public Macros / Defines     *
 ***********************************/

/**
 * @defgroup THE_EventBits Task Helper Event Bits
 * @brief Bitmasks for command completion status tracking
 * @{
 */

/** @brief Command completed successfully */
#define THE_CMD_BIT_SUCCESS (1 << 0)

/** @brief Command failed */
#define THE_CMD_BIT_FAILURE (1 << 1)

/** @brief Command timed out */
#define THE_CMD_BIT_TIMEOUT (1 << 2)

/** @} */

/***************************
 *     Public Typedefs     *
 ***************************/

typedef struct _THE_CmdHandleImpl
{
    EventGroupHandle_t eventGroup;
    uint8_t            used;
    uint8_t            ref_count;
} THE_CmdHandleImpl_t;

typedef THE_CmdHandleImpl_t* THE_CmdHandle_t;

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
static inline TickType_t THE_deadline_from_timeout_ms(uint32_t timeout_ms)
{

    return xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
}

/**
 * @brief Initialize the command handle pool
 * @details Sets up the command handle infrastructure for tracking asynchronous
 *          command execution. Must be called once before using command handles.
 * @param[in] cmdHandles Pointer to array of command handle structures
 * @param[in] maxHandles Maximum number of handles in the pool
 */
void THE_init_cmd_handles(THE_CmdHandleImpl_t* cmdHandles, size_t maxHandles);

/**
 * @brief Notify successful completion of a command
 * @details Sets the THE_CMD_BIT_SUCCESS bit in the command handle's event group
 * @param[in] cmdHandle Command handle to notify
 */
void THE_notify_task_success(THE_CmdHandle_t cmdHandle);

/**
 * @brief Notify failure of a command
 * @details Sets the THE_CMD_BIT_FAILURE bit in the command handle's event group
 * @param[in] cmdHandle Command handle to notify
 */
void THE_notify_task_failure(THE_CmdHandle_t cmdHandle);

/**
 * @brief Notify timeout of a command
 * @details Sets the THE_CMD_BIT_TIMEOUT bit in the command handle's event group
 * @param[in] cmdHandle Command handle to notify
 */
void THE_notify_task_timeout(THE_CmdHandle_t cmdHandle);

/**
 * @brief Get an available command handle from the pool
 * @param[out] cmdHandle Pointer to receive the allocated command handle
 * @param[in] cmdHandles Pointer to array of command handle structures
 * @param[in] maxHandles Maximum number of handles in the pool
 * @return kStatus_Success if a handle was allocated
 *         kStatus_Fail if no handles are available
 */
status_t THE_get_cmd_handle(THE_CmdHandle_t* cmdHandle, THE_CmdHandleImpl_t* cmdHandles,
                            size_t maxHandles);

/**
 * @brief Increment reference count for a command handle
 * @details Used to track multiple references to the same command handle
 * @param[in] cmdHandle Command handle to increment reference for
 */
void THE_add_cmd_handle_ref(THE_CmdHandle_t cmdHandle);

/**
 * @brief Decrement reference count for a command handle
 * @details When reference count reaches zero, the handle is returned to the pool
 * @param[in] cmdHandle Command handle to decrement reference for
 */
void THE_remove_cmd_handle_ref(THE_CmdHandle_t cmdHandle);

/**
 * @brief Send a command to a queue with deadline tracking
 * @param[in] xQueue FreeRTOS queue to send the command to
 * @param[in] pvItemToQueue Pointer to the item to send
 * @param[in] deadline Deadline for command completion (in ticks)
 * @param[in] cmdHandle Command handle for tracking this operation
 * @return kStatus_Success if command was queued successfully
 *         kStatus_Fail if queue is full or deadline is invalid
 */
status_t THE_send_cmd(QueueHandle_t xQueue, const void* const pvItemToQueue, TickType_t deadline,
                      THE_CmdHandle_t cmdHandle);

/**
 * @brief Wait for a single command to complete.
 *
 * Blocks until the command completes (success/failure/timeout) or the deadline is reached.
 *
 * @param[in] cmdHandle Command handle to wait for.
 * @param[in] deadline Deadline for waiting (in ticks).
 * @param[out] outBits Optional pointer to receive result bits (THE_CMD_BIT_SUCCESS,
 *                      THE_CMD_BIT_FAILURE, THE_CMD_BIT_TIMEOUT). Can be NULL.
 *
 * @return kStatus_Success if command completed with THE_CMD_BIT_SUCCESS bit set.
 *         kStatus_Fail if command has THE_CMD_BIT_FAILURE bit set or deadline exceeded.
 *         kStatus_Timeout if THE_CMD_BIT_TIMEOUT is set.
 *
 * @see THE_CMD_BIT_SUCCESS
 * @see THE_CMD_BIT_FAILURE
 * @see THE_CMD_BIT_TIMEOUT
 */
status_t THE_cmd_wait_result(THE_CmdHandle_t cmdHandle, TickType_t deadline, EventBits_t* outBits);

/**
 * @brief Check a single command result without waiting.
 *
 * Polls the command result bits once and returns immediately.
 *
 * @param[in] cmdHandle Command handle to check.
 * @param[out] outBits Optional pointer to receive result bits (THE_CMD_BIT_SUCCESS,
 *                      THE_CMD_BIT_FAILURE, THE_CMD_BIT_TIMEOUT). Can be NULL.
 *
 * @return kStatus_Success if command has THE_CMD_BIT_SUCCESS bit set.
 *         kStatus_Fail if command has THE_CMD_BIT_FAILURE bit set.
 *         kStatus_Timeout if THE_CMD_BIT_TIMEOUT is set or command not completed yet.
 *
 * @see THE_CMD_BIT_SUCCESS
 * @see THE_CMD_BIT_FAILURE
 * @see THE_CMD_BIT_TIMEOUT
 */
status_t THE_cmd_check_result(THE_CmdHandle_t cmdHandle, EventBits_t* outBits);

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
 * @param[out] outBits Optional pointer to receive result bits (THE_CMD_BIT_SUCCESS,
 *                      THE_CMD_BIT_FAILURE, THE_CMD_BIT_TIMEOUT) of the completed command.
 *                      Can be NULL.
 *
 * @return kStatus_Success if any command has THE_CMD_BIT_SUCCESS bit set.
 *         kStatus_Fail if the first completed command has THE_CMD_BIT_FAILURE bit set
 *                      or deadline exceeded.
 *         kStatus_Timeout if THE_CMD_BIT_TIMEOUT is set on first completed command.
 *
 * @see THE_CMD_BIT_SUCCESS
 * @see THE_CMD_BIT_FAILURE
 * @see THE_CMD_BIT_TIMEOUT
 */
status_t THE_cmd_wait_any(THE_CmdHandle_t* cmdHandles, size_t count, TickType_t deadline,
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
 * @param[out] outBits Optional pointer to receive result bits (THE_CMD_BIT_SUCCESS,
 *                      THE_CMD_BIT_FAILURE, THE_CMD_BIT_TIMEOUT) of the first completed command.
 *                      Can be NULL.
 *
 * @return kStatus_Success if any command has THE_CMD_BIT_SUCCESS bit set.
 *         kStatus_Fail if the first completed command has THE_CMD_BIT_FAILURE bit set.
 *         kStatus_Timeout if any command is still pending, or first completed has
 * THE_CMD_BIT_TIMEOUT.
 *
 * @see THE_CMD_BIT_SUCCESS
 * @see THE_CMD_BIT_FAILURE
 * @see THE_CMD_BIT_TIMEOUT
 */
status_t THE_cmd_check_any(THE_CmdHandle_t* cmdHandles, size_t count, size_t* outCompletedIndex,
                           EventBits_t* outBits);

/**
 * @brief Wait for all commands to complete.
 *
 * Blocks until all commands complete or the deadline is reached.
 *
 * @param[in] cmdHandles Array of command handles to wait for.
 * @param[in] count Number of handles in the array.
 * @param[in] deadline Deadline for waiting (in ticks).
 * @param[out] outResults Optional array to receive result bits (THE_CMD_BIT_SUCCESS,
 *                         THE_CMD_BIT_FAILURE, THE_CMD_BIT_TIMEOUT) for each command.
 *                         Caller must allocate array of size count. Can be NULL.
 *
 * @return kStatus_Success if all commands have THE_CMD_BIT_SUCCESS bit set.
 *         kStatus_Fail if any command has THE_CMD_BIT_FAILURE bit set, deadline exceeded,
 *                      or not all commands completed.
 *         kStatus_Timeout if any command has THE_CMD_BIT_TIMEOUT bit set.
 *
 * @see THE_CMD_BIT_SUCCESS
 * @see THE_CMD_BIT_FAILURE
 * @see THE_CMD_BIT_TIMEOUT
 */
status_t THE_cmd_wait_all(THE_CmdHandle_t* cmdHandles, size_t count, TickType_t deadline,
                          EventBits_t* outResults);

/**
 * @brief Check whether all commands have completed, without waiting.
 *
 * Polls all command handles once and returns immediately.
 *
 * @param[in] cmdHandles Array of command handles to check.
 * @param[in] count Number of handles in the array.
 * @param[out] outResults Optional array to receive result bits (THE_CMD_BIT_SUCCESS,
 *                         THE_CMD_BIT_FAILURE, THE_CMD_BIT_TIMEOUT) for each command.
 *                         Caller must allocate array of size count. Can be NULL.
 *
 * @return kStatus_Success if all commands have THE_CMD_BIT_SUCCESS bit set.
 *         kStatus_Fail if any completed command has THE_CMD_BIT_FAILURE bit set.
 *         kStatus_Timeout if any command is still pending, or any has THE_CMD_BIT_TIMEOUT.
 *
 * @see THE_CMD_BIT_SUCCESS
 * @see THE_CMD_BIT_FAILURE
 * @see THE_CMD_BIT_TIMEOUT
 */
status_t THE_cmd_check_all(THE_CmdHandle_t* cmdHandles, size_t count, EventBits_t* outResults);

/** @} */ // End of TaskHelpers_Module

#endif /* TASK_HELPERS_H_ */
