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
 * @brief Begin a task-safe operation by capturing the caller's task handle
 *
 * Usage: Place at the beginning of a public task-safe function after parameter validation.
 * Creates a local variable 'callerTaskHandle' that can be used in queue items.
 */
#define THE_TASK_SAFE_BEGIN()                                                                      \
    TaskHandle_t callerTaskHandle = xTaskGetCurrentTaskHandle();                                   \
    if (callerTaskHandle == NULL)                                                                  \
    {                                                                                              \
        LOG_WARN("Failed to get current task handle");                                             \
        return kStatus_Fail;                                                                       \
    }

/**
 * @brief Wait for notification from async operation with a deadline
 *
 * @param deadline Deadline for the operation
 *
 * Usage: Place after queueing the command. Waits for notification and
 * returns kStatus_Fail after deadline passed without receving notifications.
 *
 * @note If deadline is 0, the function will not wait for completion and will return immediately.
 */
#define THE_TASK_SAFE_WAIT(deadline)                                                               \
    do                                                                                             \
    {                                                                                              \
        if (deadline != 0)                                                                         \
        {                                                                                          \
            TickType_t currentTick = xTaskGetTickCount();                                          \
            if (deadline <= currentTick)                                                           \
            {                                                                                      \
                char logMsg[60];                                                                   \
                snprintf(logMsg, sizeof(logMsg),                                                   \
                         "Deadline for task notification has already passed");                     \
                LOG_ERROR(logMsg);                                                                 \
                return kStatus_Fail;                                                               \
            }                                                                                      \
            TickType_t ticksUntilDeadline = deadline - currentTick;                                \
            if (ulTaskNotifyTake(pdTRUE, ticksUntilDeadline) == 0)                                 \
            {                                                                                      \
                char logMsg[60];                                                                   \
                snprintf(logMsg, sizeof(logMsg), "Timeout waiting for notification after %u ms",   \
                         (unsigned int)(pdTICKS_TO_MS(ticksUntilDeadline)));                       \
                LOG_ERROR(logMsg);                                                                 \
                return kStatus_Fail;                                                               \
            }                                                                                      \
        }                                                                                          \
    } while (0)

/**
 * @brief Complete task-safe operation pattern: wait and return
 *
 * Combines notification wait with success return.
 */
#define THE_TASK_SAFE_COMPLETE(deadline)                                                           \
    THE_TASK_SAFE_WAIT(deadline)                                                                   \
    return kStatus_Success;

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

void THE_init_cmd_handles(THE_CmdHandleImpl_t* cmdHandles, size_t maxHandles);

void THE_notify_task_success(THE_CmdHandle_t cmdHandle);

void THE_notify_task_failure(THE_CmdHandle_t cmdHandle);

void THE_notify_task_timeout(THE_CmdHandle_t cmdHandle);

status_t THE_get_cmd_handle(THE_CmdHandle_t* cmdHandle, THE_CmdHandleImpl_t* cmdHandles,
                            size_t maxHandles);

void THE_add_cmd_handle_ref(THE_CmdHandle_t cmdHandle);

void THE_remove_cmd_handle_ref(THE_CmdHandle_t cmdHandle);

status_t THE_send_cmd(QueueHandle_t xQueue, const void* const pvItemToQueue, TickType_t deadline,
                      THE_CmdHandle_t cmdHandle);

/**
 * @brief Wait for a single command to complete.
 *
 * Blocks until the command completes (success/failure/timeout) or the deadline is reached.
 *
 * @param[in] cmdHandle Command handle to wait for.
 * @param[in] deadline Deadline for waiting (in ticks).
 * @param[out] outBits Optional pointer to receive result bits (THE_CMD_BIT_SUCCESS, etc). Can be
 * NULL.
 *
 * @return kStatus_Success if command completed with success bit set.
 *         kStatus_Fail if command failed, timeout, or deadline exceeded.
 */
status_t THE_cmd_wait_result(THE_CmdHandle_t cmdHandle, TickType_t deadline, EventBits_t* outBits);

/**
 * @brief Check a single command result without waiting.
 *
 * Polls the command result bits once and returns immediately.
 *
 * @param[in] cmdHandle Command handle to check.
 * @param[out] outBits Optional pointer to receive result bits. Can be NULL.
 *
 * @return kStatus_Success if command completed successfully.
 *         kStatus_Fail if command completed with failure.
 *         kStatus_Timeout if command timed out or is not completed yet.
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
 * @param[out] outCompletedIndex Optional pointer to receive index of first completed command. Can
 * be NULL.
 * @param[out] outBits Optional pointer to receive result bits of completed command. Can be NULL.
 *
 * @return kStatus_Success if any command completed successfully.
 *         kStatus_Fail if no commands completed, all failed, or deadline exceeded.
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
 * @param[out] outCompletedIndex Optional pointer to receive index of first completed command. Can
 * be NULL.
 * @param[out] outBits Optional pointer to receive result bits of completed command. Can be NULL.
 *
 * @return kStatus_Success if any command completed successfully.
 *         kStatus_Fail if any command completed with failure.
 *         kStatus_Timeout if none are completed yet, or first completed command timed out.
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
 * @param[out] outResults Optional array to receive result bits for each command. Can be NULL.
 *
 * @return kStatus_Success if all commands completed successfully.
 *         kStatus_Fail if any command failed, timed out, or deadline exceeded.
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
 * @param[out] outResults Optional array to receive result bits for each command. Can be NULL.
 *
 * @return kStatus_Success if all commands completed successfully.
 *         kStatus_Fail if all commands completed and any command failed.
 *         kStatus_Timeout if any command is still pending, or all completed with timeout only.
 */
status_t THE_cmd_check_all(THE_CmdHandle_t* cmdHandles, size_t count, EventBits_t* outResults);

/** @} */ // End of TaskHelpers_Module

#endif /* TASK_HELPERS_H_ */
