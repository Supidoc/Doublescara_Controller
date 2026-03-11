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

/****************************
 *     Public Variables     *
 ****************************/

/**************************************
 *     Public Function Prototypes    *
 **************************************/

/**
 * @brief Notify a task with operation result
 *
 * Helper function to notify a task handle with success (1) or failure (0).
 * Safe to call with NULL task handle.
 *
 * @param taskHandle Task to notify (NULL safe)
 * @param result Operation result (kStatus_Success or kStatus_Fail)
 */
static inline void THE_notify_task_result(TaskHandle_t taskHandle, status_t result)
{
    if (taskHandle != NULL)
    {
        xTaskNotify(taskHandle, result == kStatus_Success ? 1 : 0, eSetValueWithOverwrite);
    }
}

/**
 * @brief Notify task on success, don't notify on failure
 *
 * Helper function to notify a task handle with success.
 * Safe to call with NULL task handle.
 *
 * @param taskHandle Task to notify (NULL safe)
 */
static inline void THE_notify_task_success(TaskHandle_t taskHandle)
{
    if (taskHandle != NULL)
    {
        xTaskNotify(taskHandle, 1, eSetValueWithOverwrite);
    }
}

/**
 * @brief Notify task of failure
 *
 * Helper function to notify a task handle with failure.
 * Safe to call with NULL task handle.
 *
 * @param taskHandle Task to notify (NULL safe)
 */
static inline void THE_notify_task_failure(TaskHandle_t taskHandle)
{
    if (taskHandle != NULL)
    {
        xTaskNotify(taskHandle, 0, eSetValueWithOverwrite);
    }
}

/** @brief Convert timeout in milliseconds to a deadline
 *
 * @param timeout_ms Timeout in milliseconds
 * @return Deadline in ticks
 */
static inline TickType_t THE_deadline_from_timeout_ms(uint32_t timeout_ms)
{

    return xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
}

/** @} */ // End of TaskHelpers_Module

#endif /* TASK_HELPERS_H_ */
