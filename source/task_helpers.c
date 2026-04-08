/************************************************************
 * @file    task_helpers.c
 * @brief   FreeRTOS task helper utilities for command execution tracking
 * @details Implements task-safe command handle management and result tracking
 *          using FreeRTOS event groups. Commands are tracked via bitmasks
 *          (THE_CMD_BIT_SUCCESS, THE_CMD_BIT_FAILURE, THE_CMD_BIT_TIMEOUT)
 *          for result status.
 * @author  dgrob
 * @date    18 Mar 2026
 ************************************************************/

/********************
 *     Includes		*
 ********************/
#include "task_helpers.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

/************************************
 *     Private Macros / Defines		*
 ************************************/

/***************************
 *     Private Typedefs	   *
 ***************************/

/*****************************************
 *     Private Function Declarations     *
 *****************************************/

/****************************
 *     Public Variables     *
 ****************************/

/*****************************
 *     Private Variables     *
 *****************************/

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

status_t THE_send_cmd(QueueHandle_t xQueue, const void* const pvItemToQueue, TickType_t deadline,
                      THE_CmdHandle_t cmdHandle)
{
    TickType_t currentTick = xTaskGetTickCount();

    THE_add_cmd_handle_ref(cmdHandle); // Increment reference count for the receiving task

    if (deadline <= currentTick)
    {
        LOG_WARN("Command deadline has already passed");
        THE_notify_task_timeout(cmdHandle);
        THE_remove_cmd_handle_ref(cmdHandle);
        return kStatus_Fail;
    }

    TickType_t ticksUntilDeadline = deadline - currentTick;

    if (xQueueSend(xQueue, pvItemToQueue, ticksUntilDeadline) != pdTRUE)
    {
        LOG_ERROR("Failed to queue command - queue may be full");
        THE_notify_task_timeout(cmdHandle);
        THE_remove_cmd_handle_ref(cmdHandle);
        return kStatus_Fail;
    }
    return kStatus_Success;
}

status_t THE_get_cmd_handle(THE_CmdHandle_t* cmdHandle, THE_CmdHandleImpl_t* cmdHandles,
                            size_t maxHandles)
{
    if (cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    taskENTER_CRITICAL();
    for (size_t i = 0; i < maxHandles; i++)
    {
        if (!cmdHandles[i].used)
        {
            cmdHandles[i].used      = 1;
            cmdHandles[i].ref_count = 0;
            xEventGroupClearBits(cmdHandles[i].eventGroup,
                                 (THE_CMD_BIT_SUCCESS | THE_CMD_BIT_FAILURE | THE_CMD_BIT_TIMEOUT));
            if (cmdHandle != NULL)
            {
                *cmdHandle = &cmdHandles[i];
            }
            taskEXIT_CRITICAL();
            return kStatus_Success;
        }
    }
    *cmdHandle = NULL; // No available command handle
    return kStatus_Fail;
}

void THE_add_cmd_handle_ref(THE_CmdHandle_t cmdHandle)
{
    if (cmdHandle != NULL)
    {
        cmdHandle->ref_count++;
    }
}

void THE_remove_cmd_handle_ref(THE_CmdHandle_t cmdHandle)
{

    if (cmdHandle != NULL)
    {
        taskENTER_CRITICAL();
        if (cmdHandle->ref_count > 0)
        {
            cmdHandle->ref_count--;
        }
        if (cmdHandle->ref_count == 0)
        {
            cmdHandle->used = 0;
        }
        taskEXIT_CRITICAL();
    }
    else
    {
        LOG_DEBUG("Attempted to remove reference from NULL cmd handle");
    }
}

void THE_init_cmd_handles(THE_CmdHandleImpl_t* cmdHandles, size_t maxHandles)
{
    for (size_t i = 0; i < maxHandles; i++)
    {
        cmdHandles[i].used       = 0;
        cmdHandles[i].ref_count  = 0;
        cmdHandles[i].eventGroup = xEventGroupCreate();
    }
}

void THE_notify_task_success(THE_CmdHandle_t cmdHandle)
{
    if (cmdHandle != NULL)
    {
        xEventGroupSetBits(cmdHandle->eventGroup, THE_CMD_BIT_SUCCESS);
    }
}

void THE_notify_task_failure(THE_CmdHandle_t cmdHandle)
{
    if (cmdHandle != NULL)
    {
        xEventGroupSetBits(cmdHandle->eventGroup, THE_CMD_BIT_FAILURE);
    }
}

void THE_notify_task_timeout(THE_CmdHandle_t cmdHandle)
{
    if (cmdHandle != NULL)
    {
        xEventGroupSetBits(cmdHandle->eventGroup, THE_CMD_BIT_TIMEOUT);
    }
}

status_t THE_cmd_wait_result(THE_CmdHandle_t cmdHandle, TickType_t deadline, EventBits_t* outBits)
{
    if (cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    TickType_t currentTick = xTaskGetTickCount();
    if (deadline <= currentTick)
    {
        return kStatus_Timeout;
    }

    TickType_t  ticksUntilDeadline = deadline - currentTick;
    EventBits_t bits               = xEventGroupWaitBits(
        cmdHandle->eventGroup, (THE_CMD_BIT_SUCCESS | THE_CMD_BIT_FAILURE | THE_CMD_BIT_TIMEOUT),
        pdFALSE, pdFALSE, ticksUntilDeadline);

    if (outBits != NULL)
    {
        *outBits = bits;
    }

    if ((bits & THE_CMD_BIT_SUCCESS) != 0u)
    {
        return kStatus_Success;
    }

    if (bits == 0u)
    {
        return kStatus_Timeout;
    }

    return kStatus_Fail;
}

status_t THE_cmd_check_result(THE_CmdHandle_t cmdHandle, EventBits_t* outBits)
{
    if (cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    EventBits_t bits = xEventGroupWaitBits(
        cmdHandle->eventGroup, (THE_CMD_BIT_SUCCESS | THE_CMD_BIT_FAILURE | THE_CMD_BIT_TIMEOUT),
        pdFALSE, pdFALSE, 0);

    if (outBits != NULL)
    {
        *outBits = bits;
    }

    if ((bits & THE_CMD_BIT_SUCCESS) != 0u)
    {
        return kStatus_Success;
    }

    if ((bits & THE_CMD_BIT_FAILURE) != 0u)
    {
        return kStatus_Fail;
    }

    if ((bits & THE_CMD_BIT_TIMEOUT) != 0u)
    {
        return kStatus_Timeout;
    }

    return kStatus_Busy;
}

status_t THE_cmd_wait_any(THE_CmdHandle_t* cmdHandles, size_t count, TickType_t deadline,
                          size_t* outCompletedIndex, EventBits_t* outBits)
{
    if (cmdHandles == NULL || count == 0)
    {
        return kStatus_Fail;
    }

    TickType_t currentTick = xTaskGetTickCount();
    if (deadline <= currentTick)
    {
        return kStatus_Timeout;
    }

    TickType_t totalWaitTicks = deadline - currentTick;

    while (totalWaitTicks > 0)
    {
        for (size_t i = 0; i < count; i++)
        {
            if (cmdHandles[i] == NULL)
            {
                continue;
            }

            EventBits_t bits = xEventGroupWaitBits(
                cmdHandles[i]->eventGroup,
                (THE_CMD_BIT_SUCCESS | THE_CMD_BIT_FAILURE | THE_CMD_BIT_TIMEOUT), pdFALSE, pdFALSE,
                0);

            if (bits != 0)
            {
                if (outCompletedIndex != NULL)
                {
                    *outCompletedIndex = i;
                }
                if (outBits != NULL)
                {
                    *outBits = bits;
                }

                if (bits & THE_CMD_BIT_FAILURE)
                {
                    return kStatus_Fail;
                }
                else if (bits & THE_CMD_BIT_TIMEOUT)
                {
                    return kStatus_Timeout;
                }
                else if (bits & THE_CMD_BIT_SUCCESS)
                {
                    return kStatus_Success;
                }
            }
        }

        currentTick    = xTaskGetTickCount();
        totalWaitTicks = (deadline > currentTick) ? (deadline - currentTick) : 0;
        TickType_t sleepTicks =
            (totalWaitTicks > pdMS_TO_TICKS(1)) ? pdMS_TO_TICKS(1) : totalWaitTicks;

        if (sleepTicks > 0)
        {
            vTaskDelay(sleepTicks);
        }
    }

    return kStatus_Timeout;
}

status_t THE_cmd_check_any(THE_CmdHandle_t* cmdHandles, size_t count, size_t* outCompletedIndex,
                           EventBits_t* outBits)
{
    if (cmdHandles == NULL || count == 0)
    {
        return kStatus_Fail;
    }

    for (size_t i = 0; i < count; i++)
    {
        if (cmdHandles[i] == NULL)
        {
            continue;
        }

        EventBits_t bits = xEventGroupWaitBits(
            cmdHandles[i]->eventGroup,
            (THE_CMD_BIT_SUCCESS | THE_CMD_BIT_FAILURE | THE_CMD_BIT_TIMEOUT), pdFALSE, pdFALSE, 0);

        if (bits != 0u)
        {
            if (outCompletedIndex != NULL)
            {
                *outCompletedIndex = i;
            }
            if (outBits != NULL)
            {
                *outBits = bits;
            }

            if (bits & THE_CMD_BIT_SUCCESS)
            {
                return kStatus_Success;
            }
            if (bits & THE_CMD_BIT_FAILURE)
            {
                return kStatus_Fail;
            }
            if (bits & THE_CMD_BIT_TIMEOUT)
            {
                return kStatus_Timeout;
            }
        }
    }

    return kStatus_Busy;
}

status_t THE_cmd_wait_all(THE_CmdHandle_t* cmdHandles, size_t count, TickType_t deadline,
                          EventBits_t* outResults)
{
    if (cmdHandles == NULL || count == 0)
    {
        return kStatus_Fail;
    }

    TickType_t currentTick = xTaskGetTickCount();
    if (deadline <= currentTick)
    {
        return kStatus_Timeout;
    }

    TickType_t totalWaitTicks = deadline - currentTick;

    while (totalWaitTicks > 0)
    {
        uint8_t allCompleted = 1;
        uint8_t anyFailed    = 0;
        uint8_t anyTimeout   = 0;

        for (size_t i = 0; i < count; i++)
        {
            if (cmdHandles[i] == NULL)
            {
                continue;
            }

            EventBits_t bits = xEventGroupWaitBits(
                cmdHandles[i]->eventGroup,
                (THE_CMD_BIT_SUCCESS | THE_CMD_BIT_FAILURE | THE_CMD_BIT_TIMEOUT), pdFALSE, pdFALSE,
                0);

            if (outResults != NULL)
            {
                outResults[i] = bits;
            }

            if (bits == 0)
            {
                allCompleted = 0; // Not yet completed
            }
            else if ((bits & THE_CMD_BIT_FAILURE) != 0u)
            {
                anyFailed = 1;
            }
            else if ((bits & THE_CMD_BIT_TIMEOUT) != 0u)
            {
                anyTimeout = 1;
            }
        }

        if (anyTimeout)
        {
            return kStatus_Timeout;
        }
        else if (anyFailed)
        {
            return kStatus_Fail;
        }
        else if (allCompleted)
        {
            return kStatus_Success;
        }

        currentTick    = xTaskGetTickCount();
        totalWaitTicks = (deadline > currentTick) ? (deadline - currentTick) : 0;
        TickType_t sleepTicks =
            (totalWaitTicks > pdMS_TO_TICKS(1)) ? pdMS_TO_TICKS(1) : totalWaitTicks;

        if (sleepTicks > 0)
        {
            vTaskDelay(sleepTicks);
        }
    }

    return kStatus_Busy;
}

status_t THE_cmd_check_all(THE_CmdHandle_t* cmdHandles, size_t count, EventBits_t* outResults)
{
    if (cmdHandles == NULL || count == 0)
    {
        return kStatus_Fail;
    }

    uint8_t allCompleted = 1;
    uint8_t anyFailed    = 0;
    uint8_t anyTimeout   = 0;

    for (size_t i = 0; i < count; i++)
    {
        if (cmdHandles[i] == NULL)
        {
            continue;
        }

        EventBits_t bits = xEventGroupWaitBits(
            cmdHandles[i]->eventGroup,
            (THE_CMD_BIT_SUCCESS | THE_CMD_BIT_FAILURE | THE_CMD_BIT_TIMEOUT), pdFALSE, pdFALSE, 0);

        if (outResults != NULL)
        {
            outResults[i] = bits;
        }

        if ((bits & THE_CMD_BIT_SUCCESS) == 0u)
        {
            allCompleted = 0;
        }
        if ((bits & THE_CMD_BIT_FAILURE) != 0u)
        {
            anyFailed = 1;
        }
        if ((bits & THE_CMD_BIT_TIMEOUT) != 0u)
        {
            anyTimeout = 1;
        }
    }

    if (anyFailed)
    {
        return kStatus_Fail;
    }
    if (anyTimeout)
    {
        return kStatus_Timeout;
    }
    if (allCompleted)
    {
        return kStatus_Success;
    }

    return kStatus_Busy;
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/
