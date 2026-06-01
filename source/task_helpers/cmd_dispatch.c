/************************************************************
 * @file    cmd_dispatch.c
 * @brief   Filedescription
 * @author  dg
 * @date    13 Apr 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include <infrastructure/log.h>
#include "cmd_dispatch.h"

/************************************
 *     Private Macros / Defines    *
 ************************************/

/***************************
 *     Private Typedefs     *
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

status_t CDP_send_cmd(QueueHandle_t xQueue, const void* const pvItemToQueue, TickType_t deadline,
                      CHD_CmdHandle_t cmdHandle)
{
    TickType_t currentTick = xTaskGetTickCount();

    CHD_add_cmd_handle_ref(cmdHandle); // Increment reference count for the receiving task

    if (deadline <= currentTick)
    {
        LOG_WARN("Command deadline has already passed");
        CDP_notify_task_timeout(cmdHandle);
        CHD_remove_cmd_handle_ref(cmdHandle);
        return kStatus_Fail;
    }

    TickType_t ticksUntilDeadline = deadline - currentTick;

    if (xQueueSend(xQueue, pvItemToQueue, ticksUntilDeadline) != pdTRUE)
    {
        LOG_ERROR("Failed to queue command - queue may be full");
        CDP_notify_task_timeout(cmdHandle);
        CHD_remove_cmd_handle_ref(cmdHandle);
        return kStatus_Fail;
    }
    return kStatus_Success;
}

void CDP_notify_task_success(CHD_CmdHandle_t cmdHandle)
{
    if (cmdHandle != NULL)
    {
        xEventGroupSetBits(cmdHandle->eventGroup, CHD_CMD_BIT_SUCCESS);
    }
}

void CDP_notify_task_failure(CHD_CmdHandle_t cmdHandle)
{
    if (cmdHandle != NULL)
    {
        xEventGroupSetBits(cmdHandle->eventGroup, CHD_CMD_BIT_FAILURE);
    }
}

void CDP_notify_task_failure_from_isr(CHD_CmdHandle_t cmdHandle)
{
    if (cmdHandle != NULL)
    {
        BaseType_t pxHigherPriorityTaskWoken;
        xEventGroupSetBitsFromISR(cmdHandle->eventGroup, CHD_CMD_BIT_FAILURE,
                                  &pxHigherPriorityTaskWoken);
        if (pxHigherPriorityTaskWoken)
        {
            taskYIELD();
        }
    }
}

void CDP_notify_task_timeout(CHD_CmdHandle_t cmdHandle)
{
    if (cmdHandle != NULL)
    {
        xEventGroupSetBits(cmdHandle->eventGroup, CHD_CMD_BIT_TIMEOUT);
    }
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/
