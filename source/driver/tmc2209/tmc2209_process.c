/************************************************************
 * @file    tmc2209_process.c
 * @brief   Filedescription
 * @author  dg
 * @date    14 Apr 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include <infrastructure/log.h>
#include "tmc2209_process.h"
#include "tmc2209_shared.h"
#include "tmc2209_internal.h"
#include "tmc2209_setup.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "cmd_handle.h"
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

QueueHandle_t tmcCmdQueue = NULL;

CHD_CmdHandleImpl_t tmcCmdHandles[TMC_MAX_CMD_HANDLE_COUNT];

/*****************************
 *     Private Variables     *
 *****************************/

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

void TMCi_process(void)
{
    TMC_CommandQueueItem_t queueItem = {0};
    BaseType_t             status;
    status = xQueueReceive(tmcCmdQueue, &queueItem, pdMS_TO_TICKS(10));
    if (status != pdPASS)
    {
        return;
    }
    status_t cmdStatus = kStatus_Success;
    switch (queueItem.commandType)
    {
        case TMC_CMD_DEFAULT_INIT:
            cmdStatus = TMCi_init_handle(queueItem.data.initHandle.config, queueItem.deadline);
            break;
        case TMC_CMD_SET_MICROSTEPPING:
            cmdStatus = TMCi_set_microstepping(queueItem.handle,
                                               queueItem.data.setMicrostepping.microstepping,
                                               queueItem.deadline);
            break;
        case TMC_CMD_SET_IHOLD_DIVIDER:
            cmdStatus = TMCi_set_ihold_divider(
                queueItem.handle, queueItem.data.setIholdDivider.ihold, queueItem.deadline);
            if (cmdStatus == kStatus_Success)
            {
                queueItem.handle->iholdDivider = queueItem.data.setIholdDivider.ihold;
            }
            break;
        case TMC_CMD_SET_IRUN_DIVIDER:
            cmdStatus = TMCi_set_irun_divider(queueItem.handle, queueItem.data.setIrunDivider.irun,
                                              queueItem.deadline);
            if (cmdStatus == kStatus_Success)
            {
                queueItem.handle->irunDivider = queueItem.data.setIrunDivider.irun;
            }
            break;
        case TMC_CMD_ENABLE_FREEWHEELING:
            cmdStatus = TMCi_set_freewheeling(queueItem.handle, 1, queueItem.deadline);
            if (cmdStatus == kStatus_Success)
            {
                queueItem.handle->freewheeling = 1;
            }
            break;
        case TMC_CMD_DISABLE_FREEWHEELING:
            cmdStatus = TMCi_set_freewheeling(queueItem.handle, 0, queueItem.deadline);
            if (cmdStatus == kStatus_Success)
            {
                queueItem.handle->freewheeling = 0;
            }
            break;
        default:
            cmdStatus = kStatus_Fail;
            break;
    }

    if (queueItem.cmdHandle != NULL)
    {
        if (queueItem.deadline <= xTaskGetTickCount())
        {
            CDP_notify_task_timeout(queueItem.cmdHandle);
        }
        else if (cmdStatus == kStatus_Success)
        {
            CDP_notify_task_success(queueItem.cmdHandle);
        }
        else
        {
            CDP_notify_task_failure(queueItem.cmdHandle);
        }

        CHD_remove_cmd_handle_ref(queueItem.cmdHandle);
    }
}

status_t TMCi_send_cmd_async(TMC_CommandQueueItem_t* queueItem, TickType_t deadline,
                             CHD_CmdHandle_t* cmdHandle)
{
    if (queueItem == NULL)
    {
        return kStatus_Fail;
    }
    CHD_CmdHandle_t internaleCmdHandle = NULL;

    status_t allocStatus =
        CHD_get_cmd_handle(&internaleCmdHandle, tmcCmdHandles, TMC_MAX_CMD_HANDLE_COUNT);
    if (allocStatus != kStatus_Success)
    {
        return kStatus_Fail;
    }
    if (cmdHandle != NULL)
    {
        CHD_add_cmd_handle_ref(internaleCmdHandle);
    }

    queueItem->cmdHandle = internaleCmdHandle;

    status_t sendStatus = CDP_send_cmd(tmcCmdQueue, queueItem, deadline, internaleCmdHandle);
    if (sendStatus != kStatus_Success)
    {
        CHD_remove_cmd_handle_ref(internaleCmdHandle);
        if (cmdHandle != NULL)
        {
            *cmdHandle = NULL;
        }
        return kStatus_Fail;
    }
    if (cmdHandle != NULL)
    {
        *cmdHandle = internaleCmdHandle;
    }
    return kStatus_Success;
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/
