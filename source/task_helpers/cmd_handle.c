/************************************************************
 * @file    cmd_handle.c
 * @brief   Filedescription
 * @author  dg
 * @date    13 Apr 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include "cmd_handle.h"
#include "log.h"
#include "string.h"

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

void CHD_init_cmd_handles(CHD_CmdHandleImpl_t* cmdHandles, size_t maxHandles)
{
    for (size_t i = 0; i < maxHandles; i++)
    {
        cmdHandles[i].used       = 0;
        cmdHandles[i].ref_count  = 0;
        cmdHandles[i].eventGroup = xEventGroupCreate();
    }
}

status_t CHD_get_cmd_handle(CHD_CmdHandle_t* cmdHandle, CHD_CmdHandleImpl_t* cmdHandles,
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
                                 (CHD_CMD_BIT_SUCCESS | CHD_CMD_BIT_FAILURE | CHD_CMD_BIT_TIMEOUT));
            if (cmdHandle != NULL)
            {
                *cmdHandle = &cmdHandles[i];
            }
            taskEXIT_CRITICAL();
            return kStatus_Success;
        }
    }
    taskEXIT_CRITICAL();
    *cmdHandle = NULL; // No available command handle
    return kStatus_Fail;
}

void CHD_add_cmd_handle_ref(CHD_CmdHandle_t cmdHandle)
{
    if (cmdHandle != NULL)
    {
        cmdHandle->ref_count++;
    }
}

void CHD_remove_cmd_handle_ref(CHD_CmdHandle_t cmdHandle)
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

/********************************************
 *     Private Function Implementations     *
 ********************************************/
