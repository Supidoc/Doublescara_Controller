/************************************************************
 * @file    sync_wait.c
 * @brief   Filedescription
 * @author  dg
 * @date    13 Apr 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include "sync_wait.h"

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

status_t SYW_cmd_wait_result(CHD_CmdHandle_t cmdHandle, TickType_t deadline, EventBits_t* outBits)
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
        cmdHandle->eventGroup, (CHD_CMD_BIT_SUCCESS | CHD_CMD_BIT_FAILURE | CHD_CMD_BIT_TIMEOUT),
        pdFALSE, pdFALSE, ticksUntilDeadline);

    if (outBits != NULL)
    {
        *outBits = bits;
    }

    if ((bits & CHD_CMD_BIT_SUCCESS) != 0u)
    {
        return kStatus_Success;
    }

    if (bits == 0u)
    {
        return kStatus_Timeout;
    }

    return kStatus_Fail;
}

status_t SYW_cmd_check_result(CHD_CmdHandle_t cmdHandle, EventBits_t* outBits)
{
    if (cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    EventBits_t bits = xEventGroupWaitBits(
        cmdHandle->eventGroup, (CHD_CMD_BIT_SUCCESS | CHD_CMD_BIT_FAILURE | CHD_CMD_BIT_TIMEOUT),
        pdFALSE, pdFALSE, 0);

    if (outBits != NULL)
    {
        *outBits = bits;
    }

    if ((bits & CHD_CMD_BIT_SUCCESS) != 0u)
    {
        return kStatus_Success;
    }

    if ((bits & CHD_CMD_BIT_FAILURE) != 0u)
    {
        return kStatus_Fail;
    }

    if ((bits & CHD_CMD_BIT_TIMEOUT) != 0u)
    {
        return kStatus_Timeout;
    }

    return kStatus_Busy;
}

status_t SYW_cmd_wait_any(CHD_CmdHandle_t* cmdHandles, size_t count, TickType_t deadline,
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
                (CHD_CMD_BIT_SUCCESS | CHD_CMD_BIT_FAILURE | CHD_CMD_BIT_TIMEOUT), pdFALSE, pdFALSE,
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

                if (bits & CHD_CMD_BIT_FAILURE)
                {
                    return kStatus_Fail;
                }
                else if (bits & CHD_CMD_BIT_TIMEOUT)
                {
                    return kStatus_Timeout;
                }
                else if (bits & CHD_CMD_BIT_SUCCESS)
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

status_t SYW_cmd_check_any(CHD_CmdHandle_t* cmdHandles, size_t count, size_t* outCompletedIndex,
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
            (CHD_CMD_BIT_SUCCESS | CHD_CMD_BIT_FAILURE | CHD_CMD_BIT_TIMEOUT), pdFALSE, pdFALSE, 0);

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

            if (bits & CHD_CMD_BIT_SUCCESS)
            {
                return kStatus_Success;
            }
            if (bits & CHD_CMD_BIT_FAILURE)
            {
                return kStatus_Fail;
            }
            if (bits & CHD_CMD_BIT_TIMEOUT)
            {
                return kStatus_Timeout;
            }
        }
    }

    return kStatus_Busy;
}

status_t SYW_cmd_wait_all(CHD_CmdHandle_t* cmdHandles, size_t count, TickType_t deadline,
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
                (CHD_CMD_BIT_SUCCESS | CHD_CMD_BIT_FAILURE | CHD_CMD_BIT_TIMEOUT), pdFALSE, pdFALSE,
                0);

            if (outResults != NULL)
            {
                outResults[i] = bits;
            }

            if (bits == 0)
            {
                allCompleted = 0; // Not yet completed
            }
            else if ((bits & CHD_CMD_BIT_FAILURE) != 0u)
            {
                anyFailed = 1;
            }
            else if ((bits & CHD_CMD_BIT_TIMEOUT) != 0u)
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

status_t SYW_cmd_check_all(CHD_CmdHandle_t* cmdHandles, size_t count, EventBits_t* outResults)
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
            (CHD_CMD_BIT_SUCCESS | CHD_CMD_BIT_FAILURE | CHD_CMD_BIT_TIMEOUT), pdFALSE, pdFALSE, 0);

        if (outResults != NULL)
        {
            outResults[i] = bits;
        }

        if ((bits & CHD_CMD_BIT_SUCCESS) == 0u)
        {
            allCompleted = 0;
        }
        if ((bits & CHD_CMD_BIT_FAILURE) != 0u)
        {
            anyFailed = 1;
        }
        if ((bits & CHD_CMD_BIT_TIMEOUT) != 0u)
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
