/************************************************************
 * @file    cmd_dispatch.h
 * @brief   Command queue dispatch and completion notification helpers.
 * @author  dg
 * @date    13 Apr 2026
 ************************************************************/

#ifndef CMD_DISPATCH_H_
#define CMD_DISPATCH_H_

#ifdef __cplusplus
extern "C"
{
#endif

/********************
 *     Includes    *
 ********************/
#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"
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

    /**
     * @brief Send a command to a queue with deadline tracking
     * @param[in] xQueue FreeRTOS queue to send the command to
     * @param[in] pvItemToQueue Pointer to the item to send
     * @param[in] deadline Deadline for command completion (in ticks)
     * @param[in] cmdHandle Command handle for tracking this operation
     * @return kStatus_Success if command was queued successfully
     *         kStatus_Fail if queue is full or deadline is invalid
     */
    status_t CDP_send_cmd(QueueHandle_t xQueue, const void* const pvItemToQueue,
                          TickType_t deadline, CHD_CmdHandle_t cmdHandle);

    /**
     * @brief Notify successful completion of a command
     * @details Sets the CHD_CMD_BIT_SUCCESS bit in the command handle's event group
     * @param[in] cmdHandle Command handle to notify
     */
    void CDP_notify_task_success(CHD_CmdHandle_t cmdHandle);

    /**
     * @brief Notify failure of a command
     * @details Sets the CHD_CMD_BIT_FAILURE bit in the command handle's event group
     * @param[in] cmdHandle Command handle to notify
     */
    void CDP_notify_task_failure(CHD_CmdHandle_t cmdHandle);
    void CDP_notify_task_failure_from_isr(CHD_CmdHandle_t cmdHandle);

    /**
     * @brief Notify timeout of a command
     * @details Sets the CHD_CMD_BIT_TIMEOUT bit in the command handle's event group
     * @param[in] cmdHandle Command handle to notify
     */
    void CDP_notify_task_timeout(CHD_CmdHandle_t cmdHandle);

#ifdef __cplusplus
}
#endif

#endif // CMD_DISPATCH_H_
