/************************************************************
 * @file    scara_kinematics_process.h
 * @brief   Internal kinematics command processing and async helpers.
 * @author  dg
 * @date    17 Apr 2026
 ************************************************************/

#ifndef MOTION_SCARA_KINEMATICS_SCARA_KINEMATICS_PROCESS_H_
#define MOTION_SCARA_KINEMATICS_SCARA_KINEMATICS_PROCESS_H_

#ifdef __cplusplus
extern "C"
{
#endif

/********************
 *     Includes    *
 ********************/
#include <motion/scara_kinematics/scara_kinematics_internal.h>

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
     * @brief Enqueues one kinematics command for asynchronous execution.
     *
     * @param[in,out] queueItem Command payload to enqueue.
     * @param[in] deadline Timeout/deadline while queueing.
     * @param[out] cmdHandle Optional command handle for completion synchronization.
     * @return kStatus_Success if queued, kStatus_Fail otherwise.
     */
    status_t SKi_send_cmd_async(SK_CmdQueueItem_t* queueItem, TickType_t deadline,
                                CHD_CmdHandle_t* cmdHandle);

    /**
     * @brief Initializes the kinematics command queue and resources.
     *
     * @return kStatus_Success if initialization succeeds.
     */
    status_t SKi_init_queue(void);

    /**
     * @brief Processes one dequeued kinematics command.
     *
     * @param[in] queueItem Dequeued command item.
     */
    void SKi_process_cmd(SK_CmdQueueItem_t queueItem);

    /**
     * @brief Task function for processing kinematics commands.
     *
     * @param[in] pvParameters Pointer to task parameters (not used).
     */
    void SKi_task(void* pvParameters);

#ifdef __cplusplus
}
#endif

#endif // MOTION_SCARA_KINEMATICS_SCARA_KINEMATICS_PROCESS_H_
