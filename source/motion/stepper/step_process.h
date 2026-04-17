/************************************************************
 * @file    step_process.h
 * @brief   Internal stepper command queue processing helpers.
 * @author  dg
 * @date    13 Apr 2026
 ************************************************************/

/**
 * @defgroup STEPPER_Process_Internal Stepper Process Internal
 * @brief Internal queue send/dequeue and stepper command dispatch helpers.
 * @ingroup STEPPER_Module
 * @{
 */

#ifndef STEP_PROCESS_H_
#define STEP_PROCESS_H_

/********************
 *     Includes    *
 ********************/
#include "step_internal.h"

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
 * @brief Enqueues one stepper command item for asynchronous execution.
 *
 * @param[in,out] queueItem Command payload to enqueue.
 * @param[in] deadline Timeout/deadline while queueing.
 * @param[out] cmdHandle Optional command handle for completion synchronization.
 * @return kStatus_Success if queued, kStatus_Fail otherwise.
 */
status_t STPi_send_cmd_async(STP_CmdQueueItem_t* queueItem, TickType_t deadline,
                             CHD_CmdHandle_t* cmdHandle);

/**
 * @brief Runs one stepper processing cycle.
 */
void STPi_process(void);

/**
 * @brief Dispatches one dequeued stepper command item.
 *
 * @param[in] queueItem Dequeued command item.
 */
void STPi_process_cmd(STP_CmdQueueItem_t queueItem);

#endif // STEP_PROCESS_H_

/** @} */
