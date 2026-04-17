/************************************************************
 * @file    motor_process.h
 * @brief   Internal motor command-queue processing utilities.
 * @author  dg
 * @date    15 Apr 2026
 ************************************************************/

/**
 * @defgroup MOTOR_Process_Internal Motor Process Internal
 * @brief Internal queue send/dequeue and command dispatch logic.
 * @ingroup MOTOR_Facade_Module
 * @{
 */

#ifndef MOTOR_PROCESS_H_
#define MOTOR_PROCESS_H_

/********************
 *     Includes    *
 ********************/
#include "motor_internal.h"

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
 * @brief Enqueues a motor command for asynchronous execution.
 *
 * @param[in,out] queueItem Command payload to send to motor command queue.
 * @param[in] deadline Timeout/deadline applied while queueing the command.
 * @param[out] cmdHandle Optional command handle for completion synchronization.
 * @return kStatus_Success if queued successfully, kStatus_Fail otherwise.
 */
status_t MTRi_send_cmd_async(MTR_CmdQueueItem_t* queueItem, TickType_t deadline,
                             CHD_CmdHandle_t* cmdHandle);

/**
 * @brief Runs one motor command processing cycle.
 *
 * Dequeues at most one command and advances active parallel command tracking.
 */
void MTRi_process(void);

/**
 * @brief Dispatches one command item to the corresponding internal handler.
 *
 * @param[in] queueItem Command item to process.
 * @param[in,out] taskItem Parallel task bookkeeping entry for async sub-commands.
 * @return kStatus_Success, kStatus_Fail, or kStatus_Timeout depending on command result.
 */
status_t MTRi_process_cmd(MTR_CmdQueueItem_t queueItem, MTR_ParallelTaskItem* taskItem);

#endif // MOTOR_PROCESS_H_

/** @} */