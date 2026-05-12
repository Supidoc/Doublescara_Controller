/************************************************************
 * @file    tmc2209_process.h
 * @brief   Internal TMC command queue processing helpers.
 * @author  dg
 * @date    14 Apr 2026
 ************************************************************/

/**
 * @defgroup TMC2209_Process_Internal TMC2209 Process Internal
 * @brief Internal queue send/dequeue and command dispatch helpers.
 * @ingroup TMC2209_Module
 * @{
 */

#ifndef TMC2209_PROCESS_H_
#define TMC2209_PROCESS_H_

#ifdef __cplusplus
extern "C"
{
#endif

/********************
 *     Includes    *
 ********************/
#include "tmc2209_shared.h"
#include "cmd_handle.h"
#include "fsl_common.h"
#include "tmc2209_internal.h"

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
     * @brief Runs one processing cycle for queued TMC command items.
     */
    void TMCi_process(void);

    /**
     * @brief Enqueues one TMC command for asynchronous execution.
     *
     * @param[in,out] queueItem Command payload to enqueue.
     * @param[in] deadline Timeout/deadline while queueing.
     * @param[out] cmdHandle Optional command handle for completion synchronization.
     * @return kStatus_Success if queued, kStatus_Fail otherwise.
     */
    status_t TMCi_send_cmd_async(TMC_CommandQueueItem_t* queueItem, TickType_t deadline,
                                 CHD_CmdHandle_t* cmdHandle);

#ifdef __cplusplus
}
#endif

#endif // TMC2209_PROCESS_H_

/** @} */
