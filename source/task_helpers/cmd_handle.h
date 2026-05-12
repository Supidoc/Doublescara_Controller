/************************************************************
 * @file    cmd_handle.h
 * @brief   Command handle pool and synchronization helpers.
 * @author  dg
 * @date    13 Apr 2026
 ************************************************************/

#ifndef CMD_HANDLE_H_
#define CMD_HANDLE_H_

#ifdef __cplusplus
extern "C"
{
#endif

/********************
 *     Includes    *
 ********************/
#include <stdint.h>
#include "FreeRTOS.h"
#include "event_groups.h"
#include "fsl_common.h"
#include "stdlib.h"

/***********************************
 *     Public Macros / Defines     *
 ***********************************/

/**
 * @defgroup THE_EventBits Task Helper Event Bits
 * @brief Bitmasks for command completion status tracking
 * @{
 */

/** @brief Command completed successfully */
#define CHD_CMD_BIT_SUCCESS (1 << 0)

/** @brief Command failed */
#define CHD_CMD_BIT_FAILURE (1 << 1)

/** @brief Command timed out */
#define CHD_CMD_BIT_TIMEOUT (1 << 2)

    /** @} */

    /***************************
     *     Public Typedefs     *
     ***************************/

    typedef struct _CHD_CmdHandleImpl
    {
        EventGroupHandle_t eventGroup;
        uint8_t            used;
        uint8_t            ref_count;
    } CHD_CmdHandleImpl_t;

    typedef CHD_CmdHandleImpl_t* CHD_CmdHandle_t;

    /****************************
     *     Public Variables     *
     ****************************/

    /**************************************
     *     Public Function Prototypes    *
     **************************************/

    /**
     * @brief Initialize the command handle pool
     * @details Sets up the command handle infrastructure for tracking asynchronous
     *          command execution. Must be called once before using command handles.
     * @param[in] cmdHandles Pointer to array of command handle structures
     * @param[in] maxHandles Maximum number of handles in the pool
     */
    void CHD_init_cmd_handles(CHD_CmdHandleImpl_t* cmdHandles, size_t maxHandles);

    /**
     * @brief Get an available command handle from the pool
     * @param[out] cmdHandle Pointer to receive the allocated command handle
     * @param[in] cmdHandles Pointer to array of command handle structures
     * @param[in] maxHandles Maximum number of handles in the pool
     * @return kStatus_Success if a handle was allocated
     *         kStatus_Fail if no handles are available
     */
    status_t CHD_get_cmd_handle(CHD_CmdHandle_t* cmdHandle, CHD_CmdHandleImpl_t* cmdHandles,
                                size_t maxHandles);

    /**
     * @brief Increment reference count for a command handle
     * @details Used to track multiple references to the same command handle
     * @param[in] cmdHandle Command handle to increment reference for
     */
    void CHD_add_cmd_handle_ref(CHD_CmdHandle_t cmdHandle);

    /**
     * @brief Decrement reference count for a command handle
     * @details When reference count reaches zero, the handle is returned to the pool
     * @param[in] cmdHandle Command handle to decrement reference for
     */
    void CHD_remove_cmd_handle_ref(CHD_CmdHandle_t cmdHandle);
    void CHD_remove_cmd_handle_ref_from_isr(CHD_CmdHandle_t cmdHandle);

#ifdef __cplusplus
}
#endif

#endif // CMD_HANDLE_H_
