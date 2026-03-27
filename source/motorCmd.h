/************************************************************
 * @file    motorCmd.h
 * @brief   Filedescription
 * @author  dg
 * @date    11 Mar 2026
 ************************************************************/

#ifndef MOTORCMD_H_
#define MOTORCMD_H_

/********************
 *     Includes    *
 ********************/

#include "fsl_common.h"

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
 * @brief Initializes the motor command module
 *
 * This function initializes the motor command module, setting up any necessary data structures or
 * state. It should be called once during system initialization before starting the MCMD task.
 *
 * @return kStatus_Success if initialization is successful, or an appropriate error code if it
 * fails.
 *
 * @note This function is NOT task-safe and should only be called from the main initialization code
 * before any tasks are started.
 * @note The MCMD_task() should be started after this function completes successfully.
 */
status_t MCMD_init(void);

/**
 * @brief Task function for processing motor control commands
 *
 * This function is intended to be run as a FreeRTOS task. It registers motor control commands with
 * the CLI and processes any incoming commands related to motor control. The task will run
 * indefinitely, handling motor command requests as they come in.
 *
 * @param[in] pvParameters Pointer to task parameters (not used in this implementation).
 *
 * @note This function should be started after MCMD_init() completes successfully.
 * @note This task is blocked on CLI command processing and will wake when motor control commands
 * are received.
 * @warning This function does not return; it is expected to run as a perpetual FreeRTOS task.
 */
void MCMD_task(void* pvParameters);

#endif /* MOTORCMD_H_ */
