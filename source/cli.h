/************************************************************
 * @file    cli.h
 * @brief   Module for an interactive CLI over UART
 *
 * This module provides an interactive Command Line Interface (CLI) over UART.
 * It allows users to execute commands, retrieve system information, and interact
 * with the application. The CLI is implemented using FreeRTOS+CLI.
 *
 * @note    Ensure the UART peripheral is properly initialized before using this module.
 *
 * @author  dg
 * @date    18 Dec 2025
 ************************************************************/

/**
 * @defgroup CLI_Module Command Line Interface Module
 * @brief   Functions for interactive CLI over UART using FreeRTOS+CLI
 * @{
 */

#ifndef CLI_H_
#define CLI_H_

/********************
 *     Includes		*
 ********************/
#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"
#include "peripherals.h"

/***********************************
 *     Public Macros / Defines	   *
 ***********************************/

/**
 * @brief Maximum length of the input buffer for CLI commands.
 */
#define CLI_MAX_INPUT_LENGTH 200

/**
 * @brief Maximum length of the output buffer for CLI responses.
 */
#define CLI_MAX_OUTPUT_LENGTH 300

#define CLI_SHOW_COMMAND_INPUT 1

/***************************
 *     Public Typedefs	   *
 ***************************/

/****************************
 *     Public Variables     *
 ****************************/

/**************************************
 *     Public Function Prototypes	  *
 **************************************/

/**
 * @brief Initializes the Command Line Interface (CLI) module.
 *
 * This function creates the mutex required for CLI operations and registers the initial
 * set of CLI commands. It must be called before using any other CLI-related functions.
 *
 * @note This function is NOT task-safe and must be called during system initialization.
 * @note The CLI_task() should be started after this function completes successfully.
 * @note Ensure the UART interface for CLI communication is initialized before calling this.
 *
 * @return kStatus_Success if the initialization is successful.
 *         kStatus_Fail if the mutex creation or command registration fails.
 *
 * @see CLI_task()
 * @see CLI_register_command()
 */
status_t CLI_init(void);

/**
 * @brief Registers a new command with the CLI.
 *
 * This function registers a command definition with the FreeRTOS CLI. If the scheduler
 * is running, it ensures thread safety by using a mutex.
 *
 * @param[in] pxCommandToRegister Pointer to the command definition structure to register.
 *
 * @note This function is TASK-SAFE if called after CLI_init().
 * @note Commands can be registered at any time after CLI_init(), including from tasks.
 * @warning All static command names must be unique to avoid registration conflicts.
 *
 * @return kStatus_Success if the command is successfully registered.
 *         kStatus_Fail if the registration fails or the mutex cannot be acquired.
 *
 * @see CLI_init()
 */
status_t CLI_register_command(const CLI_Command_Definition_t* pxCommandToRegister);

/**
 * @brief CLI task function to process CLI commands.
 *
 * This function is intended to be run as a FreeRTOS task. It continuously processes
 * CLI commands and introduces a small delay between iterations to allow other tasks
 * to execute.
 *
 * @param[in] pvParameters Pointer to task parameters (not used in this implementation).
 *
 * @note This task should be started after CLI_init() completes successfully.
 * @note This task waits for input from the CLI UART peripheral.
 * @warning This function does not return; it is expected to run as a perpetual FreeRTOS task.
 *
 * @see CLI_init()
 */
void CLI_task(void* pvParameters);

/** @} */ // End of CLI_Module

#endif /* CLI_H_ */
