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


#ifndef CLI_H_
#define CLI_H_

/********************
 *     Includes		*
 ********************/

#include "FreeRTOS_CLI.h"

/***********************************
 *     Public Macros / Defines	   *
 ***********************************/

/**
 * @brief Maximum length of the input buffer for CLI commands.
 */
#define MAX_INPUT_LENGTH    200

/**
 * @brief Maximum length of the output buffer for CLI responses.
 */
#define MAX_OUTPUT_LENGTH   200

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
 * @return kStatus_Success if the initialization is successful.
 *         kStatus_Fail if the mutex creation or command registration fails.
 */
status_t CLI_Init(void);

/**
 * @brief Registers a new command with the CLI.
 *
 * This function registers a command definition with the FreeRTOS CLI. If the scheduler
 * is running, it ensures thread safety by using a mutex.
 *
 * @param[in] pxCommandToRegister Pointer to the command definition structure to register.
 *
 * @return kStatus_Success if the command is successfully registered.
 *         kStatus_Fail if the registration fails or the mutex cannot be acquired.
 */
status_t CLI_RegisterCommand(
		const CLI_Command_Definition_t *pxCommandToRegister);

/**
 * @brief CLI task function to process CLI commands.
 *
 * This function is intended to be run as a FreeRTOS task. It continuously processes
 * CLI commands and introduces a small delay between iterations to allow other tasks
 * to execute.
 *
 * @param[in] pvParameters Pointer to task parameters (not used in this implementation).
 */
void CLI_Task(void *pvParameters);

#endif /* CLI_H_ */
