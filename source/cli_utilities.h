/************************************************************
 * @file    cli_utilities.h
 * @brief   Module with utility functions for the CLI
 *
 * This module provides utility functions to support the Command Line Interface (CLI).
 * These functions are designed to simplify the implementation of CLI commands and
 * enhance the functionality of the CLI module.
 *
 * @note    This module is intended to be used alongside the main CLI module.
 *
 * @author  dg
 * @date    18 Dec 2025
 ************************************************************/

/**
 * @defgroup CLI_Utilities_Module CLI Utilities Module
 * @brief   Utility functions to support the Command Line Interface
 * @{
 */

#ifndef CLI_UTILITIES_H_
#define CLI_UTILITIES_H_

/********************
 *     Includes		*
 ********************/

#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"
#include "fsl_common.h"

/***********************************
 *     Public Macros / Defines	   *
 ***********************************/

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
 * @brief Retrieves the value of a specified parameter from a command string.
 *
 * This function searches for a parameter in the given command string that matches the specified
 * parameter identifier. If the parameter is found, its value is returned. If the parameter is not
 * found, the function indicates this and returns a success status.
 *
 * @param[in] parameterIdentifier The identifier of the parameter to search for.
 * @param[in] pcCommandString The command string containing the parameters.
 * @param[out] parameterFound Pointer to a variable that will be set to 1 if the parameter is found,
 * or 0 otherwise.
 * @param[out] pcParameter Pointer to a variable that will point to the first char of the parameter
 * if found.
 * @param[out] parameterStringLength Pointer to a variable that will hold the length of the
 * parameter value string.
 *
 * @return kStatus_Success if the operation completes successfully, even if the parameter is not
 * found. kStatus_Fail if an error occurs during the operation.
 *
 * @note The function uses FreeRTOS_CLIGetParameter to parse the command string.
 *       If a parameter is found without a value, the parameterFound can be treated as a boolean
 * flag indicating the presence of the parameter.
 */
status_t CLU_get_parameter_value_string(char* parameterIdentifier, const char* pcCommandString,
                                        uint8_t* parameterFound, const char** pcParameter,
                                        BaseType_t* parameterStringLength);

/** @} */ // End of CLI_Utilities_Module

#endif /* CLI_UTILITIES_H_ */
