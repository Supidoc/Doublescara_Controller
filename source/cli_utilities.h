/*
 * cli_utilities.h
 *
 *  Created on: 19 Dec 2025
 *      Author: dg
 */

#ifndef CLI_UTILITIES_H_
#define CLI_UTILITIES_H_

#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"
#include "fsl_common.h"

status_t CLU_GetParameterValueString(char * parameterIdentifier, const char * pcCommandString, uint8_t * parameterFound, const char ** pcParameter,
        BaseType_t * parameterStringLength);
#endif /* CLI_UTILITIES_H_ */
