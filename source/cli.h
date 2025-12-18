/*
 * CLI.h
 *
 *  Created on: 15 Dec 2025
 *      Author: dg
 */

#ifndef CLI_H_
#define CLI_H_

#include "FreeRTOS_CLI.h"

#define MAX_INPUT_LENGTH    50
#define MAX_OUTPUT_LENGTH   100

status_t CLI_Init(void);
status_t CLI_RegisterCommand(const CLI_Command_Definition_t  * pxCommandToRegister );
void CLI_Task(void * pvParameters);

#endif /* CLI_H_ */
