/*
 * application.c
 *
 *  Created on: 18 Dec 2025
 *      Author: dg
 */

#include "application.h"

#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "disk.h"
#include "cli.h"
#include "log.h"

/* Private function prototypes */
static status_t APP_CreateTask(TaskFunction_t taskFunction, const char *taskName, 
                                 uint16_t stackSize, UBaseType_t priority);

/* Public API implementation */

void APP_Init(void)
{
    DISK_Init();
    CLI_Init();
    LOG_Init();
}

void APP_Run(void)
{
    if (APP_CreateTask(LOG_Task, "LOG_Task", configMINIMAL_STACK_SIZE + 100, 
                       configMAX_PRIORITIES - 5) != kStatus_Success)
    {
        while (1);
    }
    
    if (APP_CreateTask(CLI_Task, "CLI_Task", configMINIMAL_STACK_SIZE + 100, 
                       configMAX_PRIORITIES - 5) != kStatus_Success)
    {
        while (1);
    }
    
    vTaskStartScheduler();
}

/* Private helper functions */

static status_t APP_CreateTask(TaskFunction_t taskFunction, const char *taskName, 
                                 uint16_t stackSize, UBaseType_t priority)
{
    if (xTaskCreate(taskFunction, taskName, stackSize, NULL, priority, NULL) != pdPASS)
    {
        return kStatus_Fail;
    }
    return kStatus_Success;
}
