/************************************************************
 * @file    application.c
 * @brief   Implementation file for module
 * @author  dg
 * @date    18 Dec 2025
 ************************************************************/

/********************
 *     Includes		*
 ********************/
#include <log.h>
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

/************************************
 *     Private Macros / Defines		*
 ************************************/

/***************************
 *     Private Typedefs	   *
 ***************************/

/*****************************************
 *     Private Function Declarations     *
 *****************************************/

static status_t APP_CreateTask(TaskFunction_t taskFunction, const char *taskName, 
                                 uint16_t stackSize, UBaseType_t priority);

/****************************
 *     Public Variables     *
 ****************************/

/*****************************
 *     Private Variables     *
 *****************************/

/*******************************************
 *     Public Function Implementations     *
 *******************************************/
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

/********************************************
 *     Private Function Implementations     *
 ********************************************/
static status_t APP_CreateTask(TaskFunction_t taskFunction, const char *taskName, 
                                 uint16_t stackSize, UBaseType_t priority)
{
    if (xTaskCreate(taskFunction, taskName, stackSize, NULL, priority, NULL) != pdPASS)
    {
        return kStatus_Fail;
    }
    return kStatus_Success;
}
