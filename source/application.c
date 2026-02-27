/************************************************************
 * @file    application.c
 * @brief   Implementation file for application module
 * @author  dg
 * @date    18 Dec 2025
 ************************************************************/

/********************
 *     Includes		*
 ********************/
#include "application.h"
#include <log.h>

#include "board.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "peripherals.h"
#include "pin_mux.h"
#include <stdio.h>

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "cli.h"
#include "disk.h"
#include "motor_test.h"
#include "step.h"
#include "tmc2209.h"

/************************************
 *     Private Macros / Defines		*
 ************************************/

/***************************
 *     Private Typedefs	   *
 ***************************/

/*****************************************
 *     Private Function Declarations     *
 *****************************************/

static status_t create_task(TaskFunction_t taskFunction, const char* taskName, uint16_t stackSize,
                            UBaseType_t priority);

/****************************
 *     Public Variables     *
 ****************************/

/*****************************
 *     Private Variables     *
 *****************************/

/*******************************************
 *     Public Function Implementations     *
 *******************************************/
void APP_init(void)
{
    DISK_init();
    CLI_init();
    LOG_init();
    STP_init();
    TMC_init();
    MTT_init();
}

void APP_run(void)
{
    if (create_task(LOG_task, "LOG_Task", configMINIMAL_STACK_SIZE + 200,
                    configMAX_PRIORITIES - 5) != kStatus_Success)
    {
        while (1)
            ;
    }

    if (create_task(CLI_task, "CLI_Task", configMINIMAL_STACK_SIZE + 200,
                    configMAX_PRIORITIES - 5) != kStatus_Success)
    {
        while (1)
            ;
    }
    if (create_task(STP_task, "STP_Task", configMINIMAL_STACK_SIZE + 200,
                    configMAX_PRIORITIES - 5) != kStatus_Success)
    {
        while (1)
            ;
    }
    if (create_task(TMC_task, "TMC_Task", configMINIMAL_STACK_SIZE + 200,
                    configMAX_PRIORITIES - 5) != kStatus_Success)
    {
        while (1)
            ;
    }
    if (create_task(MTT_task, "MTT_Task", configMINIMAL_STACK_SIZE + 200,
                    configMAX_PRIORITIES - 5) != kStatus_Success)
    {
        while (1)
            ;
    }
    vTaskStartScheduler();
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/
static status_t create_task(TaskFunction_t taskFunction, const char* taskName, uint16_t stackSize,
                            UBaseType_t priority)
{
    if (xTaskCreate(taskFunction, taskName, stackSize, NULL, priority, NULL) != pdPASS)
    {
        return kStatus_Fail;
    }
    return kStatus_Success;
}
