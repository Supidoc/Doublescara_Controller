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


void APP_Init(void){
        DISK_Init();
        CLI_Init();
        LOG_Init();
}

void APP_Run(void){
    if (xTaskCreate(LOG_Task, "LOG_Task", configMINIMAL_STACK_SIZE + 100, NULL, configMAX_PRIORITIES - 5, NULL) != pdPASS)

    {
        while (1)
            ;
    }
    if (xTaskCreate(CLI_Task, "CLI_Task", configMINIMAL_STACK_SIZE + 100, NULL, configMAX_PRIORITIES - 5, NULL) != pdPASS)

    {
        while (1)
            ;
    }
    vTaskStartScheduler();

}
