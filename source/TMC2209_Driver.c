/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    TMC2209_Driver.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* TODO: insert other include files here. */
#include "TMC2209.h"
#include "CLI.h"
/* TODO: insert other definitions and declarations here. */

/* Task priorities. */

#define my_task_PRIORITY (configMAX_PRIORITIES - 5)

/*******************************************************************************

 * Prototypes

 ******************************************************************************/

static void my_task(void *pvParameters);

/*!

 * @brief Task responsible for printing of "Hello world." message.

 */

static void my_task(void *pvParameters)

{
TMC_Init();
    for (;;)
    {
        uint32_t data = 0b11;
        TMC_write(TMC_SERIAL_ADDRESS_0, TMC_GCONF_ADDR, &data);
        vTaskDelay(100);
        uint8_t transmissionCount;
        TMC_Read_TransmissionCount(TMC_SERIAL_ADDRESS_0, &transmissionCount);
        PRINTF("Hello World!\r\n");
        vTaskDelay(100);
    }

}

static void lpuart0(void *pvParameters)
{
    CLI_Init();
    for (;;)
    {
        CLI_Process();
        vTaskDelay(1);
    }

}

/*
 * @brief   Application entry point.
 */
int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif
    if (xTaskCreate(lpuart0, "lpuart0", configMINIMAL_STACK_SIZE + 100, NULL, my_task_PRIORITY, NULL) != pdPASS)

            {

                PRINTF("Task creation failed!.\r\n");
                while (1)
                    ;

            }
/*    if (xTaskCreate(my_task, "my_task", configMINIMAL_STACK_SIZE + 100, NULL, my_task_PRIORITY, NULL) != pdPASS)

        {

            PRINTF("Task creation failed!.\r\n");
            while (1)
                ;

        }*/


        vTaskStartScheduler();
    PRINTF("Hello World\r\n");

    /* Force the counter to be placed into memory. */
    volatile static int i = 0 ;
    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
        i++ ;
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
        __asm volatile ("nop");
    }
    return 0 ;
}




