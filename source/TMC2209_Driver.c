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

/* TODO: insert other include files here. */
#include "application.h"

/* TODO: insert other definitions and declarations here. */

/* Task priorities. */

#define my_task_PRIORITY (configMAX_PRIORITIES - 5)

/*******************************************************************************

 * Prototypes

 ******************************************************************************/

static void my_task(void * pvParameters);

/*!

 * @brief Task responsible for printing of "Hello world." message.

 */

/*static void TMC_Task(void * pvParameters)

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

}*/

/*static void FS_Task(void * pvParameters)
{




    for (;;)
    {
        vTaskDelay(1);
    }

}*/

/*
 * @brief   Application entry point.
 */
int main(void)
{

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    /*if (xTaskCreate(FS_Task, "FS_Task", configMINIMAL_STACK_SIZE + 100, NULL, my_task_PRIORITY, NULL) != pdPASS)

    {

        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;

    }
    if (xTaskCreate(TMC_Task, "TMC_Task", configMINIMAL_STACK_SIZE + 100, NULL, my_task_PRIORITY, NULL) != pdPASS)

    {

        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;

    }*/

    APP_Init();

    APP_Run();

    /* Force the counter to be placed into memory. */
    volatile static int i = 0;
    /* Enter an infinite loop, just incrementing a counter. */
    while (1)
    {
        i++;
        /* 'Dummy' NOP to allow source level single stepping of
         tight while() loop */
        __asm volatile ("nop");
    }
    return 0;
}

