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
#include "application.h"
#include "board.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "peripherals.h"
#include "pin_mux.h"
#include <stdio.h>

/*******************************************************************************

 * Prototypes

 ******************************************************************************/

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

    APP_init();

    APP_run();

    /* Force the counter to be placed into memory. */
    volatile static int i = 0;
    /* Enter an infinite loop, just incrementing a counter. */
    while (1)
    {
        i++;
        /* 'Dummy' NOP to allow source level single stepping of
         tight while() loop */
        __asm volatile("nop");
    }
    return 0;
}
