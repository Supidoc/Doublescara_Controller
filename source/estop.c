/************************************************************
 * @file    estop.c
 * @brief   Filedescription
 * @author  dg
 * @date    15 May 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include "estop.h"
#include "fsl_gpio.h"
#include "init.h"
#include "fsl_port.h"
#include "peripherals.h"

/************************************
 *     Private Macros / Defines    *
 ************************************/

/***************************
 *     Private Typedefs     *
 ***************************/

/*****************************************
 *     Private Function Declarations     *
 *****************************************/

/****************************
 *     Public Variables     *
 ****************************/

bool estopFlag = false;

bool estopPressed = false;

/*****************************
 *     Private Variables     *
 *****************************/

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

void ESTOP_SetEstop(void)
{
    estopFlag = true;
    INIT_SetUnitialized();
}

void ESTOP_ClearEstop(void)
{
    estopFlag = false;
    estopPressed = ((GPIOC->PDIR & 0b1 << 8) != 0);
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/

/* PORTC_IRQn interrupt handler */
void GPIOC_IRQHANDLER(void)
{
    /* Get pin flags */
    uint32_t pin_flags = GPIO_PortGetInterruptFlags(GPIOC);


    /* Place your interrupt code here */

    /* Clear pin flags */


    if ((GPIOC->PDIR & 0b1 << 8) != 0)
    {
    	estopPressed = true;

        ESTOP_SetEstop();
    }
    else{
    	estopPressed = false;

    }

    if(estopPressed){
        PORT_SetPinInterruptConfig(PORTC, 8, kPORT_InterruptLogicZero);
    }
    else{
        PORT_SetPinInterruptConfig(PORTC, 8, kPORT_InterruptLogicOne);
    }
    GPIO_PortClearInterruptFlags(GPIOC, pin_flags);


/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
   Store immediate overlapping exception return operation might vector to incorrect interrupt. */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
