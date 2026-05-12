/************************************************************
 * @file    led.h
 * @brief   Implementation of LED control via GPIO PTB18
 * @author  rp
 * @date    12 Apr 2026
 ************************************************************/
#ifndef LED_H_
#define LED_H_

/********************
 *     Includes    *
 ********************/
#include "fsl_common.h"
/************************************
 *    Public Macros / Defines		*
 * ************************************/

/*********************************************
 *     Public Typedefs		 *
 * *********************************************/

/*********************************************
 *    Public Variables		 *
 * *********************************************/

/*********************************************
 *   Public Function Prototypes	 *
 * *********************************************/

status_t LED_init(void);
void LED_on(void);
void LED_off(void);
void LED_toggle(void);


#endif /* LED_H_ */
