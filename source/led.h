/************************************************************
 * @file    led.h
 * @brief   Simple LED control interface
 * @details Provides a minimal abstraction for controlling a status LED
 *          connected to a GPIO (PTB18 on the target board). The
 *          implementation hides the hardware details and returns
 *          standard `status_t` codes from the SDK.
 * @author  rp
 * @date    12 Apr 2026
 ************************************************************/

/**
 * @defgroup LED_Module LED Control
 * @brief   Simple status LED abstraction
 * @{
 */

#ifndef LED_H_
#define LED_H_


/********************
 *     Includes    *
 ********************/
#include "fsl_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/***********************************
 *     Public Macros / Defines     *
 ***********************************/

/***************************
 *     Public Typedefs     *
 ***************************/

/****************************
 *     Public Variables     *
 ****************************/

/**************************************
 *     Public Function Prototypes    *
 **************************************/
/**
 * @brief Initialize the LED hardware.
 *
 * Configures the GPIO used by the status LED and sets the initial state
 * (typically off). This function must be called before other LED APIs.
 *
 * @return status_t kStatus_Success on success, or an SDK error code.
 */
status_t LED_init(void);

/**
 * @brief Turn the LED on.
 *
 * Drive the LED GPIO to the active state.
 */
void LED_on(void);

/**
 * @brief Turn the LED off.
 *
 * Drive the LED GPIO to the inactive state.
 */
void LED_off(void);

/**
 * @brief Toggle the LED state.
 *
 * Invert the current LED output.
 */
void LED_toggle(void);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* LED_H_ */
