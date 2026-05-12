/************************************************************
 * @file    motor_homing.h
 * @brief   Internal motor homing helpers.
 * @author  dg
 * @date    17 Apr 2026
 ************************************************************/

#ifndef MOTOR_HOMING_H_
#define MOTOR_HOMING_H_

#ifdef __cplusplus
extern "C"
{
#endif

/********************
 *     Includes    *
 ********************/
#include "motor_internal.h"
#include "FreeRTOS.h"
#include "MK22F51212.h"

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

    /** @brief Initializes homing resources and state. */
    status_t MHM_init(void);

    /** @brief Homes the left arm motor. */
    status_t MHM_home_left_arm(TickType_t deadline);

    /** @brief Homes the right arm motor. */
    status_t MHM_home_right_arm(TickType_t deadline);

    /** @brief Interrupt handler hook used while a homing sequence is running. */
    void MHM_homing_arm_interrupt_handler(PORT_Type* port, uint32_t pin);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_HOMING_H_