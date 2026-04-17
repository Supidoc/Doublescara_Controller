/************************************************************
 * @file    motor_homing.h
 * @brief   Filedescription
 * @author  dg
 * @date    17 Apr 2026
 ************************************************************/

#ifndef MOTOR_HOMING_H_
#define MOTOR_HOMING_H_

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

status_t MHM_init(void);

status_t MHM_home_left_arm(TickType_t deadline);

status_t MHM_home_right_arm(TickType_t deadline);

void MHM_homing_arm_interrupt_handler(PORT_Type* port, uint32_t pin);

#endif // MOTOR_HOMING_H_