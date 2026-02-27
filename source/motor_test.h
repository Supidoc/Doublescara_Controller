/************************************************************
 * @file    motor_test.h
 * @brief   Motor test and diagnostics module
 *
 * This module provides functionality for testing and diagnosing stepper motors
 * and motor drivers. It allows for validation of motor control and communication
 * integrity during development and debugging.
 *
 * @author  dg
 * @date    24 Feb 2026
 ************************************************************/

/**
 * @defgroup MotorTest_Module Motor Test and Diagnostics
 * @brief   Functions for testing stepper motors and motor drivers
 * @{
 */

#ifndef MOTOR_TEST_H_
#define MOTOR_TEST_H_

/********************
 *     Includes    *
 ********************/

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
 * @brief Initializes the motor test module
 *
 * Sets up the motor test module for operation. This function should be called
 * once during system initialization before using any other motor test functions.
 *
 * @return Status code indicating success or failure
 *         - kStatus_Success: Module initialized successfully
 *         - Other: An error occurred during initialization
 */
status_t MTT_init(void);

/**
 * @brief FreeRTOS task for motor testing and diagnostics
 *
 * This is the main task function for the motor test module. It initializes
 * the stepper motor driver, configures the TMC2209 stepper motor driver,
 * and registers CLI commands for motor control. This task runs indefinitely
 * and should be created as a FreeRTOS task.
 *
 * @param[in] pvParameters Pointer to task parameters (typically NULL for this task)
 *
 * @note This function should be registered as a FreeRTOS task and will block
 *       indefinitely, yielding periodically.
 */
void MTT_task(void* pvParameters);

/** @} */ // End of MotorTest_Module

#endif /* MOTOR_TEST_H_ */
