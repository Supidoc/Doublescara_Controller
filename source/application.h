/************************************************************
 * @file    application.h
 * @brief   Main application module
 *
 * This module provides the main entry points for initializing and running
 * the application. It serves as the central control for the system, coordinating
 * the initialization of subsystems and managing the main application loop.
 *
 * @note    This module is intended to be the starting point for the application
 *          and should be used to initialize all required modules and tasks.
 *
 * @note    This module is designed to work in an RTOS environment.
 *
 * @note    APP_Init() must be called before APP_Run().
 *
 * @author  dg
 * @date    18 Dec 2025
 ************************************************************/

/**
 * @defgroup APP_Module Application Module
 * @brief   Application initialization and runtime entry points
 * @{
 */

#ifndef APPLICATION_H_
#define APPLICATION_H_

/********************
 *     Includes		*
 ********************/

/***********************************
 *     Public Macros / Defines	   *
 ***********************************/

/***************************
 *     Public Typedefs	   *
 ***************************/

/****************************
 *     Public Variables     *
 ****************************/

/**************************************
 *     Public Function Prototypes	  *
 **************************************/

/**
 * @brief Initializes the application.
 *
 * This function initializes all required modules, peripherals, and tasks
 * for the application. It must be called before running the application.
 *
 * @note This function is typically called once during system startup.
 */
void APP_init(void);

/**
 * @brief Runs the main application loop.
 *
 * This function contains the main application logic and is intended to be
 * called after APP_Init(). It typically runs indefinitely in an RTOS environment.
 *
 * @note This function should not return under normal operation.
 */
void APP_run(void);

/** @} */ // End of APP_Module

#endif /* APPLICATION_H_ */
