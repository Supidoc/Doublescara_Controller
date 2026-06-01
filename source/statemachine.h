/************************************************************
 * @file    statemachine.h
 * @brief   High-level state machine public API
 * @details State machine task and public state enum used by the application.
 * @author  rp
 * @date    21 Apr 2026
 ************************************************************/

/**
 * @defgroup Statemachine_Module State Machine
 * @brief   Top-level application state machine interfaces
 * @{
 */

#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

#ifdef __cplusplus
extern "C"
{
#endif

    /********************
     *     Includes    *
     ********************/

    /***********************************
     *     Public Macros / Defines     *
     ***********************************/

    /***************************
     *     Public Typedefs     *
     ***************************/

    /**
     * @brief Application state machine states
     */
    typedef enum
    {
        IDLE,    /**< No action in progress */
        EXECUTE, /**< Currently executing a command */
        DONE,    /**< Command completed successfully */
        ERROR,    /**< Error state */
		ESTOP,
		RESET
    } State;

    /****************************
     *     Public Variables     *
     ****************************/

    /**************************************
     *     Public Function Prototypes    *
     **************************************/

    /**
     * @brief FreeRTOS task implementing the application state machine.
     *
     * This function is intended to be created as a FreeRTOS task and will
     * run indefinitely, processing incoming commands and driving the system
     * state transitions.
     *
     * @param[in] pvParameters Task parameter (typically NULL)
     */
    void Statemachine_task(void* pvParameters);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* STATEMACHINE_H_ */
