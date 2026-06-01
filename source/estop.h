/************************************************************
 * @file    estop.h
 * @brief   Emergency stop (E-Stop) interface
 * @details Provides a simple global emergency-stop API used by the
 *          application to immediately stop motors and disable actuators
 *          when a hazardous condition is detected. The implementation
 *          should perform any required hardware shutdown actions.
 * @author  dg
 * @date    15 May 2026
 ************************************************************/

/**
 * @defgroup Estop_Module Emergency Stop
 * @brief   Global emergency-stop API and helpers
 * @{
 */

#ifndef ESTOP_H_
#define ESTOP_H_

/********************
 *     Includes    *
 ********************/
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
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
    /** @brief Global emergency-stop flag.
     *
     * True when an emergency-stop is active. Modules that perform motion
     * or drive external actuators should check this flag and refrain from
     * issuing commands while it is set.
     */
    extern bool estopFlag;

    extern bool estopPressed;

    /**************************************
     *     Public Function Prototypes    *
     **************************************/
    /**
     * @brief Assert the emergency stop.
     *
     * Sets the global E-Stop flag and triggers any immediate shutdown actions
     * required by the platform (motor disable, brake engage, etc.). This
     * function is safe to call from task context. If ISR safety is required
     * a separate API should be provided by the implementation.
     */
    void ESTOP_SetEstop(void);

    /**
     * @brief Clear the emergency stop.
     *
     * Clears the global E-Stop flag and allows normal operation to resume.
     * Callers should ensure it is safe to resume motors and actuators before
     * calling this function.
     */
    void ESTOP_ClearEstop(void);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* ESTOP_H_ */
