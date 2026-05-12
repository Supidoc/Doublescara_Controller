/************************************************************
 * @file    step_shared.h
 * @brief   Shared stepper types, constants, and handle declarations.
 * @author  dg
 * @date    13 Apr 2026
 ************************************************************/

#ifndef STEP_SHARED_H_
#define STEP_SHARED_H_

#ifdef __cplusplus
extern "C"
{
#endif

/********************
 *     Includes    *
 ********************/
#include <stdint.h>
#include <driver/pca9555a/pca9555a.h>
#include "fsl_device_registers.h"
#include "fsl_common.h"
#include "fsl_ftm.h"
#include "fsl_gpio.h"
#include "fsl_port.h"

/***********************************
 *     Public Macros / Defines     *
 ***********************************/

/**
 * @brief Maximum number of stepper motor handles that can be created.
 */
#define STP_MAX_HANDLE_COUNT 6

/**
 * @brief Size of the command queue.
 */
#define STP_CMD_QUEUE_SIZE 20

/** @brief Maximum number of stepper command handles available. */
#define STP_MAX_CMD_HANDLE_COUNT 30

/**
 * @brief Maximum number of steps in the acceleration/deceleration lookup table.
 * This limits the size of pre-calculated delay profiles to conserve memory.
 * Movements with more acceleration steps will be clamped to this value.
 */
#define STP_MAX_ACCEL_TABLE_SIZE 1000

/**
 * @brief Interpolation factor between lookup table entries.
 * Each table entry spans this many acceleration steps.
 * A value of 1 means no interpolation.
 */
#define STP_ACCEL_TABLE_INTERP_FACTOR 1

/**
 * @brief Number of acceleration lookup tables in the static pool.
 * Each stepper movement requires one table from the pool.
 * If all tables are in use, new movement commands will fail until a table is freed.
 */
#define STP_ACCEL_TABLE_POOL_SIZE 8

    /***************************
     *     Public Typedefs     *
     ***************************/

    /**
     * @brief Enumeration of stepper motor movement states.
     *
     * Represents the current state of a stepper motor movement operation.
     */
    typedef enum _STP_MovementState
    {
        /** @brief Motor is idling */
        STP_MOVEMENT_IDLE,
        /** @brief Movement command received, but profile not yet calculated */
        STP_MOVEMENT_CALLED,
        /** @brief Movement profile calculated and ready to start */
        STP_MOVEMENT_PLANNED,
        /** @brief Movement is waiting for synchronized start trigger */
        STP_MOVEMENT_STARTED,
        /** @brief Motor is currently accelerating */
        STP_MOVEMENT_ACCELERATING,
        /** @brief Motor is currently running with constant velocity */
        STP_MOVEMENT_CONST_VELOCITY,
        /** @brief Motor is currently decelerating */
        STP_MOVEMENT_DECELRATING,
        /** @brief Movement stopped before completion */
        STP_MOVEMENT_STOPPED,
        /** @brief Movement completed */
        STP_MOVEMENT_FINISHED,
        /** @brief Movement completed successfully */
        STP_MOVEMENT_SUCCESSFUL,
        /** @brief Movement failed due to an error */
        STP_MOVEMENT_FAILED
    } STP_MovementState_t;

    /**
     * @brief Enumeration of stepper motor rotation directions.
     */
    typedef enum _STP_Direction
    {
        /** @brief Rotate clockwise */
        STP_CLOCKWISE,
        /** @brief Rotate counterclockwise */
        STP_COUNTERCLOCKWISE
    } STP_Direction_t;

    /**
     * @brief Opaque handle type for stepper motor control instances.
     *
     */
    typedef struct _STP_StepperHandleImpl* STP_Handle_t;

    /**
     * @brief Configuration structure for initializing a stepper motor.
     *
     * Contains all necessary configuration parameters to initialize and set up
     * a new stepper motor control instance.
     */
    typedef struct _STP_StepperConfig
    {
        FTM_Type*  ftmBase;    /**< Pointer to FTM module base address */
        ftm_chnl_t ftmChannel; /**< FTM channel for step output */
        PORT_Type* stepPort;   /**< PORT module for step pin mux control */
        GPIO_Type* stepGPIO;   /**< GPIO module for step pin */
        uint8_t    stepPin;    /**< Step output pin number */
        PCA_Port_t dirPort;    /**< PORT for direction pin control */
        uint8_t    dirPin;     /**< Direction output pin number */
        uint8_t
            dirLogicHighClockwise; /**< Flag: 1 if high = clockwise, 0 if high = counterclockwise */
        double      acceleration;  /**< Acceleration in steps/s² */
        double      endVelocity;   /**< Final velocity in steps/s */
        const char* label;         /**< Identifier label for logging (e.g., "Motor_X") */
    } STP_StepperConfig_t;

    /****************************
     *     Public Variables     *
     ****************************/

    /**************************************
     *     Public Function Prototypes    *
     **************************************/

#ifdef __cplusplus
}
#endif

#endif // STEP_SHARED_H_
