/************************************************************
 * @file    motor_configs.h
 * @brief   Motor configuration factory functions
 * @details Provides configuration builder functions for creating
 *          MTR_MotorConfig_t structures for each motor in the
 *          double SCARA system. Each function configures all
 *          parameters for a specific motor.
 * @author  dg
 * @date    2 Apr 2026
 ************************************************************/

/**
 * @defgroup MotorConfigs_Module Motor Configuration Module
 * @brief   Factory functions for motor configuration structures
 * @details Provides five factory functions for complete motor system:
 *          SCARA System (UART1):\n
 *          - M_L_Arm(): Left SCARA arm motor (serial address 0, FTM3 ch.2, GPIOC pin 11)\n
 *          - M_R_Arm(): Right SCARA arm motor (serial address 1, FTM3 ch.1, GPIOD pin 2)\n
 *          - M_Platform(): Platform motor (serial address 2, FTM3 ch.0, GPIOD pin 0)\n
 *          Auxiliary System (UART1):\n
 *          - M_Magnet(): Electromagnet control (serial address 3, FTM3 ch.3, GPIOC pin 10, 7.5°
 * steps)\n Independent System (UART0):\n
 *          - M_Rotation(): Rotation/indexing motor (serial address 0, FTM3 ch.4, GPIOC pin 9)\n
 *          All motors controlled via PCA9555A I2C IO expander for direction pins.\n
 *          SCARA arms: 1.8° step, 16x microstep, 3.0x reduction, 0.8A run, 0.1A hold.\n
 *          Platform: 1.8° step, 1x microstep, 1.0x reduction, 0.1A both.\n
 *          Magnet: 7.5° step, 1x microstep, 1.0x reduction, 0.1A both.\n
 *          Rotation: 1.8° step, 16x microstep, 1.0x reduction, 0.2A run, 0.1A hold.
 * @{
 */

#ifndef MOTOR_CONFIGS_H_
#define MOTOR_CONFIGS_H_

#ifdef __cplusplus
extern "C"
{
#endif

/********************
 *     Includes    *
 ********************/
#include "MK22F51212.h"
#include "motor_core.h"
#include "peripherals.h"

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
     * @brief Creates and returns the configuration for the left arm motor
     * @details Initializes all stepper and TMC2209 driver parameters for the
     *          left arm motor including acceleration, velocity, microstepping,
     *          gear reduction, and pin assignments.
     * @return MTR_MotorConfig_t structure with complete left arm motor configuration
     */
    static inline MTR_MotorConfig_t M_L_Arm(void)
    {
        MTR_MotorConfig_t motorConfig;

        motorConfig.label = "m_l_arm";

        motorConfig.acceleration    = 400;
        motorConfig.endVelocity     = 600;
        motorConfig.stepAngle       = 1.8;
        motorConfig.microstep       = 16;
        motorConfig.reductionFactor = 3.0;

        // Configure stepper parameters
        motorConfig.stepperConfig.dirPin                = 2;
        motorConfig.stepperConfig.dirPort               = PCA_PORT_0;
        motorConfig.stepperConfig.ftmBase               = FTM3;
        motorConfig.stepperConfig.ftmChannel            = kFTM_Chnl_2;
        motorConfig.stepperConfig.stepGPIO              = GPIOC;
        motorConfig.stepperConfig.stepPin               = 11;
        motorConfig.stepperConfig.stepPort              = PORTC;
        motorConfig.stepperConfig.dirLogicHighClockwise = 0;

        // Configure TMC driver
        motorConfig.tmcConfig.serialAdress   = TMC_SERIAL_ADDRESS_2;
        motorConfig.tmcConfig.uartHandle     = &UART1_uart_handle;
        motorConfig.tmcConfig.uartRTOSHandle = &UART1_rtos_handle;

        motorConfig.tmcConfig.iHoldCurrentA = 0.4;
        motorConfig.tmcConfig.iRunCurrentA  = 2.0;
        return motorConfig;
    }

    /**
     * @brief Creates and returns the configuration for the right arm motor
     * @details Initializes all stepper and TMC2209 driver parameters for the
     *          right arm motor including acceleration, velocity, microstepping,
     *          gear reduction, and pin assignments.
     * @return MTR_MotorConfig_t structure with complete right arm motor configuration
     */
    static inline MTR_MotorConfig_t M_R_Arm(void)
    {
        MTR_MotorConfig_t motorConfig;

        motorConfig.label = "m_r_arm";

        motorConfig.acceleration    = 400;
        motorConfig.endVelocity     = 600;
        motorConfig.stepAngle       = 1.8;
        motorConfig.microstep       = 16;
        motorConfig.reductionFactor = 3.0;

        // Configure stepper parameters
        motorConfig.stepperConfig.dirPin                = 1;
        motorConfig.stepperConfig.dirPort               = PCA_PORT_0;
        motorConfig.stepperConfig.ftmBase               = FTM3;
        motorConfig.stepperConfig.ftmChannel            = kFTM_Chnl_1;
        motorConfig.stepperConfig.stepGPIO              = GPIOD;
        motorConfig.stepperConfig.stepPin               = 1;
        motorConfig.stepperConfig.stepPort              = PORTD;
        motorConfig.stepperConfig.dirLogicHighClockwise = 0;

        // Configure TMC driver
        motorConfig.tmcConfig.serialAdress   = TMC_SERIAL_ADDRESS_1;
        motorConfig.tmcConfig.uartHandle     = &UART1_uart_handle;
        motorConfig.tmcConfig.uartRTOSHandle = &UART1_rtos_handle;

        motorConfig.tmcConfig.iHoldCurrentA = 0.4;
        motorConfig.tmcConfig.iRunCurrentA  = 2.0;
        return motorConfig;
    }

    /**
     * @brief Creates and returns the configuration for the platform motor
     * @details Initializes all stepper and TMC2209 driver parameters for the
     *          platform rotation motor including acceleration, velocity,
     *          microstepping, and pin assignments.
     * @return MTR_MotorConfig_t structure with complete platform motor configuration
     */
    static inline MTR_MotorConfig_t M_Platform(void)
    {
        MTR_MotorConfig_t motorConfig;

        motorConfig.label = "m_platform";

        motorConfig.acceleration    = 500;
        motorConfig.endVelocity     = 1000;
        motorConfig.stepAngle       = 1.8;
        motorConfig.microstep       = 4;
        motorConfig.reductionFactor = 1.0;

        // Configure stepper parameters
        motorConfig.stepperConfig.dirPin                = 0;
        motorConfig.stepperConfig.dirPort               = PCA_PORT_0;
        motorConfig.stepperConfig.ftmBase               = FTM3;
        motorConfig.stepperConfig.ftmChannel            = kFTM_Chnl_0;
        motorConfig.stepperConfig.stepGPIO              = GPIOD;
        motorConfig.stepperConfig.stepPin               = 0;
        motorConfig.stepperConfig.stepPort              = PORTD;
        motorConfig.stepperConfig.dirLogicHighClockwise = 0;

        // Configure TMC driver
        motorConfig.tmcConfig.serialAdress   = TMC_SERIAL_ADDRESS_0;
        motorConfig.tmcConfig.uartHandle     = &UART1_uart_handle;
        motorConfig.tmcConfig.uartRTOSHandle = &UART1_rtos_handle;

        motorConfig.tmcConfig.iHoldCurrentA = 0.1;
        motorConfig.tmcConfig.iRunCurrentA  = 0.9;
        return motorConfig;
    }

    /**
     * @brief Creates and returns the configuration for the magnet motor
     * @details Initializes all stepper and TMC2209 driver parameters for the
     *          electromagnet control motor. Configured with 7.5° step angle for
     *          reduced step resolution, 1x microstepping. Uses UART1 serial address 3.
     *          Hardware: FTM3 channel 3, GPIO C pin 10, PCA9555A port bit 3 for direction.
     *          Acceleration: 3200°/s², velocity: 5760°/s. Hold and run currents: 0.1A.
     * @return MTR_MotorConfig_t structure with complete magnet motor configuration
     */
    static inline MTR_MotorConfig_t M_Magnet(void)
    {
        MTR_MotorConfig_t motorConfig;

        motorConfig.label = "m_magnet";

        motorConfig.acceleration    = 2000;
        motorConfig.endVelocity     = 2000;
        motorConfig.stepAngle       = 7.5;
        motorConfig.microstep       = 1;
        motorConfig.reductionFactor = 1.0;

        // Configure stepper parameters
        motorConfig.stepperConfig.dirPin                = 3;
        motorConfig.stepperConfig.dirPort               = PCA_PORT_0;
        motorConfig.stepperConfig.ftmBase               = FTM3;
        motorConfig.stepperConfig.ftmChannel            = kFTM_Chnl_3;
        motorConfig.stepperConfig.stepGPIO              = GPIOC;
        motorConfig.stepperConfig.stepPin               = 10;
        motorConfig.stepperConfig.stepPort              = PORTC;
        motorConfig.stepperConfig.dirLogicHighClockwise = 0;

        // Configure TMC driver
        motorConfig.tmcConfig.serialAdress   = TMC_SERIAL_ADDRESS_3;
        motorConfig.tmcConfig.uartHandle     = &UART1_uart_handle;
        motorConfig.tmcConfig.uartRTOSHandle = &UART1_rtos_handle;

        motorConfig.tmcConfig.iHoldCurrentA = 0.05;
        motorConfig.tmcConfig.iRunCurrentA  = 0.4;
        return motorConfig;
    }

    /**
     * @brief Creates and returns the configuration for the rotation motor
     * @details Initializes all stepper and TMC2209 driver parameters for the
     *          mechanical rotation/indexing motor. Uses 1.8° step angle with 16x microstepping
     *          for fine position control. Configured on UART0 (serial address 0) separate
     *          from the SCARA arm system. Hardware: FTM3 channel 4, GPIO C pin 9,
     *          PCA9555A port bit 4 for direction. Acceleration: 1600°/s², velocity: 2880°/s.
     *          Hold current: 0.1A, run current: 0.2A.
     * @return MTR_MotorConfig_t structure with complete rotation motor configuration
     */
    static inline MTR_MotorConfig_t M_Rotation(void)
    {
        MTR_MotorConfig_t motorConfig;

        motorConfig.label = "m_rot";

        motorConfig.acceleration    = 200;
        motorConfig.endVelocity     = 100;
        motorConfig.stepAngle       = 1.8;
        motorConfig.microstep       = 16;
        motorConfig.reductionFactor = 1.0;

        // Configure stepper parameters
        motorConfig.stepperConfig.dirPin                = 4;
        motorConfig.stepperConfig.dirPort               = PCA_PORT_0;
        motorConfig.stepperConfig.ftmBase               = FTM3;
        motorConfig.stepperConfig.ftmChannel            = kFTM_Chnl_4;
        motorConfig.stepperConfig.stepGPIO              = GPIOC;
        motorConfig.stepperConfig.stepPin               = 9;
        motorConfig.stepperConfig.stepPort              = PORTC;
        motorConfig.stepperConfig.dirLogicHighClockwise = 1;

        // Configure TMC driver
        motorConfig.tmcConfig.serialAdress   = TMC_SERIAL_ADDRESS_0;
        motorConfig.tmcConfig.uartHandle     = &UART0_uart_handle;
        motorConfig.tmcConfig.uartRTOSHandle = &UART0_rtos_handle;

        motorConfig.tmcConfig.iHoldCurrentA = 0.1;
        motorConfig.tmcConfig.iRunCurrentA  = 0.2;
        return motorConfig;
    }

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* MOTOR_CONFIGS_H_ */
