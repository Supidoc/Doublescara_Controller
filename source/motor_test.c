/************************************************************
 * @file    motor_test.c
 * @brief   Implementation file for motor test and diagnostics module
 * @author  dg
 * @date    24 Feb 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"
#include "cli.h"
#include "cli_utilities.h"
#include "log.h"
#include "stdio.h"
#include "motor.h"
#include "motor_test.h"
/************************************
 *     Private Macros / Defines    *
 ************************************/

/***************************
 *     Private Typedefs     *
 ***************************/

/*****************************************
 *     Private Function Declarations     *
 *****************************************/

/****************************
 *     Public Variables     *
 ****************************/

/*****************************
 *     Private Variables     *
 *****************************/

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

status_t MTT_init(void)
{
    return kStatus_Success;
}

void MTT_task(void* pvParameters)
{
    LOG_INFO("Started MTT Task");

    MTR_MotorConfig_t motorConfig;

#if MTT_MOTOR_STEPPER_TYPE_NORMAL
    motorConfig.acceleration = 400;
    motorConfig.endVelocity  = 720;
    motorConfig.stepAngle    = 1.8 // Typical NEMA stepper: 1.8 degrees per step
                            motorConfig.microstep = 64;  // 8x microstepping
    motorConfig.reductionFactor                   = 1.0; // No gearbox
#elif MTT_MOTOR_STEPPER_TYPE_LINEAR_BIG
    motorConfig.acceleration    = 200;
    motorConfig.endVelocity     = 360;
    motorConfig.stepAngle       = 1.8; // Typical NEMA stepper: 1.8 degrees per step
    motorConfig.microstep       = 16;  // 8x microstepping
    motorConfig.reductionFactor = 1.0; // No gearbox
#elif MTT_MOTOR_STEPPER_TYPE_LINEAR_SMALL
    motorConfig.acceleration    = 200 * 8;
    motorConfig.endVelocity     = 360 * 8;
    motorConfig.stepAngle       = 7.5; // Typical NEMA stepper: 1.8 degrees per step
    motorConfig.microstep       = 128; // 8x microstepping
    motorConfig.reductionFactor = 1.0; // No gearbox
#endif
    motorConfig.label = "motor0";

    // Configure stepper parameters
    motorConfig.stepperConfig.dirGPIO               = GPIOB;
    motorConfig.stepperConfig.dirPin                = 2;
    motorConfig.stepperConfig.dirPort               = PORTB;
    motorConfig.stepperConfig.ftmBase               = FTM3;
    motorConfig.stepperConfig.ftmChannel            = kFTM_Chnl_0;
    motorConfig.stepperConfig.stepGPIO              = GPIOD;
    motorConfig.stepperConfig.stepPin               = 0;
    motorConfig.stepperConfig.stepPort              = PORTD;
    motorConfig.stepperConfig.stepMuxFTM            = kPORT_MuxAlt4;
    motorConfig.stepperConfig.stepMuxGPIO           = kPORT_MuxAlt1;
    motorConfig.stepperConfig.dirMux                = kPORT_MuxAlt1;
    motorConfig.stepperConfig.dirLogicHighClockwise = 1;

    // Configure TMC driver
    motorConfig.tmcConfig.serialAdress   = TMC_SERIAL_ADDRESS_0;
    motorConfig.tmcConfig.uartHandle     = &UART1_uart_handle;
    motorConfig.tmcConfig.uartRTOSHandle = &UART1_rtos_handle;

#if MTT_MOTOR_STEPPER_TYPE_LINEAR_BIG || MTT_MOTOR_STEPPER_TYPE_LINEAR_SMALL
    motorConfig.tmcConfig.iHoldCurrentA = 0.1;
    motorConfig.tmcConfig.iRunCurrentA  = 0.1;
#elif MTT_MOTOR_STEPPER_TYPE_NORMAL
    motorConfig.tmcConfig.iHoldCurrentA = 0.2;
    motorConfig.tmcConfig.iRunCurrentA  = 0.4;
#endif
    if (MTR_init_handle(motorConfig, 3000) == kStatus_Success)
    {
        LOG_INFO("Motor initialized successfully: motor0");
    }
    else
    {
        LOG_ERROR("Failed to initialize motor: motor0");
    }

    MTR_MotorHandle_t handle = NULL;

    for (;;)
    {
        vTaskDelay(100);
    }
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/
