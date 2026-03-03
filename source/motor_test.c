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
#include "step.h"
#include "tmc2209.h"
/************************************
 *     Private Macros / Defines    *
 ************************************/

/***************************
 *     Private Typedefs     *
 ***************************/

/*****************************************
 *     Private Function Declarations     *
 *****************************************/
BaseType_t motor_move_command(char* pcWriteBuffer, size_t xWriteBufferLen,
                              const char* pcCommandString);

BaseType_t motor_stop(char* pcWriteBuffer, size_t xWriteBufferLen, const char* pcCommandString);

/****************************
 *     Public Variables     *
 ****************************/

/*****************************
 *     Private Variables     *
 *****************************/

static const CLI_Command_Definition_t xMotorMoveCommandDefinition = {
    .pcCommand    = "mmove",
    .pcHelpString = "mmove [OPTIONS]: moves the specified motor \r\n \t -m [MOTOR_ID] string "
                    "identifier of the motor \r\n \t -s [STEP_COUNT] Number of steps to move \r\n "
                    "\t -d [DIRECTION] direction specified with cw or ccw \r\n",
    .pxCommandInterpreter        = motor_move_command,
    .cExpectedNumberOfParameters = -1};

static const CLI_Command_Definition_t xMotorStopCommandDefinition = {
    .pcCommand = "mstop",
    .pcHelpString =
        "mstop [OPTIONS]: stops the specified motor \r\n \t -m [MOTOR_ID] string identifier of the "
        "motor \r\n\t -d decelerate the motor with constant deceleration \r\n ",
    .pxCommandInterpreter        = motor_stop,
    .cExpectedNumberOfParameters = -1};

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

    CLI_register_command(&xMotorMoveCommandDefinition);
    CLI_register_command(&xMotorStopCommandDefinition);

    STP_StepperConfig_t config;

    config.acceleration = 4000.0;
    config.dirGPIO      = GPIOB;
    config.dirPin       = 2;
    config.dirPort      = PORTB;
    config.endVelocity  = 360.0;
    config.ftmBase      = FTM3;
    config.ftmChannel   = kFTM_Chnl_0;
    config.label        = "motor0";
    config.stepGPIO     = GPIOD;
    config.stepPin      = 0;
    config.stepPort     = PORTD;
    config.stepMuxFTM   = kPORT_MuxAlt4;
    config.stepMuxGPIO  = kPORT_MuxAlt1;
    config.dirMux       = kPORT_MuxAlt1;

    STP_init_stepper(config);

    LOG_DEBUG("Configured stepper motor: motor0");

    TMC_Handle_t tmcHandle;
    tmcHandle.label             = "tmc0";
    tmcHandle.serialAdress      = TMC_SERIAL_ADDRESS_0;
    tmcHandle.transmissionCount = 0;
    tmcHandle.uartHandle        = &UART1_uart_handle;
    tmcHandle.uartRTOSHandle    = &UART1_rtos_handle;

    if (TMC_init_default(&tmcHandle, NULL) == kStatus_Success)
        LOG_DEBUG("Configured TMC2209 driver: tmc0");

    for (;;)
    {
        vTaskDelay(1);
    }
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/

BaseType_t motor_stop(char* pcWriteBuffer, size_t xWriteBufferLen, const char* pcCommandString)
{
    STP_StepperHandle_t* handle;
    char                 logMsg[80];

    uint8_t     parameterFound;
    const char* pcParameter = NULL;
    BaseType_t  parameterStringLength;
    uint8_t     doDeceleration = 0;

    CLU_get_parameter_value_string("-m", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (parameterFound == 0 || parameterStringLength == 0)
    {
        LOG_WARN("Motor command received without motor ID");
        strncpy(pcWriteBuffer, "Please specify a motorId", strlen("Please specify a motorId") + 1);
        return pdFALSE;
    }
    else
    {
        if (STP_get_handle_by_label(pcParameter, &handle) != kStatus_Success)
        {
            snprintf(logMsg, sizeof(logMsg), "Invalid motor ID requested: %.*s",
                     (int)parameterStringLength, pcParameter);
            LOG_WARN(logMsg);
            strncpy(pcWriteBuffer, "Invalid motorId", strlen("Invalid motorId") + 1);
            return pdFALSE;
        }
    }

    CLU_get_parameter_value_string("-d", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (parameterFound == 1)
    {
        doDeceleration = 1;
    }

    STP_stop(handle, doDeceleration);
    return pdFALSE;
}

BaseType_t motor_move_command(char* pcWriteBuffer, size_t xWriteBufferLen,
                              const char* pcCommandString)
{

    STP_StepperHandle_t* handle;
    uint32_t             stepCount;
    STP_Direction_t      direction;

    uint8_t     parameterFound;
    const char* pcParameter = NULL;
    BaseType_t  parameterStringLength;

    CLU_get_parameter_value_string("-m", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (parameterFound == 0 || parameterStringLength == 0)
    {
        strncpy(pcWriteBuffer, "Please specify a motorId", strlen("Please specify a motorId") + 1);
        return pdFALSE;
    }
    else
    {
        if (STP_get_handle_by_label(pcParameter, &handle) != kStatus_Success)
        {
            strncpy(pcWriteBuffer, "Invalid motorId", strlen("Invalid motorId") + 1);
            return pdFALSE;
        }
    }

    CLU_get_parameter_value_string("-s", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (parameterFound == 0 || parameterStringLength == 0)
    {
        strncpy(pcWriteBuffer, "Please specify a step count",
                strlen("Please specify a step count") + 1);
        return pdFALSE;
    }
    else
    {
        stepCount = strtoul(pcParameter, NULL, 0);
    }

    CLU_get_parameter_value_string("-d", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (parameterFound == 0 || parameterStringLength == 0)
    {
        strncpy(pcWriteBuffer, "Please specify a direction",
                strlen("Please specify a direction") + 1);
        return pdFALSE;
    }
    else
    {
        if (strncmp("cw", pcParameter, 2) == 0)
        {
            direction = STP_CLOCKWISE;
        }
        else if (strncmp("ccw", pcParameter, 3) == 0)
        {
            direction = STP_COUNTERCLOCKWISE;
        }
        else
        {
            strncpy(pcWriteBuffer, "Invalid direction", strlen("Invalid direction") + 1);
            return pdFALSE;
        }
    }

    int32_t stepsSigned = (direction == STP_COUNTERCLOCKWISE) ? stepCount : -stepCount;
    STP_move_relative(handle, stepsSigned);
    return pdFALSE;
}
