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
#include "task_helpers.h"
#include "pca9555a.h"
#include "motor_configs.h"
/************************************
 *     Private Macros / Defines    *
 ************************************/

/***************************
 *     Private Typedefs     *
 ***************************/

/*****************************************
 *     Private Function Declarations     *
 *****************************************/

void init_motor(MTR_MotorConfig_t config);

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

    init_motor(M_L_Arm());
    init_motor(M_R_Arm());
    LOG_wait_for_queue_empty(portMAX_DELAY);
    init_motor(M_Platform());
    init_motor(M_Magnet());
    LOG_wait_for_queue_empty(portMAX_DELAY);
    init_motor(M_Rotation());

    for (;;)
    {
        vTaskDelay(100);
    }
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/

void init_motor(MTR_MotorConfig_t config)
{
    char            logMsg[60];
    TickType_t      deadline  = THE_deadline_from_timeout_ms(100);
    THE_CmdHandle_t cmdHandle = NULL;
    if (MTR_init_handle_async(config, deadline, &cmdHandle) == kStatus_Success &&
        THE_cmd_wait_result(cmdHandle, deadline, NULL) == kStatus_Success)
    {
        THE_remove_cmd_handle_ref(cmdHandle);
    }
    else
    {
        THE_remove_cmd_handle_ref(cmdHandle);
    }
}
