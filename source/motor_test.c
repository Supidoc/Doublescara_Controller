/************************************************************
 * @file    motor_test.c
 * @brief   Implementation file for motor test and diagnostics module
 * @author  dg
 * @date    24 Feb 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include <driver/pca9555a/pca9555a.h>
#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"
#include "cli.h"
#include "cli_utilities.h"
#include "log.h"
#include "stdio.h"
#include "motor_core.h"
#include "motor_test.h"
#include "sync_wait.h"
#include "motor_configs.h"
#include "motor_homing.h"
#include "scara_kinematics.h"
#include "motor_homing.h"
/************************************
 *     Private Macros / Defines    *
 ************************************/

/***************************
 *     Private Typedefs     *
 ***************************/

/*****************************************
 *     Private Function Declarations     *
 *****************************************/

void            init_motor(MTR_MotorConfig_t config);
static status_t MTT_wait_and_release(CHD_CmdHandle_t cmdHandle, TickType_t deadline);

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
    LOG_wait_for_queue_empty(portMAX_DELAY);
    init_motor(M_L_Arm());
    LOG_wait_for_queue_empty(portMAX_DELAY);
    init_motor(M_R_Arm());
    LOG_wait_for_queue_empty(portMAX_DELAY);
    init_motor(M_Platform());
    LOG_wait_for_queue_empty(portMAX_DELAY);
    init_motor(M_Magnet());
    LOG_wait_for_queue_empty(portMAX_DELAY);
    init_motor(M_Rotation());
    LOG_wait_for_queue_empty(portMAX_DELAY);
    MHM_init();

    MTR_MotorConfig_t lArmConfig = M_L_Arm();
    MTR_MotorConfig_t rArmConfig = M_R_Arm();
    MTR_MotorConfig_t platConfig = M_Platform();
    MTR_MotorConfig_t magConfig  = M_Magnet();
    MTR_MotorConfig_t rotConfig  = M_Rotation();


    MTR_MotorHandle_t l_arm_handle = NULL;
    MTR_MotorHandle_t r_arm_handle = NULL;
    MTR_MotorHandle_t plat_handle  = NULL;
    MTR_MotorHandle_t mag_handle   = NULL;
    MTR_MotorHandle_t rot_handle   = NULL;

    MTR_get_motor_by_label(lArmConfig.label, &l_arm_handle);
    if (l_arm_handle == NULL)
    {
        for (;;)
        {
        }
    }
    MTR_get_motor_by_label(rArmConfig.label, &r_arm_handle);
    if (r_arm_handle == NULL)
    {
        for (;;)
        {
        }
    }

    MTR_get_motor_by_label(platConfig.label, &plat_handle);
    if (plat_handle == NULL)
    {
        for (;;)
        {
        }
    }

    MTR_get_motor_by_label(magConfig.label, &mag_handle);
    if (mag_handle == NULL)
    {
        for (;;)
        {
        }
    }

    MTR_get_motor_by_label(rotConfig.label, &rot_handle);
    if (rot_handle == NULL)
    {
        for (;;)
        {
        }
    }

    if (SK_init_motor_handles(l_arm_handle, r_arm_handle, rot_handle) != kStatus_Success)
    {
        while (1)
            ;
    }

    //    CHD_CmdHandle_t platHandle = NULL;
    //    MTR_move_absolute_angle_async(plat_handle, -800, portMAX_DELAY, &platHandle);
    //
    //
    //    CHD_CmdHandle_t magHandle = NULL;
    //    MTR_move_absolute_angle_async(mag_handle, -3000, portMAX_DELAY, &magHandle);
    //
    //    MTT_wait_and_release(magHandle, portMAX_DELAY);
    //    MTT_wait_and_release(platHandle, portMAX_DELAY);

    MHM_home_left_arm(portMAX_DELAY);
    CHD_CmdHandle_t moveHandle = NULL;
    MTR_move_absolute_angle_async(l_arm_handle, 45, portMAX_DELAY, &moveHandle);
    MTT_wait_and_release(moveHandle, portMAX_DELAY);

    MHM_home_right_arm(portMAX_DELAY);
    MTR_move_absolute_angle_async(r_arm_handle, 135, portMAX_DELAY, &moveHandle);
    MTT_wait_and_release(moveHandle, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(100));

    SK_Point_t startPoint = {.x = 30, .y = 130};
    SK_move_to_xy_async(startPoint, 0, false,portMAX_DELAY, &moveHandle);
    MTT_wait_and_release(moveHandle, portMAX_DELAY);

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
    TickType_t      deadline  = SYW_deadline_from_timeout_ms(400);
    CHD_CmdHandle_t cmdHandle = NULL;
    if (MTR_init_handle_async(config, deadline, &cmdHandle) == kStatus_Success &&
        SYW_cmd_wait_result(cmdHandle, deadline, NULL) == kStatus_Success)
    {
        CHD_remove_cmd_handle_ref(cmdHandle);
        snprintf(logMsg, sizeof(logMsg), "[%s] Motor init completed", config.label);
        LOG_INFO(logMsg);
    }
    else
    {
        CHD_remove_cmd_handle_ref(cmdHandle);
        snprintf(logMsg, sizeof(logMsg), "[%s] Motor init failed or timed out", config.label);
        LOG_ERROR(logMsg);
    }
}

static status_t MTT_wait_and_release(CHD_CmdHandle_t cmdHandle, TickType_t deadline)
{
    if (cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    status_t status = SYW_cmd_wait_result(cmdHandle, deadline, NULL);
    CHD_remove_cmd_handle_ref(cmdHandle);
    return status;
}
