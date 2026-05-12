/************************************************************
 * @file    grabber.c
 * @brief   Filedescription
 * @author  dg
 * @date    27 Apr 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include "grabber.h"
#include "motor_core.h"
#include "motor_configs.h"
#include "sync_wait.h"
#include "cmd_handle.h"

/************************************
 *     Private Macros / Defines    *
 ************************************/

/***************************
 *     Private Typedefs     *
 ***************************/

/*****************************************
 *     Private Function Declarations     *
 *****************************************/

static status_t wait_and_release(CHD_CmdHandle_t cmdHandle, TickType_t deadline);

/****************************
 *     Public Variables     *
 ****************************/

/*****************************
 *     Private Variables     *
 *****************************/

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

void GRB_grab(void)
{
    MTR_MotorConfig_t platConfig = M_Platform();
    MTR_MotorConfig_t magConfig  = M_Magnet();

    MTR_MotorHandle_t plat_handle = NULL;
    MTR_MotorHandle_t mag_handle  = NULL;

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

    CHD_CmdHandle_t platHandle = NULL;
    MTR_move_angle_async(plat_handle, 720, portMAX_DELAY, &platHandle);
    wait_and_release(platHandle, portMAX_DELAY);

    CHD_CmdHandle_t magHandle = NULL;
    MTR_move_angle_async(mag_handle, 2400, portMAX_DELAY, &magHandle);
    wait_and_release(magHandle, portMAX_DELAY);

    MTR_move_angle_async(plat_handle, -720, portMAX_DELAY, &platHandle);
    wait_and_release(platHandle, portMAX_DELAY);
}

void GRB_release(void)
{
    MTR_MotorConfig_t platConfig = M_Platform();
    MTR_MotorConfig_t magConfig  = M_Magnet();

    MTR_MotorHandle_t plat_handle = NULL;
    MTR_MotorHandle_t mag_handle  = NULL;

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

    CHD_CmdHandle_t platHandle = NULL;
    MTR_move_angle_async(plat_handle, 720, portMAX_DELAY, &platHandle);
    wait_and_release(platHandle, portMAX_DELAY);

    CHD_CmdHandle_t magHandle = NULL;
    MTR_move_angle_async(mag_handle, -2400, portMAX_DELAY, &magHandle);
    wait_and_release(magHandle, portMAX_DELAY);

    MTR_move_angle_async(plat_handle, -720, portMAX_DELAY, &platHandle);
    wait_and_release(platHandle, portMAX_DELAY);
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/

static status_t wait_and_release(CHD_CmdHandle_t cmdHandle, TickType_t deadline)
{
    if (cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    status_t status = SYW_cmd_wait_result(cmdHandle, deadline, NULL);
    CHD_remove_cmd_handle_ref(cmdHandle);
    return status;
}
