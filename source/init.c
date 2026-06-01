/************************************************************
 * @file    init.c
 * @brief   Filedescription
 * @author  dg
 * @date    22 May 2026
 ************************************************************/


/********************
 *     Includes    *
 ********************/
#include <pca9555a.h>
#include <cli.h>
#include <cli_utilities.h>
#include <log.h>
#include <motor_homing.h>
#include <motor_homing.h>
#include <scara_kinematics.h>
#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"
#include "stdio.h"
#include "motor_core.h"
#include "motor_test.h"
#include "sync_wait.h"
#include "motor_configs.h"
#include "stdbool.h"
#include "display_helpers.h"
#include <display/McuFontCour10Normal.h>
#include <display/McuFontDisplay.h>
#include <display/McuGDisplaySSD1306.h>
/************************************
 *     Private Macros / Defines    *
 ************************************/

/***************************
 *     Private Typedefs     *
 ***************************/

/*****************************************
 *     Private Function Declarations     *
 *****************************************/

static status_t INITi_init_motor(MTR_MotorConfig_t config);
static status_t INITi_wait_and_release(CHD_CmdHandle_t cmdHandle, TickType_t deadline);

/****************************
 *     Public Variables     *
 ****************************/
static bool initialized = false;

/*****************************
 *     Private Variables     *
 *****************************/

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

status_t INIT_init_motors(void){
	initialized = false;
    LOG_wait_for_queue_empty(portMAX_DELAY);
    if(INITi_init_motor(M_L_Arm()) != kStatus_Success){
    	return kStatus_Fail;
    }
    LOG_wait_for_queue_empty(portMAX_DELAY);
    if(INITi_init_motor(M_R_Arm()) != kStatus_Success){
    	return kStatus_Fail;
    }
    LOG_wait_for_queue_empty(portMAX_DELAY);
    if(INITi_init_motor(M_Platform()) != kStatus_Success){
    	return kStatus_Fail;
    }
    LOG_wait_for_queue_empty(portMAX_DELAY);
    if(INITi_init_motor(M_Magnet()) != kStatus_Success){
    	return kStatus_Fail;
    }
    LOG_wait_for_queue_empty(portMAX_DELAY);
    if(INITi_init_motor(M_Rotation()) != kStatus_Success){
    	return kStatus_Fail;
    }
    LOG_wait_for_queue_empty(portMAX_DELAY);
    if(MHM_init() != kStatus_Success){
    	return kStatus_Fail;
    }
    return kStatus_Success;
}

status_t INIT_init_robot(void){
    CHD_CmdHandle_t moveHandle = NULL;

	if(initialized){
	    SK_Point_t startPoint = {.x = 30, .y = 130};
	    if(SK_move_to_xy_async(startPoint, 0, false, portMAX_DELAY, &moveHandle) != kStatus_Success)
	    {
	    	return kStatus_Fail;
	    }
	    if(INITi_wait_and_release(moveHandle, portMAX_DELAY) != kStatus_Success)
	    {
	    	return kStatus_Fail;
	    }
	}
    	initialized = false;

		//INIT_init_motors();

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
	    	return kStatus_Fail;

	    }
	    MTR_get_motor_by_label(rArmConfig.label, &r_arm_handle);
	    if (r_arm_handle == NULL)
	    {
	    	return kStatus_Fail;

	    }

	    MTR_get_motor_by_label(platConfig.label, &plat_handle);
	    if (plat_handle == NULL)
	    {
	    	return kStatus_Fail;

	    }

	    MTR_get_motor_by_label(magConfig.label, &mag_handle);
	    if (mag_handle == NULL)
	    {
	    	return kStatus_Fail;

	    }

	    MTR_get_motor_by_label(rotConfig.label, &rot_handle);
	    if (rot_handle == NULL)
	    {
	    	return kStatus_Fail;
	    }

	    if (SK_init_motor_handles(l_arm_handle, r_arm_handle, rot_handle) != kStatus_Success)
	    {
	    	return kStatus_Fail;
	    }

	    CHD_CmdHandle_t freewheelHandle;
	    if(MTR_enable_freewheeling_async(r_arm_handle, portMAX_DELAY, &freewheelHandle) != kStatus_Success){
	    	return kStatus_Fail;
	    }
	    if(INITi_wait_and_release(freewheelHandle, portMAX_DELAY) != kStatus_Success)
	    {
	    	return kStatus_Fail;
	    }
	    if(MTR_enable_freewheeling_async(l_arm_handle, portMAX_DELAY, &freewheelHandle) != kStatus_Success){
	    	return kStatus_Fail;
	    }
	    if(INITi_wait_and_release(freewheelHandle, portMAX_DELAY) != kStatus_Success)
	    {
	    	return kStatus_Fail;
	    }
	    Show2Liner("Move arms to A5", "then press start");
	    uint8_t pinvalue;
	    while(1){
		if (PCA_Read_Pin(PCA_PORT_1, 2, &pinvalue) == kStatus_Success)
		{
			if (pinvalue == 1)
			{
				break;
			}
		}
	    vTaskDelay(pdMS_TO_TICKS(100));
}
	    Show1Liner("Homing");

	    if(MTR_disable_freewheeling_async(r_arm_handle, portMAX_DELAY, &freewheelHandle) != kStatus_Success){
	    	return kStatus_Fail;
	    }
	    if(INITi_wait_and_release(freewheelHandle, portMAX_DELAY) != kStatus_Success)
	    {
	    	return kStatus_Fail;
	    }
	    if(MTR_disable_freewheeling_async(l_arm_handle, portMAX_DELAY, &freewheelHandle) != kStatus_Success){
	    	return kStatus_Fail;
	    }
	    if(INITi_wait_and_release(freewheelHandle, portMAX_DELAY) != kStatus_Success)
	    {
	    	return kStatus_Fail;
	    }


	    CHD_CmdHandle_t platHandle = NULL;
	    if(MTR_move_absolute_angle_async(plat_handle, -800, portMAX_DELAY, &platHandle) != kStatus_Success)
	    {
	    	return kStatus_Fail;
	    }

	    CHD_CmdHandle_t magHandle = NULL;
	    if(MTR_move_absolute_angle_async(mag_handle, -3000, portMAX_DELAY, &magHandle) != kStatus_Success)
	    {
	    	return kStatus_Fail;
	    }

	    if(INITi_wait_and_release(magHandle, portMAX_DELAY) != kStatus_Success)
	    {
	    	return kStatus_Fail;
	    }
	    if(INITi_wait_and_release(platHandle, portMAX_DELAY) != kStatus_Success)
	    {
	    	return kStatus_Fail;
	    }

	    if(MTR_move_angle_async(mag_handle, -100, portMAX_DELAY, &magHandle) != kStatus_Success)
	    {
	    	return kStatus_Fail;
	    }
	    if(INITi_wait_and_release(magHandle, portMAX_DELAY) != kStatus_Success)
	    {
	    	return kStatus_Fail;
	    }

	    if(MHM_home_left_arm(portMAX_DELAY) != kStatus_Success)
	    {
	    	return kStatus_Fail;
	    }
	    if(MTR_move_absolute_angle_async(l_arm_handle, 45, portMAX_DELAY, &moveHandle) != kStatus_Success)
	    {
	    	return kStatus_Fail;
	    }
	    if(INITi_wait_and_release(moveHandle, portMAX_DELAY) != kStatus_Success)
	    {
	    	return kStatus_Fail;
	    }

	    if(MHM_home_right_arm(portMAX_DELAY) != kStatus_Success)
	    {
	    	return kStatus_Fail;
	    }
		if(MTR_move_absolute_angle_async(r_arm_handle, 135, portMAX_DELAY, &moveHandle) != kStatus_Success)
	    {
	    	return kStatus_Fail;
	    }
	    if(INITi_wait_and_release(moveHandle, portMAX_DELAY) != kStatus_Success)
	    {
	    	return kStatus_Fail;
	    }

	    vTaskDelay(pdMS_TO_TICKS(100));

	    SK_Point_t startPoint = {.x = 30, .y = 130};
	    if(SK_move_to_xy_async(startPoint, 0, false, portMAX_DELAY, &moveHandle) != kStatus_Success)
	    {
	    	return kStatus_Fail;
	    }
	    if(INITi_wait_and_release(moveHandle, portMAX_DELAY) != kStatus_Success)
	    {
	    	return kStatus_Fail;
	    }

	    initialized = true;
		return kStatus_Success;
}

status_t INIT_move_to_start_point(void){

    CHD_CmdHandle_t moveHandle = NULL;

    SK_Point_t startPoint = {.x = 30, .y = 130};
    if(SK_move_to_xy_async(startPoint, 0, false, portMAX_DELAY, &moveHandle) != kStatus_Success)
    {
    	return kStatus_Fail;
    }
    if(INITi_wait_and_release(moveHandle, portMAX_DELAY) != kStatus_Success)
    {
    	return kStatus_Fail;
    }
}

void INIT_SetUnitialized(void){
	initialized = false;
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/

static status_t INITi_init_motor(MTR_MotorConfig_t config)
{
    char            logMsg[60];
    TickType_t      deadline  = SYW_deadline_from_timeout_ms(400);
    CHD_CmdHandle_t cmdHandle = NULL;
    if (MTR_init_handle_async(config, deadline, &cmdHandle) == kStatus_Success &&
        SYW_cmd_wait_result(cmdHandle, deadline, NULL) == kStatus_Success)
    {
        CHD_remove_cmd_handle_ref(cmdHandle);
        snprintf(logMsg, sizeof(logMsg),
                 ""
                 "[%s] Motor init completed",
                 config.label);
        LOG_INFO(logMsg);
        return kStatus_Success;
    }
    else
    {
        CHD_remove_cmd_handle_ref(cmdHandle);
        snprintf(logMsg, sizeof(logMsg), "[%s] Motor init failed or timed out", config.label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }
}

static status_t INITi_wait_and_release(CHD_CmdHandle_t cmdHandle, TickType_t deadline)
{
    if (cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    status_t status = SYW_cmd_wait_result(cmdHandle, deadline, NULL);
    CHD_remove_cmd_handle_ref(cmdHandle);
    return status;
}
