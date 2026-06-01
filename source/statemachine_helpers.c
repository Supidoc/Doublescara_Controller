/**
 * @file statemachine_helpers.c
 * @brief Command parsing and helper implementations for the state machine
 *
 * Implements command execution, parsing and simple display/log helpers
 * used by the statemachine task.
 *
 * Author: BSc
 * Date:   18 Apr 2026
 */

/********************
 *     Includes    *
 ********************/
#include <display/McuFontCour10Normal.h>
#include <display/McuFontDisplay.h>
#include <display/McuGDisplaySSD1306.h>
#include <motion/grabber/grabber.h>
#include <motion/scara_kinematics/scara_kinematics.h>
#include "statemachine_helpers.h"
#include "led.h"
#include "pin_mux.h"
#include "uart.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "cmd_handle.h"
#include "sync_wait.h"
#include "init.h"
#include "display_helpers.h"
#include "pca9555a.h"

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

bool ExecuteCmd(CommandPackage pkg)
{
    switch (pkg.cmd)
    {
        case INIT:
             if (INIT_init_robot() != kStatus_Success)
            {
                return false;
            }

             Show2Liner("Robot is Ready", "Press Start");

             uint8_t pinvalue;
             while (1)
             					{
             						if (PCA_Read_Pin(PCA_PORT_1, 2, &pinvalue) == kStatus_Success)
             						{
             							if (pinvalue != 0)
             							{
             								break;
             							}
             						}
             						vTaskDelay(pdMS_TO_TICKS(100));
             					}
             Show2Liner("Taking Photo", "Finding Solution");

            return true;
        case LED_ON:
            LED_on();
            return true;
        case LED_OFF:
            LED_off();
            return true;
        case GOTO_XY_ROTATE:
        {
            CHD_CmdHandle_t cmdHandle   = NULL;
            SK_Point_t      targetPoint = {.x = pkg.x, .y = pkg.y};
            if (SK_move_to_xy_async(targetPoint, pkg.rotate, true, portMAX_DELAY, &cmdHandle) !=
                kStatus_Success)
            {
                return false;
            }
            if (SYW_cmd_wait_result(cmdHandle, portMAX_DELAY, NULL) != kStatus_Success)
            {
                return false;
            }
            return true;
        }
        case MAGNET_UP:
            if (GRB_magnet_up() != kStatus_Success)
            {
                return false;
            }
            return true;
        case MAGNET_DOWN:
            if (GRB_magnet_down() != kStatus_Success)
            {
                return false;
            }
            return true;
        case Z_UP:
            if (GRB_platform_up() != kStatus_Success)
            {
                return false;
            }
            return true;
        case Z_DOWN:
            if (GRB_platform_down() != kStatus_Success)
            {
                return false;
            }
            return true;
        case GRAB:
            if (GRB_grab() != kStatus_Success)
            {
                return false;
            }
            return true;
        case RELEASE:
            if (GRB_release() != kStatus_Success)
            {
                return false;
            }
            return true;
        case ERR:
            return false;
        case FINISH:

        	Show2Liner("Finished","");
            INIT_move_to_start_point();
        	Show2Liner("Finished", "Press Start");


        	                while (1)
        	                					{
        	                						if (PCA_Read_Pin(PCA_PORT_1, 2, &pinvalue) == kStatus_Success)
        	                						{
        	                							if (pinvalue != 0)
        	                							{
        	                								break;
        	                							}
        	                						}
        	                						vTaskDelay(pdMS_TO_TICKS(100));
        	                					}
        	                return true;
        	                break;
        default:
            return false;
    }
    return true;
}

bool ReadCmd(CommandPackage* pkg, const char* receivedMessage)
{
    size_t length = strlen(receivedMessage);

    /* convert into 5 fields: cmd,x,y,rotate,transid */
    char* fields[5];
    int   fieldCount = 0;
    char* token      = strtok((char*)receivedMessage, ",");

    while (token != NULL && fieldCount < 5)
    {
        fields[fieldCount++] = token;
        token                = strtok(NULL, ",");
    }

    if (fieldCount != 5)
    {
        return false;
    }

    /* parse fields */
    pkg->cmd     = (uint8_t)atoi(fields[0]);
    pkg->x       = atof(fields[1]);
    pkg->y       = atof(fields[2]);
    pkg->rotate  = atof(fields[3]);
    pkg->transID = (uint8_t)atoi(fields[4]);

    return uart2IsValidCommand(pkg->cmd);
}

void SendOK(uint8_t transID)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "OK,%d", transID);
    uart2WriteLine(buf);
}

void SendError(uint8_t transID)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "ERROR,%d", transID);
    uart2WriteLine(buf);
}

const char* CommandToString(uint8_t cmd)
{
    switch (cmd)
    {
        case INIT:
            return "INIT";
        case LED_ON:
            return "LED_ON";
        case LED_OFF:
            return "LED_OFF";
        case GOTO_XY_ROTATE:
            return "GOTO_XY_ROTATE";
        case Z_UP:
            return "Z_UP";
        case Z_DOWN:
            return "Z_DOWN";
        case MAGNET_UP:
            return "MAGNET_UP";
        case MAGNET_DOWN:
            return "MAGNET_DOWN";
        case ERR:
            return "ERROR";
        case FINISH:
            return "FINISH";
        case GRAB:
            return "GRAB";
        case RELEASE:
            return "RELEASE";
        default:
            return "UNKNOWN";
    }
}


