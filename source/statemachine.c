/************************************************************
 * @file    statemachine.c
 * @brief   Implementation of the application state machine task
 *
 * State machine loop that receives commands, parses them and executes
 * high-level actions through helper functions. This file contains task
 * logic only; helpers are implemented in `statemachine_helpers.c`.
 *
 * Author: BSc
 * Date:   21 Apr 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include <display/McuGDisplaySSD1306.h>
#include <infrastructure/log.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "pca9555a.h"
#include "statemachine_helpers.h"
#include "statemachine.h"
#include "uart.h"
#include "estop.h"
#include "init.h"
#include "display_helpers.h"


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

static State state = ERROR;

static CommandPackage currentpkg;

static bool reset = false;

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

void Statemachine_task(void* pvParameters)
{
    McuSSD1306_Init();
    McuGDisplaySSD1306_Init();

    INIT_init_motors();
    PCA_Pin_Config_t config = {.pinDirection = PCA_PIN_INPUT, .outputLevel = 1, .polarityInversion = PCA_HIGH_IS_ONE};
    PCA_init_pin(PCA_PORT_1, 2, config);

    uint8_t pinvalue;
    currentpkg.transID = 0;
    currentpkg.cmd     = 0;
    bool initialized   = false;

    while(1){
    while (!initialized)
    {
    	if(estopPressed){
        Show2Liner("Please unpress","Emergency Stop");
    	}
    	while(estopPressed){}
    	ESTOP_ClearEstop();
                uart2WriteLine("START");

                char     response[3] = {0};
                size_t   received    = 0;
                status_t status;

                LOG_INFO("Waiting for Communications with RPI...");

                Show2Liner("Connecting","to RPI...");

                status = UART_RTOS_Receive(&UART2_rtos_handle, (uint8_t*)response, sizeof(response),
                                           &received, 5000);
                response[sizeof(response) - 1] = '\0';
                if (received != sizeof(response))
                {
                    continue;
                }
                else
                {
                    if (strcmp(response, "OK") == 0)
                    {
                        state       = IDLE;
                        initialized = true;
                        break;
                    }
                    else
                    {
                        /* wrong answer, wait for a new response */
                        continue;
                    }
                }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    LOG_INFO("Communications Initialized");
    Show1Liner("Idle");

    /* statemachine main loop */
    while (1)
    {
    	if(estopFlag){
            state = ESTOP;

    	}

    	if(!initialized){
    		break;
    	}
        switch (state)
        {
            case IDLE:
            {
                char     receivedLine[32];
                uint16_t index        = 0;
                char     receivedChar = '\0';
                size_t   received     = 0;
                status_t status;

                while (1)
                {
                	if(estopFlag){
                		break;
                	}
                    status = UART_RTOS_Receive(&UART2_rtos_handle, (uint8_t*)&receivedChar, 1,
                                               &received, 1000);
                    if (received != 1)
                    {
                        continue;
                    }

                    if (receivedChar == '\r')
                    {
                        continue;
                    }

                    if (receivedChar == '\n')
                    {
                        receivedLine[index] = '\0';
                        break;
                    }

                    if (index < sizeof(receivedLine) - 1)
                    {
                        receivedLine[index++] = receivedChar;
                    }
                }

                if(estopFlag){
                	state = ESTOP;
                	break;
                }

                if (ReadCmd(&currentpkg, receivedLine))
                {
                    state = EXECUTE;
                    char buffer[32];
                    snprintf(buffer, sizeof(buffer), "%s", CommandToString(currentpkg.cmd));

                    Show2Liner("Running:",buffer);
                }
                else
                {
                    state = ERROR;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
                break;
            }
            case EXECUTE:
                if (estopFlag == true)
                {
                    state = ESTOP;
                    break;
                }
                if (ExecuteCmd(currentpkg))
                {
                    state = DONE;
                }
                else
                {
                    state = ERROR;
                }
                break;
            case DONE:

            	state = IDLE;
               	if(currentpkg.cmd == FINISH){

                    		state = RESET;
                    	}
                SendOK(currentpkg.transID);
                break;
            case ERROR:
                /* send error and wait for start button */
                SendError(currentpkg.transID);
                {
                    char buffer[64];
                    snprintf(buffer, sizeof(buffer), "error at %s",
                             CommandToString(currentpkg.cmd));

                    Show2Liner(buffer, "Press start to reset");
                }

                while (1)
                {
                    if (PCA_Read_Pin(PCA_PORT_1, 2, &pinvalue) == kStatus_Success)
                    {
                        if (pinvalue != 0)
                        {
                            state = RESET;
                            break;
                        }
                    }
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                break;
            case ESTOP:{
                Show1Liner("Emergency Stop:");

            	 while (1)
					{
						if (PCA_Read_Pin(PCA_PORT_1, 2, &pinvalue) == kStatus_Success)
						{
							if (pinvalue != 0)
							{
								state = RESET;
								break;
							}
						}
						vTaskDelay(pdMS_TO_TICKS(100));
					}
            }
            case RESET:{
            	NVIC_SystemReset();
            	initialized = false;
            	break;
            }
        }
    }
    }
}
