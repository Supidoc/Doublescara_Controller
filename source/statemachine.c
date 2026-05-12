/*
 * statemachine.c
 *
 *  Created on: 21.04.2026
 *      Author: BSc
 */

#include <stdio.h>
#include "FreeRTOS.h"
#include "log.h"
#include "pca9555a.h"
#include "statemachine_helpers.h"
#include "statemachine.h"
#include "display.h"
#include "uart.h"
#include "log.h"

static State state = ERROR;

static CommandPackage currentpkg;

void Statemachine_task(void *pvParameters){
	DISPLAY_write("press start to solve the puzzle!");
	uint8_t pinvalue;
	currentpkg.transID = 0;
	currentpkg.cmd = 0;
	bool initialized = false;

	while(!initialized){
		//If Start Button pressed
		if (PCA_Read_Pin(1, 2, &pinvalue) == kStatus_Success){
			if (pinvalue == 1){
				uart2WriteLine("START");
				DISPLAY_write("idle");
				// wait for raspi response "OK" (time >3s == ERROR)
				char response[3] = {0};
				size_t received = 0;
			    status_t            status;
			    status = UART_RTOS_Receive(&UART2_rtos_handle, (uint8_t *)response, sizeof(response), &received, 1000);
				response[sizeof(response) - 1] = '\0';
				if(received != sizeof(response))
				{
					DISPLAY_write("Init failed");
					continue;
				}else{
					if (strcmp(response, "OK") == 0) {
						state = IDLE;
	                		initialized = true;
	                		break;
	                	}
					else {
						// wrong answer, wait for a new response
						//uart2WriteLine("ERROR");
						DISPLAY_write("Init failed");
						continue;
					}
				}

				}
			}
		// PCA error
		else {
			state = ERROR;
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}

	LOG_INFO("Communications Initialized");
	// statemachine
    while(1){
    	switch(state){
        	case IDLE:
			{
				char receivedLine[32];
				uint16_t index = 0;
				char receivedChar = '\0';
				size_t received = 0;
			    status_t            status;

				while (1)
				{
					status = UART_RTOS_Receive(&UART2_rtos_handle, (uint8_t *)&receivedChar, 1, &received, 1000);
					if (received  != 1)
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

				if(ReadCmd(&currentpkg, receivedLine)){
					state = EXECUTE;
					char buffer[32];
					snprintf(buffer, sizeof(buffer), "%s running", CommandToString(currentpkg.cmd));
					DISPLAY_write(buffer);
				}
				else{
					state = ERROR;
				}
				vTaskDelay(pdMS_TO_TICKS(10));
				break;
			}
            case EXECUTE:
            	if (GPIO_PinRead(GPIOB, 18)){	// read nothalt
            		state = ERROR;
            		break;
            	}
            	if (ExecuteCmd(currentpkg)){
                    state = DONE;
                }
                else{
                    state = ERROR;
                }
                break;
            case DONE:
                SendOK(currentpkg.transID);
                state = IDLE;
    			DISPLAY_write("idle");
                break;
            case ERROR:
            	// cmd nullung
                SendError(currentpkg.transID);
                char buffer[16];
                snprintf(buffer, sizeof(buffer), "error at %s, press start to retry", CommandToString(currentpkg.cmd));
                DISPLAY_write(buffer);
                // push start to continue
                while(1){
                if (PCA_Read_Pin(1, 2, &pinvalue) == kStatus_Success){
                	if (pinvalue == 1){
                		state = IDLE;
                		break;
                	}
                }
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
    }
}
