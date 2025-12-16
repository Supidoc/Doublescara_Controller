/*
 * UART1.c
 *
 *  Created on: 5 Dec 2025
 *      Author: dg
 */
#include "peripherals.h"

struct UART_Interface {
    void (*init)(void);
    void (*deinit)(void);
    void (*transmit)(uint8_t *pData, uint16_t size);
    void (*setTransmitCmpltCB)(void (*transmitCmpltCB)(void));
    void (*setReceiveCB)(void (*receiveCB)(void));
    const char* name;
};

uart_handle_t uart1_handle;


void UART1_Init(void){

}

void UART1_Transmit(void){

}

void UART1_SetTransmitCmpltCB(void (*transmitCmpltCB)(void)){

}

void UART1_SetReceiveCB(void (*receiveCB)(void)){


}


