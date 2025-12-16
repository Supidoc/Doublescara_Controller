/*
 * UART_Interface.h
 *
 *  Created on: 5 Dec 2025
 *      Author: dg
 */

#ifndef UART_INTERFACE_H_
#define UART_INTERFACE_H_

struct UART_Interface {
    void (*init)(void);
    void (*deinit)(void);
    void (*transmit)(uint8_t *pData, uint16_t size);
    void (*setTransmitCmpltCB)(void (*transmitCmpltCB)(void));
    void (*setReceiveCB)(void (*receiveCB)(void));
    const char* name;
};

#endif /* UART_INTERFACE_H_ */
