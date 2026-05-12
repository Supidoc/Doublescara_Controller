#ifndef UART2_H
#define UART2_H

#include <stdint.h>
#include <stdbool.h>

/* === Konfiguration (falls nicht global definiert) === */
#ifndef UART2_RX_BUF_SIZE
#define UART2_RX_BUF_SIZE 128
#endif

#ifndef UART2_TX_BUF_SIZE
#define UART2_TX_BUF_SIZE 128
#endif

#ifndef NEW_LINE
#define NEW_LINE '\n'
#endif

/* === Funktionsprototypen === */

/* Initialisierung */
bool uart2IsValidCommand(int8_t ch);

/* Senden */
void uart2WriteChar(char ch);
void uart2Write(const char *str);
void uart2WriteLine(const char *str);

/* Empfangen */
char uart2ReadChar(void);
uint16_t uart2ReadLine(char *str, uint16_t length);

/* Interrupt-Handler */
void UART2_RX_TX_IRQHandler(void);
void UART2_ERR_IRQHandler(void);

#endif /* UART2_H */
