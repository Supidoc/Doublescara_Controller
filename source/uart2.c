/*
 * uart2.c
 *
 *  Created on: 06.05.2026
 *      Author: BSc
 */

#include "peripherals.h"
#include "uart.h"



/**
* Writes one Byte in the transmit buffer.
*
* @details
*   Switching on the TIE interrupt causes an interrupt to be
*   triggered immediately. The function blocks until there is
*   space in the txBuf queue.
*
* @param[in] ch
*   the byte to send
*/
void uart2WriteChar(char ch)
{
  UART_RTOS_Send(&UART2_rtos_handle, (uint8_t *)&ch, 1);
}

/**
* Writes a null terminated string in the send buffer. If the
* string is null, the function returns immediately.
*
* @param[in] str
*   the null terminated string to send
*/
void uart2Write(const char *str)
{
size_t len =	strlen(str);
if(len > 0){
	UART_RTOS_Send(&UART2_rtos_handle, (uint8_t *)str, len);
}
}

/**
* Writes a null terminated string in the send buffer. If the
* string is null, only a new line character is sent.
*
* @param[in] str
*   the null terminated string to send
*/
void uart2WriteLine(const char *str)
{
  uart2Write(str);
  uart2WriteChar(NEW_LINE);
}

/**
* Reads one char out of the rxBuf queue. The function blocks
* until there is at least one byte in the queue.
*
* @return
*   the received byte
*/
char uart2ReadChar(void)
{
	  char ch = '\0';
	  size_t received = 0;
	UART_RTOS_Receive(&UART2_rtos_handle,(uint8_t *) &ch, 1, &received, portMAX_DELAY);

  return ch;
}

/**
* Reads a null terminated string out of the rxBuf queue. The
* function blocks until a new line character has been received
* or the length has been exceeded.
*
* @details
*   the new line character will be replaced with a '\0' to
*   terminate the string.
*
* @param[out] *str
*   pointer to a char array to store the received string
* @param[in] length
*   the length of the str char array.
* @returns
*   the length of the received string.
*/
uint16_t uart2ReadLine(char *str, uint16_t length)
{
  uint16_t i;
  for (i = 1; i < length; i++)
  {
    *str = uart2ReadChar();
    if (*str == NEW_LINE)
    {
      *str = '\0';
      break;
    }
    str++;
  }
  return i;
}


bool uart2IsValidCommand(int8_t ch){
  if (ch <= 10){
    return true;
  }
  else
    return false;
}
