/************************************************************
 * @file    uart2.c
 * @brief   UART2 wrapper implementation using RTOS helpers
 *
 * Small buffered UART2 implementation using the RTOS-adapter
 * (UART_RTOS_Send / UART_RTOS_Receive). Provides simple
 * helpers for sending characters, strings and reading lines.
 *
 * Created on: 06.05.2026
 * Author: BSc
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include "peripherals.h"
#include "uart.h"
#include <string.h>

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

/**
 * @brief Writes one byte into the transmit buffer.
 *
 * Switching on the TIE interrupt causes an interrupt to be
 * triggered immediately. The function blocks until there is
 * space in the tx buffer queue.
 *
 * @param[in] ch Byte to send
 */
void uart2WriteChar(char ch)
{
    UART_RTOS_Send(&UART2_rtos_handle, (uint8_t*)&ch, 1);
}

/**
 * @brief Write a NUL-terminated string to the UART.
 *
 * If the string is empty, the function returns immediately.
 *
 * @param[in] str NUL-terminated string to send
 */
void uart2Write(const char* str)
{
    size_t len = strlen(str);
    if (len > 0)
    {
        UART_RTOS_Send(&UART2_rtos_handle, (uint8_t*)str, len);
    }
}

/**
 * @brief Write a NUL-terminated string followed by `NEW_LINE`.
 *
 * Convenience wrapper that appends `NEW_LINE` after the provided string.
 *
 * @param[in] str NUL-terminated string to send
 */
void uart2WriteLine(const char* str)
{
    uart2Write(str);
    uart2WriteChar(NEW_LINE);
}

/**
 * @brief Read a single character from the UART receive buffer.
 *
 * Blocks until a character is available.
 *
 * @return Received character
 */
char uart2ReadChar(void)
{
    char   ch       = '\0';
    size_t received = 0;
    UART_RTOS_Receive(&UART2_rtos_handle, (uint8_t*)&ch, 1, &received, portMAX_DELAY);

    return ch;
}

/**
 * @brief Read a line into the provided buffer.
 *
 * Blocks until a newline (`NEW_LINE`) is received or the buffer length
 * is exceeded. The newline character is replaced with a terminating NUL.
 *
 * @param[out] str Buffer to receive the line (must be at least `length` bytes)
 * @param[in] length Size of `str` in bytes
 * @return Number of bytes written (excluding terminating NUL)
 */
uint16_t uart2ReadLine(char* str, uint16_t length)
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

/**
 * @brief Simple character validation helper used by the parser.
 *
 * @param ch Character to validate
 * @return true if the character is considered a valid command start
 */
bool uart2IsValidCommand(int8_t ch)
{
    if (ch <= 10)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/*******************************************
 *     Private Function Implementations    *
 *******************************************/
