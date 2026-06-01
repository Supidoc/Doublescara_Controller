/************************************************************
 * @file    uart.h
 * @brief   UART2 serial interface (small buffered wrapper)
 *
 * Lightweight UART2 wrapper for simple transmit/receive operations
 * with small software buffers. Provides convenience helpers for
 * writing strings and lines, and exposes the IRQ handlers to be
 * linked to the vector table.
 *
 * The implementation is responsible for configuring the peripheral
 * and the IRQs. Buffer sizes can be overridden by defining the
 * configurable macros before including this header.
 *
 * @author  dg
 * @date    20 May 2026
 ************************************************************/

/**
 * @defgroup UART2_Module UART2 Wrapper
 * @brief   UART2 serial helper functions and IRQ prototypes
 * @{
 */

#ifndef UART2_H
#define UART2_H

#ifdef __cplusplus
extern "C"
{
#endif

    /********************
     *     Includes    *
     ********************/

#include <stdint.h>
#include <stdbool.h>

    /***********************************
     *     Public Macros / Defines     *
     ***********************************/

/** Size of the UART RX software buffer. Can be overridden at compile time. */
#ifndef UART2_RX_BUF_SIZE
#define UART2_RX_BUF_SIZE 128
#endif

/** Size of the UART TX software buffer. Can be overridden at compile time. */
#ifndef UART2_TX_BUF_SIZE
#define UART2_TX_BUF_SIZE 128
#endif

/** Character used as newline for convenience helpers. */
#ifndef NEW_LINE
#define NEW_LINE '\n'
#endif

    /***************************
     *     Public Typedefs     *
     ***************************/

    /****************************
     *     Public Variables     *
     ****************************/

    /**************************************
     *     Public Function Prototypes    *
     **************************************/

    /**
     * @brief Validate whether a received character is a valid command start.
     *
     * This helper is typically used by the receive parser to determine if the
     * incoming byte should be treated as the beginning of a command.
     *
     * @param ch Received character (signed 8-bit to match internal parsing code)
     * @return true if the character can start a command, false otherwise
     */
    bool uart2IsValidCommand(int8_t ch);

    /**
     * @brief Write a single character to the UART transmit buffer.
     *
     * This function may block briefly if the software buffer is full.
     *
     * @param ch Character to send
     */
    void uart2WriteChar(char ch);

    /**
     * @brief Write a NUL-terminated string to the UART.
     *
     * @param str NUL-terminated string to send
     */
    void uart2Write(const char* str);

    /**
     * @brief Write a NUL-terminated string followed by the newline character.
     *
     * Convenience wrapper that appends `NEW_LINE` after the provided string.
     *
     * @param str NUL-terminated string to send
     */
    void uart2WriteLine(const char* str);

    /**
     * @brief Read a single character from the UART receive buffer.
     *
     * @return Received character (or 0 if none available depending on implementation)
     */
    char uart2ReadChar(void);

    /**
     * @brief Read a line into the provided buffer.
     *
     * Copies up to `length - 1` characters into `str` and NUL-terminates the
     * result. The exact behavior (blocking or non-blocking) depends on the
     * implementation.
     *
     * @param[out] str Buffer to receive the line
     * @param length Size of the buffer in bytes
     * @return Number of bytes written (excluding terminating NUL)
     */
    uint16_t uart2ReadLine(char* str, uint16_t length);

    /**
     * @brief UART2 RX/TX IRQ handler.
     *
     * Entry point for the UART2 RX/TX interrupt. Link this symbol to the
     * vector table so the driver can handle transmit and receive events.
     */
    void UART2_RX_TX_IRQHandler(void);

    /**
     * @brief UART2 error IRQ handler.
     *
     * Entry point for UART2 error interrupts (overrun, framing, parity).
     */
    void UART2_ERR_IRQHandler(void);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* UART2_H */
