/************************************************************
 * @file    tmc2209_shared.h
 * @brief   Shared TMC2209 types, constants, and handle declarations.
 * @author  dg
 * @date    14 Apr 2026
 ************************************************************/

#ifndef TMC2209_SHARED_H_
#define TMC2209_SHARED_H_

#ifdef __cplusplus
extern "C"
{
#endif

/********************
 *     Includes    *
 ********************/
#include <stdint.h>
#include "fsl_uart_freertos.h"

/***********************************
 *     Public Macros / Defines     *
 ***********************************/

/**
 * @brief Enable write confirmation for TMC2209 configuration commands.
 */
#define TMC_CONFIRM_WRITES (1)

/**
 * @brief Maximum length of the TMC command queue.
 */
#define TMC_COMMAND_QUEUE_LENGTH 10

/**
 * @brief Maximum number of TMC2209 handles that can be created.
 */
#define TMC_MAX_HANDLE_COUNT 5

/** @brief Maximum number of TMC command handles available. */
#define TMC_MAX_CMD_HANDLE_COUNT 20

    /***************************
     *     Public Typedefs     *
     ***************************/

    /**
     * @brief Enumeration of TMC2209 serial interface addresses.
     */
    typedef enum _TMC_SerialAddress
    {
        /** @brief Serial address 0 */
        TMC_SERIAL_ADDRESS_0 = 0,
        /** @brief Serial address 1 */
        TMC_SERIAL_ADDRESS_1 = 1,
        /** @brief Serial address 2 */
        TMC_SERIAL_ADDRESS_2 = 2,
        /** @brief Serial address 3 */
        TMC_SERIAL_ADDRESS_3 = 3,
    } TMC_SerialAddress_t;

    /**
     * @brief Enumeration of TMC2209 microstepping resolutions. The value corresponds to the
     * register setting for microstepping configuration.
     */
    typedef enum _TMC_MICROSTEPPING
    {
        /** @brief 256 microsteps per full step */
        TMC_MS_256 = 0b0000,
        /** @brief 128 microsteps per full step */
        TMC_MS_128 = 0b0001,
        /** @brief 64 microsteps per full step */
        TMC_MS_64 = 0b00010,
        /** @brief 32 microsteps per full step */
        TMC_MS_32 = 0b0011,
        /** @brief 16 microsteps per full step */
        TMC_MS_16 = 0b0100,
        /** @brief 8 microsteps per full step */
        TMC_MS_8 = 0b0101,
        /** @brief 4 microsteps per full step */
        TMC_MS_4 = 0b0110,
        /** @brief 2 microsteps per full step */
        TMC_MS_2 = 0b0111,
        /** @brief Full step */
        TMC_MS_FULL = 0b1000
    } TMC_MICROSTEPPING_t;

    /**
     * @brief Rounding modes for current to divider conversion.
     *
     * Controls how the calculated divider is rounded when converting from current values.
     */
    typedef enum _TMC_RoundingMode
    {
        /** @brief Round down to nearest integer */
        TMC_ROUND_FLOOR,
        /** @brief Round up to nearest integer */
        TMC_ROUND_CEIL,
        /** @brief Round to nearest integer */
        TMC_ROUND_NEAREST
    } TMC_RoundingMode_t;

    /**
     * @brief Opaque pointer to the TMC2209 handle implementation
     *
     */
    typedef struct _TMC_HandleImpl* TMC_Handle_t;

    /**
     * @brief Configuration structure for initializing a TMC2209 driver handle.
     *
     * This structure contains all necessary parameters for configuring a TMC2209 driver,
     * including UART communication handles, device address, and default settings.
     */
    typedef struct _TMC_Config
    {
        /** @brief Pointer to the RTOS-aware UART handle for thread-safe communication */
        uart_rtos_handle_t* uartRTOSHandle;
        /** @brief Pointer to the low-level UART driver handle */
        uart_handle_t* uartHandle;
        /** @brief Serial address of this TMC2209 device (0-3) */
        TMC_SerialAddress_t serialAdress;
        /** @brief Descriptive label for this motor driver (e.g., "X-Axis", "Y-Axis") */
        const char* label;
        /** @brief Default microstepping configuration */
        TMC_MICROSTEPPING_t microstepping;
        /** @brief Default IHOLD divider (0-31) */
        uint8_t iholdDivider;
        /** @brief Default IRUN divider (0-31) */
        uint8_t irunDivider;
    } TMC_Config_t;

    /****************************
     *     Public Variables     *
     ****************************/

    /**************************************
     *     Public Function Prototypes    *
     **************************************/

#ifdef __cplusplus
}
#endif

#endif // TMC2209_SHARED_H_
