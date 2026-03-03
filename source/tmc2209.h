/************************************************************
 * @file    tmc2209.h
 * @brief   TMC2209 stepper motor driver interface module
 *
 * This module provides functionality for communicating with and controlling
 * TMC2209 stepper motor drivers via UART. It supports configuration, status
 * monitoring, and command queuing for multiple motor drives.
 *
 * @author  dg
 * @date    25 Feb 2026
 ************************************************************/

/**
 * @defgroup TMC2209_Module TMC2209 Stepper Motor Driver
 * @brief   Functions and types for communicating with and controlling TMC2209 stepper motor drivers
 * @{
 */

#ifndef TMC2209_H_
#define TMC2209_H_

/********************
 *     Includes    *
 ********************/

#include "fsl_common.h"
#include "peripherals.h"
#include "stdint.h"

/***********************************
 *     Public Macros / Defines     *
 ***********************************/

/**
 * @brief Timeout duration for TMC2209 communication in milliseconds.
 */
#define TMC_TIMEOUT_MS 1000

/**
 * @brief Enable write confirmation for TMC2209 configuration commands.
 */
#define TMC_CONFIRM_WRITES (1)

/**
 * @brief Maximum length of the TMC command queue.
 */
#define TMC_COMMAND_QUEUE_LENGTH 10

/***************************
 *     Public Typedefs     *
 ***************************/

/**
 * @brief Enumeration of TMC2209 serial interface addresses.
 */
typedef enum _TMC_SerialAddress
{
    /** @brief Serial address 0 - typically for first motor driver */
    TMC_SERIAL_ADDRESS_0 = 0,
    /** @brief Serial address 1 - typically for second motor driver */
    TMC_SERIAL_ADDRESS_1 = 1,
    /** @brief Serial address 2 - typically for third motor driver */
    TMC_SERIAL_ADDRESS_2 = 2,
    /** @brief Serial address 3 - typically for fourth motor driver */
    TMC_SERIAL_ADDRESS_3 = 3,
} TMC_SerialAddress_t;

/**
 * @brief Structure representing a TMC2209 write datagram for UART communication.
 *
 * This structure follows the TMC2209 UART protocol format for writing to registers.
 */
typedef struct _TMC_Write_Datagram
{
    /** @brief Synchronization byte (4 bits) - identifies datagram start */
    uint8_t sync : 4;
    /** @brief Reserved bits (4 bits) - not used in current implementation */
    uint8_t reserved : 4;
    /** @brief Device address (8 bits) - selects which TMC2209 device */
    uint8_t dev_addr : 8;
    /** @brief Register address (8 bits) - specifies target register */
    uint8_t reg_addr : 8;
    /** @brief Data to write (32 bits) - configuration or command data */
    uint32_t data : 32;
    /** @brief CRC checksum (8 bits) - ensures data integrity */
    uint8_t crc : 8;
} TMC_Write_Datagram_t;

/**
 * @brief Structure representing a TMC2209 read datagram for UART communication.
 *
 * This structure follows the TMC2209 UART protocol format for reading from registers.
 */
typedef struct _TMC_Read_Datagram
{
    /** @brief Synchronization byte (4 bits) - identifies datagram start */
    uint8_t sync : 4;
    /** @brief Reserved bits (4 bits) - not used in current implementation */
    uint8_t reserved : 4;
    /** @brief Device address (8 bits) - selects which TMC2209 device */
    uint8_t dev_addr : 8;
    /** @brief Register address (8 bits) - specifies source register */
    uint8_t reg_addr : 8;
    /** @brief CRC checksum (8 bits) - ensures data integrity */
    uint8_t crc : 8;
} TMC_Read_Datagram_t;

/**
 * @brief Handle structure for managing a TMC2209 motor driver instance.
 *
 * Contains all necessary information to communicate with and control a single
 * TMC2209 motor driver connected via UART.
 */
typedef struct _TMC_Handle
{
    /** @brief Pointer to the RTOS-aware UART handle for thread-safe communication */
    uart_rtos_handle_t* uartRTOSHandle;
    /** @brief Pointer to the low-level UART driver handle */
    uart_handle_t* uartHandle;
    /** @brief Serial address of this TMC2209 device (0-3) */
    TMC_SerialAddress_t serialAdress;
    /** @brief Count of write operations */
    uint8_t transmissionCount;
    /** @brief Descriptive label for this motor driver (e.g., "X-Axis", "Y-Axis") */
    const char* label;
} TMC_Handle_t;

typedef enum _TMC_MICROSTEPPING
{
    TMC_MS_256  = 0b0000,
    TMC_MS_128  = 0b0001,
    TMC_MS_64   = 0b00010,
    TMC_MS_32   = 0b0011,
    TMC_MS_16   = 0b0100,
    TMC_MS_8    = 0b0101,
    TMC_MS_4    = 0b0110,
    TMC_MS_2    = 0b0111,
    TMC_MS_FULL = 0b1000
} TMC_MICROSTEPPING_t;

/****************************
 *     Public Variables     *
 ****************************/

/**************************************
 *     Public Function Prototypes    *
 **************************************/

/**
 * @brief Initializes the TMC2209 module.
 *
 * This function initializes the TMC2209 communication interface and sets up
 * the motor driver for operation. It must be called before using any other
 * functions in this module.
 *
 * @note Ensure that the UART peripherals are initialized via UART_init() before calling this
 * function.
 * @note This function is NOT task-safe and must be called during system initialization before any
 * tasks start.
 * @note The TMC_task() should be started after this function completes successfully.
 *
 * @return kStatus_Success if the initialization is successful.
 *         kStatus_Fail if the initialization fails (UART setup error).
 *
 * @see TMC_task()
 */
status_t TMC_init(void);

/**
 * @brief Task function for processing TMC2209 commands.
 *
 * This function is intended to be run as a FreeRTOS task. It continuously
 * processes commands from the TMC command queue and communicates with the
 * TMC2209 motor drivers.
 *
 * @param[in] pvParameters Pointer to task parameters (not used in this implementation).
 *
 * @note This function should be started after TMC_init() completes successfully.
 * @note This task is blocked on the command queue and will wake when commands are queued.
 * @warning This function does not return; it is expected to run as a perpetual FreeRTOS task.
 *
 * @see TMC_init()
 */
void TMC_task(void* pvParameters);

/**
 * @brief Initializes a TMC2209 handle with default configuration.
 *
 * This function queues a default initialization command for a TMC2209 driver handle.
 * The initialization is performed asynchronously by the TMC_task.
 * The calling task can wait for completion using the returned taskHandle parameter.
 *
 * @param[in] handle Pointer to the TMC handle structure to initialize.
 * @param[in] taskHandle Task handle to notify when initialization completes. Can be NULL to skip
 * notification.
 *
 * @note This function queues the command and returns immediately.
 * @note The calling task should use ulTaskNotifyTake() to wait for completion.
 * @note If taskHandle is NULL, no notification will be sent.
 *
 * @return kStatus_Success if the command is successfully queued.
 *         kStatus_Fail if the queue is full or an error occurs.
 *
 * @see ulTaskNotifyTake()
 */
status_t TMC_init_default(TMC_Handle_t* handle, TaskHandle_t taskHandle);

/**
 * @brief Sets the microstepping resolution for a TMC2209 driver.
 *
 * This function queues a microstepping configuration command for a TMC2209 driver handle.
 * The microstepping is performed asynchronously by the TMC_task.
 * The calling task can wait for completion using the returned taskHandle parameter.
 *
 * @param[in] handle Pointer to the TMC handle structure.
 * @param[in] microstepping The microstepping resolution to set (TMC_MICROSTEPPING_t).
 * @param[in] taskHandle Task handle to notify when microstepping set completes. Can be NULL to skip
 * notification.
 *
 * @note This function queues the command and returns immediately.
 * @note The calling task should use ulTaskNotifyTake() to wait for completion.
 * @note If taskHandle is NULL, no notification will be sent.
 *
 * @return kStatus_Success if the command is successfully queued.
 *         kStatus_Fail if the queue is full or an error occurs.
 *
 * @see ulTaskNotifyTake()
 * @see TMC_MICROSTEPPING_t
 */
status_t TMC_set_microstepping(TMC_Handle_t* handle, TMC_MICROSTEPPING_t microstepping,
                               TaskHandle_t taskHandle);

/**
 * @brief Converts a uint8_t microstepping value to the corresponding enum member.
 *
 * This function converts a raw uint8_t microstepping value to the corresponding
 * TMC_MICROSTEPPING_t enum value.
 *
 * @param[in] value The uint8_t microstepping value to convert (0-8).
 * @param[out] microstepping Pointer to store the converted enum value. Must not be NULL.
 *
 * @return kStatus_Success if the value matches a valid enum member.
 *         kStatus_Fail if the value does not correspond to any enum member.
 *
 * @see TMC_MICROSTEPPING_t
 */
status_t TMC_microstepping_value_to_enum(uint8_t value, TMC_MICROSTEPPING_t* microstepping);

/** @} */ // End of TMC2209_Module

#endif /* TMC2209_H_ */
