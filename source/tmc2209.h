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
 *
 * This module provides an interface for initializing TMC2209 driver handles, configuring
 * microstepping and current settings, and converting desired current values to appropriate register
 * divider settings. It uses a command queue system to execute operations in the context of a
 * dedicated TMC task, allowing for asynchronous configuration with deadline-based completion.
 *
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
#include "task_helpers.h"

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
 * @brief Enumeration of TMC2209 microstepping resolutions. The value corresponds to the register
 * setting for microstepping configuration.
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

/**
 * @brief Initializes the TMC2209 module.
 *
 * This function initializes the TMC2209 module. It must be called before using any other
 * functions in this module.
 *
 * @note This function is NOT task-safe and must be called during system initialization before any
 * tasks start.
 * @note The TMC_task() should be started after this function completes successfully.
 *
 * @return kStatus_Success if the initialization is successful.
 *         kStatus_Fail if the initialization fails.
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
 * @brief Retrieves the default configuration for a TMC2209 driver handle.
 *
 * This function populates a TMC_Config_t structure with default values for initializing a TMC2209
 * driver handle.
 *
 * @param[out] config Pointer to a TMC_Config_t structure to be filled with default configuration
 * values. Must not be NULL.
 *
 * @return kStatus_Success if the default configuration was successfully retrieved and stored in the
 * provided structure. kStatus_Fail if the provided pointer is NULL or an error occurs while
 * retrieving
 */
status_t TMC_get_default_config(TMC_Config_t* config);

/**
 * @brief Initializes a TMC2209 handle with default configuration asynchronously.
 *
 * This function queues a default initialization command for a TMC2209 driver handle.
 * The initialization is performed asynchronously by the TMC_task.
 * The caller can await completion using the returned command handle.
 *
 * @param[in] config  TMC config structure to initialize the handle with.
 * @param[in] deadline Deadline for the initialization command.
 * @param[out] cmdHandle Pointer to receive command handle for awaiting completion.
 *
 * @return kStatus_Success if the command is successfully queued.
 *         kStatus_Fail if the queue is full, cmdHandle is NULL, or an error occurs.
 *
 * @see THE_cmd_wait_result()
 */
status_t TMC_init_handle_async(TMC_Config_t config, TickType_t deadline,
                               THE_CmdHandle_t* cmdHandle);

/**
 * @brief Retrieves a TMC2209 handle by its label.
 *
 * This function searches for a TMC2209 handle with the specified label and returns it if found.
 *
 * @param[in] label The label of the TMC2209 handle to retrieve. Must not be NULL.
 * @param[out] handle Pointer to a TMC_Handle_t variable where the found handle will be stored. Must
 * not be NULL.
 *
 * @return kStatus_Success if a handle with the specified label is found and returned.
 *         kStatus_Fail if the label is NULL, the handle pointer is NULL, or no matching handle is
 * found.
 */
status_t TMC_get_handle_by_label(const char* label, TMC_Handle_t* handle);

/**
 * @brief Sets the microstepping resolution for a TMC2209 driver asynchronously.
 *
 * This function queues a microstepping configuration command for a TMC2209 driver handle.
 * The operation is performed asynchronously by the TMC_task.
 * The caller can await completion using the returned command handle.
 *
 * @param[in] handle Handle for the TMC2209 driver to configure.
 * @param[in] microstepping The microstepping resolution to set (TMC_MICROSTEPPING_t).
 * @param[in] deadline Deadline for the operation.
 * @param[out] cmdHandle Pointer to receive command handle for awaiting completion.
 *
 * @return kStatus_Success if the command is successfully queued.
 *         kStatus_Fail if the queue is full, handle is NULL, cmdHandle is NULL, or an error occurs.
 *
 * @see TMC_MICROSTEPPING_t
 */
status_t TMC_set_microstepping_async(TMC_Handle_t handle, TMC_MICROSTEPPING_t microstepping,
                                     TickType_t deadline, THE_CmdHandle_t* cmdHandle);

/**
 * @brief Converts a uint8_t microstepping value to the corresponding enum member.
 *
 * This function converts a raw uint8_t microstepping value to the corresponding
 * TMC_MICROSTEPPING_t enum value.
 *
 * @param[in] value The uint16_t microstepping value to convert.
 * @param[out] microstepping Pointer to store the converted enum value. Must not be NULL.
 *
 * @return kStatus_Success if the value matches a valid enum member.
 *         kStatus_Fail if the value does not correspond to any enum member.
 *
 * @see TMC_MICROSTEPPING_t
 */
status_t TMC_microstepping_uint_to_enum(uint16_t value, TMC_MICROSTEPPING_t* microstepping);

/**
 * @brief Sets the IHOLD current divider asynchronously.
 *
 * Queues a command to set the hold current divider value.
 * The operation is executed in the TMC task context.
 * The caller can await completion using the returned command handle.
 *
 * @param[in] handle Handle for the TMC2209 driver to configure.
 * @param[in] ihold The hold current divider value (0-31).
 * @param[in] deadline Deadline for the operation.
 * @param[out] cmdHandle Pointer to receive command handle for awaiting completion.
 *
 * @return kStatus_Success if command was successfully queued.
 *         kStatus_Fail if queue operation failed, handle is NULL, or cmdHandle is NULL.
 */
status_t TMC_set_ihold_divider_async(TMC_Handle_t handle, uint8_t ihold, TickType_t deadline,
                                     THE_CmdHandle_t* cmdHandle);

/**
 * @brief Sets the IRUN current divider asynchronously.
 *
 * Queues a command to set the run current divider value.
 * The operation is executed in the TMC task context.
 * The caller can await completion using the returned command handle.
 *
 * @param[in] handle Handle for the TMC2209 driver to configure.
 * @param[in] irun The run current divider value (0-31).
 * @param[in] deadline Deadline for the operation.
 * @param[out] cmdHandle Pointer to receive command handle for awaiting completion.
 *
 * @return kStatus_Success if command was successfully queued.
 *         kStatus_Fail if queue operation failed, handle is NULL, or cmdHandle is NULL.
 */
status_t TMC_set_irun_divider_async(TMC_Handle_t handle, uint8_t irun, TickType_t deadline,
                                    THE_CmdHandle_t* cmdHandle);

/**
 * @brief Converts a desired current to a TMC2209 current divider value.
 *
 * Calculates the appropriate current divider (0-31) that achieves a desired output current
 * based on a reference current. Uses the formula:
 *       divider = (desired_current * 32 / reference_current) - 1
 *
 * The forumla for the reference current is derived from the TMC2209 datasheet (Rev. 1.09 p. 53)
 * and results in a current of approximately 1.77A at 100% (divider=0).
 *
 * The result is rounded according to the specified rounding mode to ensure valid divider values.
 * Dividers are limited to the valid range [0, 31] as per TMC2209 register bit width.
 *
 * @param[in] desired_current The target output current in milliamps.
 * @param[in] rounding Rounding mode for the divider calculation (FLOOR, CEIL, or NEAREST).
 * @param[out] divider Pointer to store the calculated divider value (0-31). Must not be NULL.
 *
 * @return kStatus_Success if the conversion succeeded and divider is valid (0-31).
 *         kStatus_Fail if divider parameter is NULL, currents are invalid (zero or negative, or
 * exceed reference current), or the calculated divider exceeds valid range.
 *
 * @note Divider values outside the valid range [0, 31] will cause the function to return
 * kStatus_Fail.
 * @note Higher divider values result in lower output currents.
 *
 * @see TMC_RoundingMode_t, TMC_set_ihold_divider(), TMC_set_irun_divider()
 */
status_t TMC_current_to_divider(float desired_current, TMC_RoundingMode_t rounding,
                                uint8_t* divider);

/** @} */ // End of TMC2209_Module

#endif /* TMC2209_H_ */
