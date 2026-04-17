/************************************************************
 * @file    tmc2209_internal.h
 * @brief   Internal shared types, queue payloads, and state for TMC2209 subsystem.
 * @author  dg
 * @date    14 Apr 2026
 ************************************************************/

/**
 * @defgroup TMC2209_Internal TMC2209 Internal
 * @brief Internal command payload types and shared subsystem state.
 * @ingroup TMC2209_Module
 * @{
 */

#ifndef TMC2209_INTERNAL_H_
#define TMC2209_INTERNAL_H_

/********************
 *     Includes    *
 ********************/
#include "tmc2209_shared.h"
#include "cmd_handle.h"

/***********************************
 *     Public Macros / Defines     *
 ***********************************/

/***************************
 *     Public Typedefs     *
 ***************************/

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

typedef enum _TMC_CommandType
{
    TMC_CMD_DEFAULT_INIT,
    TMC_CMD_SET_MICROSTEPPING,
    TMC_CMD_SET_IHOLD_DIVIDER,
    TMC_CMD_SET_IRUN_DIVIDER,
    TMC_CMD_ENABLE_FREEWHEELING,
    TMC_CMD_DISABLE_FREEWHEELING
} TMC_CommandType_t;

typedef struct _TMC_HandleImpl
{
    uart_rtos_handle_t* uartRTOSHandle;
    uart_handle_t*      uartHandle;
    TMC_SerialAddress_t serialAdress;
    uint8_t             transmissionCount;
    TMC_MICROSTEPPING_t microstepping;
    uint8_t             iholdDivider;
    uint8_t             irunDivider;
    uint8_t             freewheeling; /**< 1 if in freewheeling mode, 0 otherwise */
    const char*         label;
} TMC_HandleImpl_t;

typedef struct _TMC_CommandQueueItem
{
    TMC_CommandType_t commandType;
    TMC_Handle_t      handle;
    TickType_t        deadline;
    CHD_CmdHandle_t   cmdHandle; // NEW: async completion handle
    union
    {
        struct
        {
            TMC_MICROSTEPPING_t microstepping;
        } setMicrostepping;
        struct
        {
            uint8_t ihold;
        } setIholdDivider;
        struct
        {
            uint8_t irun;
        } setIrunDivider;
        struct
        {
            TMC_Config_t config;
        } initHandle;
        struct
        {
            uint8_t enabled;
        } setFreewheeling;
    } data;
} TMC_CommandQueueItem_t;

typedef struct _TMC_HandleArrayItem
{
    TMC_HandleImpl_t handle;
    uint8_t          used;
} TMC_HandleArrayItem_t;

/****************************
 *     Public Variables     *
 ****************************/

extern TMC_HandleArrayItem_t tmcHandleArray[TMC_MAX_HANDLE_COUNT];

extern QueueHandle_t tmcCmdQueue;

extern CHD_CmdHandleImpl_t tmcCmdHandles[TMC_MAX_CMD_HANDLE_COUNT];

/**************************************
 *     Public Function Prototypes    *
 **************************************/

#endif // TMC2209_INTERNAL_H_

/** @} */