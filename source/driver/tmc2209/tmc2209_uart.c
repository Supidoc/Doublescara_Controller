/************************************************************
 * @file    tmc2209_uart.c
 * @brief   Filedescription
 * @author  dg
 * @date    14 Apr 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include <infrastructure/log.h>
#include "tmc2209_uart.h"
#include "tmc2209_shared.h"
#include "tmc2209_internal.h"
#include "stdint.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_uart_freertos.h"

/************************************
 *     Private Macros / Defines    *
 ************************************/

#define TMC_READ_PACKAGE_SIZE 4
#define TMC_WRITE_PACKAGE_SIZE 8

#define TMC_IFCNT_ADDR 0x02

/***************************
 *     Private Typedefs     *
 ***************************/

/*****************************************
 *     Private Function Declarations     *
 *****************************************/

static inline void create_read_datagram(TMC_Handle_t handle, uint8_t reg_addr,
                                        TMC_Read_Datagram_t* datagram);
static inline void create_write_datagram(TMC_Handle_t handle, uint8_t reg_addr, uint32_t data,
                                         TMC_Write_Datagram_t* datagram);
static inline void build_read_package(TMC_Read_Datagram_t* datagram, uint8_t* package);
static inline void build_write_package(TMC_Write_Datagram_t* datagram, uint8_t* package);
static inline void calc_CRC(uint8_t* datagram, size_t datagramLength);
static uint8_t     dump_rx_buffer(TMC_Handle_t handle, uint8_t dumpCount, TickType_t deadline);

/****************************
 *     Public Variables     *
 ****************************/

/*****************************
 *     Private Variables     *
 *****************************/

static uint8_t transmit_read_package[TMC_READ_PACKAGE_SIZE];
static uint8_t transmit_response_package[TMC_WRITE_PACKAGE_SIZE];
static uint8_t transmit_write_package[TMC_WRITE_PACKAGE_SIZE];

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

status_t TMCi_read(TMC_Handle_t handle, uint8_t regAddr, uint32_t* data, TickType_t deadline)
{
    if (handle == NULL || data == NULL)
    {
        return kStatus_Fail;
    }

    status_t            status;
    TMC_Read_Datagram_t readDatagram;
    char                logMsg[60];

    create_read_datagram(handle, regAddr, &readDatagram);
    build_read_package(&readDatagram, transmit_read_package);
    TickType_t currentTick = xTaskGetTickCount();
    if (deadline <= currentTick)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Deadline for UART receive has already passed",
                 handle->label);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }
    TickType_t ticksUntilDeadline = deadline - currentTick;
    PORT_SetPinMux(PORTE, 0, kPORT_MuxAlt3);
    status = UART_RTOS_Send(handle->uartRTOSHandle, transmit_read_package, TMC_READ_PACKAGE_SIZE);
    PORT_SetPinMux(PORTE, 0, kPORT_MuxAlt1);
    dump_rx_buffer(handle, TMC_READ_PACKAGE_SIZE, deadline);
    if (status != kStatus_Success)
    {
        char logMsg[60];
        snprintf(logMsg, sizeof(logMsg), "[%s] UART send failed during read", handle->label);
        LOG_ERROR(logMsg);
        return status;
    }

    status = UART_RTOS_Receive(handle->uartRTOSHandle, transmit_response_package,
                               TMC_WRITE_PACKAGE_SIZE, NULL, ticksUntilDeadline);
    if (status != kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] UART receive timeout during read", handle->label);
        LOG_WARN(logMsg);
        return status;
    }

    calc_CRC(transmit_response_package, TMC_WRITE_PACKAGE_SIZE);

    *data = (transmit_response_package[3] << 24) | (transmit_response_package[4] << 16) |
            (transmit_response_package[5] << 8) | transmit_response_package[6];

    /* Clear response buffer */
    for (size_t i = 0; i < TMC_WRITE_PACKAGE_SIZE; i++)
    {
        transmit_response_package[i] = 0;
    }

    return kStatus_Success;
}

status_t TMCi_write(TMC_Handle_t handle, uint8_t regAddr, uint32_t* data, TickType_t deadline)
{
    if (handle == NULL || data == NULL)
    {
        return kStatus_Fail;
    }

#if TMC_CONFIRM_WRITES == (1)
    uint8_t expectedTransmissionCount = handle->transmissionCount + 1;
#endif

    status_t             status;
    TMC_Write_Datagram_t writeDatagram;

    create_write_datagram(handle, regAddr, *data, &writeDatagram);
    build_write_package(&writeDatagram, transmit_write_package);
    PORT_SetPinMux(PORTE, 0, kPORT_MuxAlt3);
    status = UART_RTOS_Send(handle->uartRTOSHandle, transmit_write_package, TMC_WRITE_PACKAGE_SIZE);
    PORT_SetPinMux(PORTE, 0, kPORT_MuxAlt1);
    dump_rx_buffer(handle, TMC_WRITE_PACKAGE_SIZE, deadline);
    if (status != kStatus_Success)
    {
        char logMsg[60];
        snprintf(logMsg, sizeof(logMsg), "[%s] UART send failed during write", handle->label);
        LOG_ERROR(logMsg);
        return status;
    }

#if TMC_CONFIRM_WRITES == (1)
    status = TMCi_read_transmissioncount(handle, deadline);
    if (status != kStatus_Success)
    {
        char logMsg[70];
        snprintf(logMsg, sizeof(logMsg), "[%s] Failed to confirm write (transmissioncount)",
                 handle->label);
        LOG_WARN(logMsg);
        return status;
    }

    if (expectedTransmissionCount != handle->transmissionCount)
    {
        char logMsg[80];
        snprintf(logMsg, sizeof(logMsg), "[%s] Write confirmation mismatch: exp=%d got=%d",
                 handle->label, expectedTransmissionCount, handle->transmissionCount);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }
#endif

    return kStatus_Success;
}

status_t TMCi_read_transmissioncount(TMC_Handle_t handle, TickType_t deadline)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    status_t status;
    uint32_t receiveBuffer;

    status = TMCi_read(handle, TMC_IFCNT_ADDR, &receiveBuffer, deadline);
    if (status != kStatus_Success)
    {
        return status;
    }

    handle->transmissionCount = receiveBuffer & 0xFF;
    return kStatus_Success;
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/

static inline void create_write_datagram(TMC_Handle_t handle, uint8_t reg_addr, uint32_t data,
                                         TMC_Write_Datagram_t* datagram)
{
    if (handle == NULL || datagram == NULL)
    {
        return;
    }

    datagram->sync     = 0b0101;
    datagram->reserved = 0b0000;
    datagram->dev_addr = handle->serialAdress & 0xFF;
    datagram->reg_addr = (reg_addr | 0x80) & 0xFF;
    datagram->data     = data & 0xFFFFFFFF;
}

static inline void create_read_datagram(TMC_Handle_t handle, uint8_t reg_addr,
                                        TMC_Read_Datagram_t* datagram)
{
    if (handle == NULL || datagram == NULL)
    {
        return;
    }

    datagram->sync     = 0b0101;
    datagram->reserved = 0b0000;
    datagram->dev_addr = handle->serialAdress & 0xFF;
    datagram->reg_addr = reg_addr & 0xFF;
}

static inline void calc_CRC(uint8_t* datagram, size_t datagramLength)
{
    if (datagram == NULL || datagramLength == 0)
    {
        return;
    }

    uint8_t  i;
    uint8_t  j;
    uint8_t* crc = datagram + (datagramLength - 1); // CRC located in last byte of message
    uint8_t  currentByte;
    *crc = 0;
    for (i = 0; i < (datagramLength - 1); i++)
    {                              // Execute for all bytes of a message
        currentByte = datagram[i]; // Retrieve a byte to be sent from Array
        for (j = 0; j < 8; j++)
        {
            if ((*crc >> 7) ^ (currentByte & 0x01)) // update CRC based result of XOR operation
            {
                *crc = (*crc << 1) ^ 0x07;
            }
            else
            {
                *crc = (*crc << 1);
            }
            currentByte = currentByte >> 1;
        } // for CRC bit
    } // for message byte
}

static inline void build_read_package(TMC_Read_Datagram_t* datagram, uint8_t* package)
{
    if (datagram == NULL || package == NULL)
    {
        return;
    }

    package[0] = ((datagram->reserved << 4) & 0xF0) | (datagram->sync & 0x0F);
    package[1] = datagram->dev_addr & 0xFF;
    package[2] = datagram->reg_addr & 0xFF;
    package[3] = datagram->crc & 0xFF;
    calc_CRC(package, TMC_READ_PACKAGE_SIZE);
}

static inline void build_write_package(TMC_Write_Datagram_t* datagram, uint8_t* package)
{
    if (datagram == NULL || package == NULL)
    {
        return;
    }

    package[0] = ((datagram->reserved << 4) & 0xF0) | (datagram->sync & 0x0F);
    package[1] = datagram->dev_addr & 0xFF;
    package[2] = datagram->reg_addr & 0xFF;
    package[3] = (datagram->data >> 24) & 0xFF;
    package[4] = (datagram->data >> 16) & 0xFF;
    package[5] = (datagram->data >> 8) & 0xFF;
    package[6] = datagram->data & 0xFF;
    package[7] = datagram->crc & 0xFF;
    calc_CRC(package, TMC_WRITE_PACKAGE_SIZE);
}

static uint8_t dump_rx_buffer(TMC_Handle_t handle, uint8_t dumpCount, TickType_t deadline)
{
    if (handle == NULL)
    {
        return 0;
    }

    /* Dump received echo */
    static char logMsg[60];
    uint8_t     dump = 0;
    size_t      n;
    uint8_t     testCount = 0;

    for (uint8_t i = 0; i < dumpCount; i++)
    {
        TickType_t currentTick = xTaskGetTickCount();
        if (deadline <= currentTick)
        {
            snprintf(logMsg, sizeof(logMsg), "[%s] Deadline for UART receive has already passed",
                     handle->label);
            LOG_ERROR(logMsg);
            return kStatus_Fail;
        }
        TickType_t ticksUntilDeadline = deadline - currentTick;

        status_t status =
            UART_RTOS_Receive(handle->uartRTOSHandle, &dump, 1, &n, ticksUntilDeadline);
        // size_t   currentLength = UART_TransferGetRxRingBufferLength(handle->uartHandle);
        if (status == kStatus_Success && n > 0)
        {
            testCount++;
        }
        else
        {
            LOG_ERROR("Failed to dump RX buffer byte");
        }
    }

    return testCount;
}
