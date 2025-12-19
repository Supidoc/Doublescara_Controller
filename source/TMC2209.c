/*
 * TMC2209.c
 *
 *  Created on: 5 Dec 2025
 *      Author: dg
 */

#include "TMC2209.h"
#include "peripherals.h"

/* Private defines */
#define TMC_READ_PACKAGE_SIZE 4
#define TMC_WRITE_PACKAGE_SIZE 8

/* Private function prototypes */
static void _create_read_datagram(TMC2209_Read_Datagram_t * datagram, uint8_t dev_addr, uint8_t reg_addr);
static void _create_write_datagram(TMC2209_Write_Datagram_t * datagram, uint8_t dev_addr, uint8_t reg_addr, uint32_t data);
static void _build_read_package(TMC2209_Read_Datagram_t * datagram, uint8_t * package);
static void _build_write_package(TMC2209_Write_Datagram_t * datagram, uint8_t * package);
static void _calcCRC(uint8_t * datagram, size_t datagramLength);
static uint8_t _dump_rx_buffer(uint8_t dumpCount);

/* Private variables */
static uint8_t transmit_read_package[TMC_READ_PACKAGE_SIZE];
static uint8_t transmit_response_package[TMC_WRITE_PACKAGE_SIZE];
static uint8_t transmit_write_package[TMC_WRITE_PACKAGE_SIZE];
static uint8_t interfaceTransmissionCount[4];
static TMC_SlaveStatus_t slaveStatus;

/*******************************************************************************
 * Public API Implementation
 ******************************************************************************/

status_t TMC_Init(void)
{
    return TMC_GetSlaveStates(&slaveStatus);
}

status_t TMC_Write_GCONF(TMC_SerialAddress_t serialAdress, TMC_GCONF_t *gconf)
{
    uint32_t data = 0;
    data |= (gconf->I_scale_analog & 0b1);
    data |= (gconf->internal_Rsense & 0b1) << 1;
    data |= (gconf->en_SpreadCycle & 0b1) << 2;
    data |= (gconf->shaft & 0b1) << 3;
    data |= (gconf->index_otpw & 0b1) << 3;
    data |= (gconf->index_step & 0b1) << 4;
    data |= (gconf->pdn_disable & 0b1) << 5;
    data |= (gconf->mstep_reg_select & 0b1) << 6;
    data |= (gconf->multistep_filt & 0b1) << 7;

    return TMC_write(serialAdress, TMC_GCONF_ADDR, &data);
}

status_t TMC_GetSlaveStates(TMC_SlaveStatus_t *slaveStatus)
{
    status_t status;
    *slaveStatus = 0;

    status = TMC_Read_TransmissionCount(TMC_SERIAL_ADDRESS_0, &interfaceTransmissionCount[0]);
    if (status == kStatus_Success)
    {
        *slaveStatus |= TMC_SLAVE_0_ACTIVE;
    }
    
    status = TMC_Read_TransmissionCount(TMC_SERIAL_ADDRESS_1, &interfaceTransmissionCount[1]);
    if (status == kStatus_Success)
    {
        *slaveStatus |= TMC_SLAVE_1_ACTIVE;
    }
    
    status = TMC_Read_TransmissionCount(TMC_SERIAL_ADDRESS_2, &interfaceTransmissionCount[2]);
    if (status == kStatus_Success)
    {
        *slaveStatus |= TMC_SLAVE_2_ACTIVE;
    }
    
    status = TMC_Read_TransmissionCount(TMC_SERIAL_ADDRESS_3, &interfaceTransmissionCount[3]);
    if (status == kStatus_Success)
    {
        *slaveStatus |= TMC_SLAVE_3_ACTIVE;
    }
    
    return kStatus_Success;
}

status_t TMC_Read_TransmissionCount(TMC_SerialAddress_t serialAdress, uint8_t *transmissionCount)
{
    status_t status;
    uint32_t receiveBuffer;
    
    status = TMC_read(serialAdress, TMC_IFCNT_ADDR, &receiveBuffer);
    if (status != kStatus_Success)
    {
        return status;
    }

    *transmissionCount = receiveBuffer & 0xFF;
    return kStatus_Success;
}

status_t TMC_read(TMC_SerialAddress_t serialAdress, uint8_t regAddr, uint32_t *data)
{
    status_t status;
    TMC2209_Read_Datagram_t readDatagram;

    _create_read_datagram(&readDatagram, serialAdress, regAddr);
    _build_read_package(&readDatagram, transmit_read_package);

    status = UART_RTOS_Send(&UART1_rtos_handle, transmit_read_package, TMC_READ_PACKAGE_SIZE);
    _dump_rx_buffer(TMC_READ_PACKAGE_SIZE);
    if (status != kStatus_Success)
    {
        return status;
    }

    status = UART_RTOS_Receive(&UART1_rtos_handle, transmit_response_package,
                               TMC_WRITE_PACKAGE_SIZE, NULL, TMC_TIMEOUT_MS);
    if (status != kStatus_Success)
    {
        return status;
    }

    _calcCRC(transmit_response_package, TMC_WRITE_PACKAGE_SIZE);

    *data = (transmit_response_package[3] << 24) | (transmit_response_package[4] << 16) |
            (transmit_response_package[5] << 8) | transmit_response_package[6];
    
    /* Clear response buffer */
    for (size_t i = 0; i < TMC_WRITE_PACKAGE_SIZE; i++)
    {
        transmit_response_package[i] = 0;
    }
    
    return kStatus_Success;
}

status_t TMC_write(TMC_SerialAddress_t serialAddress, uint8_t regAddr, uint32_t *data)
{
#ifdef TMC_CONFIRM_WRITES
    uint8_t expectedTransmissionCount = interfaceTransmissionCount[serialAddress] + 1;
#endif

    status_t status;
    TMC2209_Write_Datagram_t writeDatagram;

    _create_write_datagram(&writeDatagram, serialAddress, regAddr, *data);
    _build_write_package(&writeDatagram, transmit_write_package);

    status = UART_RTOS_Send(&UART1_rtos_handle, transmit_write_package, TMC_WRITE_PACKAGE_SIZE);
    _dump_rx_buffer(TMC_WRITE_PACKAGE_SIZE);
    if (status != kStatus_Success)
    {
        return status;
    }

#ifdef TMC_CONFIRM_WRITES
    status = TMC_Read_TransmissionCount(serialAddress, &interfaceTransmissionCount[serialAddress]);
    if (status != kStatus_Success)
    {
        return status;
    }

    if (expectedTransmissionCount != interfaceTransmissionCount[serialAddress])
    {
        return kStatus_Fail;
    }
#endif
    
    return kStatus_Success;
}

/*******************************************************************************
 * Private Helper Functions
 ******************************************************************************/

static void _create_write_datagram(TMC2209_Write_Datagram_t *datagram, uint8_t dev_addr, uint8_t reg_addr, uint32_t data)
{
    datagram->sync = 0b0101;
    datagram->reserved = 0b0000;
    datagram->dev_addr = dev_addr & 0xFF;
    datagram->reg_addr = (reg_addr | 0x80) & 0xFF;
    datagram->data = data & 0xFFFFFFFF;
}

static void _create_read_datagram(TMC2209_Read_Datagram_t *datagram, uint8_t dev_addr, uint8_t reg_addr)
{
    datagram->sync = 0b0101;
    datagram->reserved = 0b0000;
    datagram->dev_addr = dev_addr & 0xFF;
    datagram->reg_addr = reg_addr & 0xFF;
}

static void _calcCRC(uint8_t * datagram, size_t datagramLength)
{
    uint8_t i;
    uint8_t j;
    uint8_t * crc = datagram + (datagramLength - 1); // CRC located in last byte of message
    uint8_t currentByte;
    *crc = 0;
    for (i = 0; i < (datagramLength - 1); i++)
    { // Execute for all bytes of a message
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

static void _build_read_package(TMC2209_Read_Datagram_t *datagram, uint8_t *package)
{
    package[0] = ((datagram->reserved << 4) & 0xF0) | (datagram->sync & 0x0F);
    package[1] = datagram->dev_addr & 0xFF;
    package[2] = datagram->reg_addr & 0xFF;
    package[3] = datagram->crc & 0xFF;
    _calcCRC(package, TMC_READ_PACKAGE_SIZE);
}

static void _build_write_package(TMC2209_Write_Datagram_t *datagram, uint8_t *package)
{
    package[0] = ((datagram->reserved << 4) & 0xF0) | (datagram->sync & 0x0F);
    package[1] = datagram->dev_addr & 0xFF;
    package[2] = datagram->reg_addr & 0xFF;
    package[3] = (datagram->data >> 24) & 0xFF;
    package[4] = (datagram->data >> 16) & 0xFF;
    package[5] = (datagram->data >> 8) & 0xFF;
    package[6] = datagram->data & 0xFF;
    package[7] = datagram->crc & 0xFF;
    _calcCRC(package, TMC_WRITE_PACKAGE_SIZE);
}

static uint8_t _dump_rx_buffer(uint8_t dumpCount)
{
    /* Dump received echo */
    uint8_t dump = 0;
    size_t n;
    uint8_t testCount = 0;
    
    for (uint8_t i = 0; i < dumpCount; i++)
    {
        status_t status = UART_RTOS_Receive(&UART1_rtos_handle, &dump, 1, &n, TMC_TIMEOUT_MS);
        size_t currentLength = UART_TransferGetRxRingBufferLength(&UART1_uart_handle);
        if (status == kStatus_Success && n > 0)
        {
            testCount++;
        }
        else
        {
            __BKPT();
        }
    }
    
    return testCount;
}
