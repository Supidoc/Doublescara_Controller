/************************************************************
 * @file    pca9555a.c
 * @brief   Filedescription
 * @author  dg
 * @date    2 Apr 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include "pca9555a.h"
#include "stdint.h"
#include "fsl_common.h"
#include "fsl_i2c_freertos.h"
#include "FreeRTOS.h"
#include "log.h"

/************************************
 *     Private Macros / Defines    *
 ************************************/

/***************************
 *     Private Typedefs     *
 ***************************/

#define PCA_REG_I0 0x0
#define PCA_REG_I1 0x1
#define PCA_REG_O0 0x2
#define PCA_REG_O1 0x3
#define PCA_REG_PI0 0x4
#define PCA_REG_PI1 0x5
#define PCA_REG_C0 0x6
#define PCA_REG_C1 0x7

/*****************************************
 *     Private Function Declarations     *
 *****************************************/

/****************************
 *     Public Variables     *
 ****************************/

/*****************************
 *     Private Variables     *
 *****************************/

static SemaphoreHandle_t pcaMutex = NULL;

/*******************************************
 *     Public Function Implementations     *
 *******************************************/
status_t PCA_init(void)
{
    pcaMutex = xSemaphoreCreateRecursiveMutex();
    if (pcaMutex == NULL)
    {
        return kStatus_Fail;
    }
    return kStatus_Success;
}

status_t PCA_init_pin(PCA_Port_t port, uint8_t pin, PCA_Pin_Config_t config)
{
    i2c_master_transfer_t transfer;
    transfer.slaveAddress   = PCA_ADRESS;
    transfer.direction      = kI2C_Read;
    transfer.subaddressSize = 1;
    uint8_t data;
    if (config.pinDirection == PCA_PIN_INPUT)
    {
        if (port == 0)
        {
            transfer.subaddress = PCA_REG_PI0;
        }
        else
        {
            transfer.subaddress = PCA_REG_PI1;
        }
        transfer.dataSize = 1;
        transfer.data     = &data;
        transfer.flags    = 0;
        if (I2C_RTOS_Transfer(PCA_I2C_HANDLE, &transfer) != kStatus_Success)
        {
            LOG_ERROR("PCA9555A: Failed to read input pin polarity register");
            return kStatus_Fail;
        }
        transfer.flags     = 0;
        transfer.direction = kI2C_Write;
        data               = (data & ~(0b1 << pin)) | (config.polarityInversion << pin);
        if (I2C_RTOS_Transfer(PCA_I2C_HANDLE, &transfer) != kStatus_Success)
        {
            LOG_ERROR("PCA9555A: Failed to write input pin polarity");
            return kStatus_Fail;
        }
        LOG_DEBUG("PCA9555A: Input pin configured");
    }
    else
    {
        if (port == 0)
        {
            transfer.subaddress = PCA_REG_O0;
        }
        else
        {
            transfer.subaddress = PCA_REG_O1;
        }
        transfer.dataSize = 1;
        transfer.data     = &data;
        transfer.flags    = 0;
        if (I2C_RTOS_Transfer(PCA_I2C_HANDLE, &transfer) != kStatus_Success)
        {
            LOG_ERROR("PCA9555A: Failed to read output pin configuration");
            return kStatus_Fail;
        }
        transfer.flags     = 0;
        transfer.direction = kI2C_Write;
        data               = (data & ~(0b1 << pin)) | (config.outputLevel << pin);
        if (I2C_RTOS_Transfer(PCA_I2C_HANDLE, &transfer) != kStatus_Success)
        {
            LOG_ERROR("PCA9555A: Failed to write output pin level");
            return kStatus_Fail;
        }
        LOG_DEBUG("PCA9555A: Output pin configured");
    }
    if (port == 0)
    {
        transfer.subaddress = PCA_REG_C0;
    }
    else
    {
        transfer.subaddress = PCA_REG_C1;
    }
    transfer.dataSize  = 1;
    transfer.data      = &data;
    transfer.flags     = 0;
    transfer.direction = kI2C_Read;
    if (I2C_RTOS_Transfer(PCA_I2C_HANDLE, &transfer) != kStatus_Success)
    {
        LOG_ERROR("PCA9555A: Failed to read configuration register");
        return kStatus_Fail;
    }
    transfer.flags     = 0;
    transfer.direction = kI2C_Write;
    data               = (data & ~(0b1 << pin)) | (config.pinDirection << pin);
    if (I2C_RTOS_Transfer(PCA_I2C_HANDLE, &transfer) != kStatus_Success)
    {
        LOG_ERROR("PCA9555A: Failed to write configuration register");
        return kStatus_Fail;
    }
    return kStatus_Success;
}

status_t PCA_Read_Pin(PCA_Port_t port, uint8_t pin, uint8_t* value)
{
    uint16_t pins = 0;
    if (PCA_Read_All_Inputs(&pins) != kStatus_Success)
    {
        LOG_ERROR("PCA9555A: Failed to read pin - cannot read inputs");
        return kStatus_Fail;
    }
    *value = (pins & (0b1 << (port * 8 + pin))) != 0;
    return kStatus_Success;
}

status_t PCA_Read_All_Inputs(uint16_t* pins)
{
    i2c_master_transfer_t transfer;
    uint16_t              data = 0;

    transfer.slaveAddress   = PCA_ADRESS;
    transfer.direction      = kI2C_Read;
    transfer.subaddressSize = 1;
    transfer.subaddress     = PCA_REG_I0;
    transfer.dataSize       = sizeof(data);
    transfer.data           = (void*)&data;
    transfer.flags          = 0;

    if (I2C_RTOS_Transfer(PCA_I2C_HANDLE, &transfer) != kStatus_Success)
    {
        LOG_ERROR("PCA9555A: Failed to read input register - I2C communication error");
        return kStatus_Fail;
    }

    *pins = data;
    return kStatus_Success;
}

status_t PCA_Read_All_Outputs(uint16_t* pins)
{
    i2c_master_transfer_t transfer;
    uint16_t              data = 0;

    transfer.slaveAddress   = PCA_ADRESS;
    transfer.direction      = kI2C_Read;
    transfer.subaddressSize = 1;
    transfer.subaddress     = PCA_REG_O0;
    transfer.dataSize       = sizeof(data);
    transfer.data           = (void*)&data;
    transfer.flags          = 0;

    if (I2C_RTOS_Transfer(PCA_I2C_HANDLE, &transfer) != kStatus_Success)
    {
        LOG_ERROR("PCA9555A: Failed to read output register - I2C communication error");
        return kStatus_Fail;
    }

    *pins = data;
    return kStatus_Success;
}

status_t PCA_Write_Pin(PCA_Port_t port, uint8_t pin, uint8_t value)
{
    uint16_t pins        = 0;
    uint16_t newPinValue = (value & 0b1) << (port * 8 + pin);

    if (PCA_Read_All_Outputs(&pins) != kStatus_Success)
    {
        LOG_ERROR("PCA9555A: Failed to write pin - cannot read current output state");
        return kStatus_Fail;
    }
    pins = (pins & ~(0b1 << (port * 8 + pin))) | newPinValue;

    if (PCA_Write_All_Outputs(pins) != kStatus_Success)
    {
        LOG_ERROR("PCA9555A: Failed to write pin output");
        return kStatus_Fail;
    }
    return kStatus_Success;
}

status_t PCA_Set_Pin(PCA_Port_t port, uint8_t pin)
{
    uint16_t pins = 0b1 << (port * 8 + pin);

    if (PCA_Set_All_Outputs(pins) != kStatus_Success)
    {
        LOG_ERROR("PCA9555A: Failed to set pin");
        return kStatus_Fail;
    }
    return kStatus_Success;
}

status_t PCA_Clear_Pin(PCA_Port_t port, uint8_t pin)
{
    uint16_t pins = 0b1 << (port * 8 + pin);

    if (PCA_Clear_All_Outputs(pins) != kStatus_Success)
    {
        LOG_ERROR("PCA9555A: Failed to clear pin");
        return kStatus_Fail;
    }
    return kStatus_Success;
}

status_t PCA_Toggle_Pin(PCA_Port_t port, uint8_t pin)
{
    uint16_t pins = 0b1 << (port * 8 + pin);

    if (PCA_Toggle_All_Outputs(pins) != kStatus_Success)
    {
        LOG_ERROR("PCA9555A: Failed to toggle pin");
        return kStatus_Fail;
    }
    return kStatus_Success;
}

status_t PCA_Write_All_Outputs(uint16_t pins)
{
    i2c_master_transfer_t transfer;

    transfer.slaveAddress   = PCA_ADRESS;
    transfer.direction      = kI2C_Write;
    transfer.subaddressSize = 1;
    transfer.subaddress     = PCA_REG_O0;
    transfer.dataSize       = sizeof(pins);
    transfer.data           = (void*)&pins;

    if (I2C_RTOS_Transfer(PCA_I2C_HANDLE, &transfer) != kStatus_Success)
    {
        LOG_ERROR("PCA9555A: Failed to write output register - I2C communication error");
        return kStatus_Fail;
    }

    return kStatus_Success;
}

status_t PCA_Set_All_Outputs(uint16_t pins)
{
    uint16_t data = 0;
    if (PCA_Read_All_Outputs(&data) != kStatus_Success)
    {
        LOG_ERROR("PCA9555A: Failed to set pins - cannot read current output state");
        return kStatus_Fail;
    }
    data |= pins;

    if (PCA_Write_All_Outputs(data) != kStatus_Success)
    {
        LOG_ERROR("PCA9555A: Failed to set pins - write failed");
        return kStatus_Fail;
    }
    return kStatus_Success;
}

status_t PCA_Clear_All_Outputs(uint16_t pins)
{
    uint16_t data = 0;
    if (PCA_Read_All_Outputs(&data) != kStatus_Success)
    {
        LOG_ERROR("PCA9555A: Failed to clear pins - cannot read current output state");
        return kStatus_Fail;
    }
    data &= ~pins;

    if (PCA_Write_All_Outputs(data) != kStatus_Success)
    {
        LOG_ERROR("PCA9555A: Failed to clear pins - write failed");
        return kStatus_Fail;
    }
    return kStatus_Success;
}

status_t PCA_Toggle_All_Outputs(uint16_t pins)
{
    uint16_t data = 0;
    if (PCA_Read_All_Outputs(&data) != kStatus_Success)
    {
        LOG_ERROR("PCA9555A: Failed to toggle pins - cannot read current output state");
        return kStatus_Fail;
    }
    data ^= pins;

    if (PCA_Write_All_Outputs(data) != kStatus_Success)
    {
        LOG_ERROR("PCA9555A: Failed to toggle pins - write failed");
        return kStatus_Fail;
    }
    return kStatus_Success;
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/
