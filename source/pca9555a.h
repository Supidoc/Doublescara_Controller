/************************************************************
 * @file    pca9555a.h
 * @brief   PCA9555A I2C IO Expander Driver
 * @details Provides functions to initialize and control the PCA9555A
 *          16-bit I2C-bus IO expander with individual input/output
 *          configuration. Supports reading/writing individual pins
 *          and entire ports.
 * @author  dg
 * @date    2 Apr 2026
 ************************************************************/

#ifndef PCA9555A_H_
#define PCA9555A_H_

/********************
 *     Includes    *
 ********************/
#include "fsl_common.h"
#include "peripherals.h"

/***********************************
 *     Public Macros / Defines     *
 ***********************************/

/**
 * @defgroup PCA9555A PCA9555A Driver
 * @{
 */

/** @brief 7-bit I2C device address for PCA9555A */
#define PCA_ADRESS 0b0100000

/** @brief I2C handle for PCA9555A communication */
#define PCA_I2C_HANDLE &I2C0_rtosHandle



/***************************
 *     Public Typedefs     *
 ***************************/

/**
 * @brief Port selection for PCA9555A IO expander
 * @details Selects which 8-bit port (Port 0 or Port 1) to access
 */
typedef enum _PCA_Port
{
    PCA_PORT_0 = 0, /**< Port 0 (IO0-IO7) */
    PCA_PORT_1 = 1  /**< Port 1 (IO8-IO15) */
} PCA_Port_t;

/**
 * @brief Pin direction configuration
 * @details Configures whether a pin is used as input or output
 */
typedef enum _PCA_PinDirection
{
    PCA_PIN_INPUT  = 1, /**< Configure pin as input */
    PCA_PIN_OUTPUT = 0  /**< Configure pin as output */
} PCA_PinDirection_t;

/**
 * @brief Output level for pin configuration
 * @details Defines the default output level for pins configured as outputs
 */
typedef enum _PCA_OutputLevel
{
    PCA_PIN_LOW  = 0, /**< Default output level is low */
    PCA_PIN_HIGH = 1  /**< Default output level is high */
} PCA_OutputLevel_t;

/**
 * @brief Polarity inversion configuration
 * @details Enables or disables polarity inversion on input pins
 */
typedef enum _PCA_PolarityInversion
{
    PCA_HIGH_IS_ONE  = 0, /**< Normal polarity (no inversion) */
    PCA_HIGH_IS_ZERO = 1  /**< Inverted polarity */
} PCA_PolarityInversion_t;

/**
 * @brief Pin configuration structure
 * @details Contains all configuration parameters for a single PCA9555A pin
 */
typedef struct _PCA_Pin_Config
{
    PCA_PinDirection_t      pinDirection;      /**< Pin direction (input or output) */
    PCA_OutputLevel_t       outputLevel;       /**< Default output level */
    PCA_PolarityInversion_t polarityInversion; /**< Polarity inversion setting */
} PCA_Pin_Config_t;

/****************************
 *     Public Variables     *
 ****************************/

/**************************************
 *     Public Function Prototypes    *
 **************************************/

/**
 * @brief Initialize the PCA9555A IO expander
 * @details Performs initial setup and configuration of the PCA9555A device
 *          via I2C communication
 * @return kStatus_Success if initialization successful, error code otherwise
 */
status_t PCA_init(void);

/**
 * @brief Configure a specific pin on the PCA9555A
 * @param port Port to configure (PCA_PORT_0 or PCA_PORT_1)
 * @param pin Pin number within the port (0-7)
 * @param config Configuration structure containing pin settings
 * @return kStatus_Success if configuration successful, error code otherwise
 */
status_t PCA_init_pin(PCA_Port_t port, uint8_t pin, PCA_Pin_Config_t config);

/**
 * @brief Read the value of a specific pin
 * @param port Port to read from (PCA_PORT_0 or PCA_PORT_1)
 * @param pin Pin number within the port (0-7)
 * @param value Pointer to store the read pin value (0 or 1)
 * @return kStatus_Success if read successful, error code otherwise
 */
status_t PCA_Read_Pin(PCA_Port_t port, uint8_t pin, uint8_t* value);

/**
 * @brief Read all input pins
 * @details Reads the state of all 16 pins configured as inputs
 * @param pins Pointer to store the 16-bit input values
 *             Lower byte (bits 0-7) = Port 0, Upper byte (bits 8-15) = Port 1
 * @return kStatus_Success if read successful, error code otherwise
 */
status_t PCA_Read_All_Inputs(uint16_t* pins);

/**
 * @brief Read all output pins
 * @details Reads the state of all 16 pins configured as outputs
 * @param pins Pointer to store the 16-bit output values
 *             Lower byte (bits 0-7) = Port 0, Upper byte (bits 8-15) = Port 1
 * @return kStatus_Success if read successful, error code otherwise
 */
status_t PCA_Read_All_Outputs(uint16_t* pins);

/**
 * @brief Write a value to a specific output pin
 * @param port Port containing the pin (PCA_PORT_0 or PCA_PORT_1)
 * @param pin Pin number within the port (0-7)
 * @param value Value to write (0 for low, 1 for high)
 * @return kStatus_Success if write successful, error code otherwise
 */
status_t PCA_Write_Pin(PCA_Port_t port, uint8_t pin, uint8_t value);

/**
 * @brief Set a specific output pin high
 * @param port Port containing the pin (PCA_PORT_0 or PCA_PORT_1)
 * @param pin Pin number within the port (0-7)
 * @return kStatus_Success if set successful, error code otherwise
 */
status_t PCA_Set_Pin(PCA_Port_t port, uint8_t pin);

/**
 * @brief Clear a specific output pin (set low)
 * @param port Port containing the pin (PCA_PORT_0 or PCA_PORT_1)
 * @param pin Pin number within the port (0-7)
 * @return kStatus_Success if clear successful, error code otherwise
 */
status_t PCA_Clear_Pin(PCA_Port_t port, uint8_t pin);

/**
 * @brief Toggle a specific output pin
 * @param port Port containing the pin (PCA_PORT_0 or PCA_PORT_1)
 * @param pin Pin number within the port (0-7)
 * @return kStatus_Success if toggle successful, error code otherwise
 */
status_t PCA_Toggle_Pin(PCA_Port_t port, uint8_t pin);

/**
 * @brief Write values to all output pins
 * @param pins 16-bit value representing the desired state of all output pins
 *             Lower byte (bits 0-7) = Port 0, Upper byte (bits 8-15) = Port 1
 * @return kStatus_Success if write successful, error code otherwise
 */
status_t PCA_Write_All_Outputs(uint16_t pins);

/**
 * @brief Set all output pins high
 * @param pins 16-bit bitmask indicating which pins to set
 *             Lower byte (bits 0-7) = Port 0, Upper byte (bits 8-15) = Port 1
 * @return kStatus_Success if set successful, error code otherwise
 */
status_t PCA_Set_All_Outputs(uint16_t pins);

/**
 * @brief Clear all output pins (set low)
 * @param pins 16-bit bitmask indicating which pins to clear
 *             Lower byte (bits 0-7) = Port 0, Upper byte (bits 8-15) = Port 1
 * @return kStatus_Success if clear successful, error code otherwise
 */
status_t PCA_Clear_All_Outputs(uint16_t pins);

/**
 * @brief Toggle all output pins
 * @param pins 16-bit bitmask indicating which pins to toggle
 *             Lower byte (bits 0-7) = Port 0, Upper byte (bits 8-15) = Port 1
 * @return kStatus_Success if toggle successful, error code otherwise
 */
status_t PCA_Toggle_All_Outputs(uint16_t pins);

/** @} */
#endif /* PCA9555A_H_ */
