/************************************************************
 * @file    display.c
 * @brief   SSD1306 OLED Display Driver (128x64, I2C)
 * @author  rp
 * @date    12 Apr 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include "display.h"
#include "peripherals.h"
#include "fsl_i2c_freertos.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/************************************
 *     Private Macros / Defines      *
 ************************************/
#define DISPLAY_I2C_ADDRESS 0x3Cu
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define DISPLAY_PAGES (DISPLAY_HEIGHT / 8)
#define DISPLAY_TEXT_MAX 64

/* SSD1306 Commands */
#define SSD1306_SETCONTRAST 0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON 0xA5
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF
#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETCOMPINS 0xDA
#define SSD1306_SETVCOMDETECT 0xDB
#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETPRECHARGE 0xD9
#define SSD1306_SETMULTIPLEXRATIO 0xA8
#define SSD1306_SETSTARTLINE 0x40
#define SSD1306_SETSEGMENTREMAP 0xA1
#define SSD1306_COMSCANDEC 0xC8
#define SSD1306_CHARGEPUMP 0x8D
#define SSD1306_MEMORYMODE 0x20
#define SSD1306_PAGEADDR 0x22
#define SSD1306_COLUMNADDR 0x21

static uint8_t displayBuffer[DISPLAY_PAGES][DISPLAY_WIDTH];
static char displayStatus[DISPLAY_TEXT_MAX] = "System start";
static bool displayReady = false;

/*******************************************
 *   Private Function Implementations     *
 *******************************************/
static status_t DISPLAY_i2cWrite(const uint8_t *data, size_t length)
{
    i2c_master_transfer_t transfer = {0};

    transfer.slaveAddress   = DISPLAY_I2C_ADDRESS;
    transfer.direction      = kI2C_Write;
    transfer.subaddressSize = 0;
    transfer.data           = (uint8_t *)data;
    transfer.dataSize       = length;
    transfer.flags          = kI2C_TransferDefaultFlag;

    return I2C_RTOS_Transfer(&I2C0_rtosHandle, &transfer);
}

static status_t DISPLAY_sendCommand(uint8_t cmd)
{
    uint8_t data[] = {0x00, cmd};
    return DISPLAY_i2cWrite(data, sizeof(data));
}

static status_t DISPLAY_sendData(const uint8_t *buffer, size_t length)
{
    uint8_t header = 0x40;  /* Data mode */
    status_t ret = DISPLAY_i2cWrite(&header, 1);
    if (ret != kStatus_Success)
    {
        return ret;
    }
    return DISPLAY_i2cWrite(buffer, length);
}

static status_t DISPLAY_initSequence(void)
{
    status_t status = kStatus_Success;

    status |= DISPLAY_sendCommand(SSD1306_DISPLAYOFF);
    status |= DISPLAY_sendCommand(SSD1306_SETDISPLAYCLOCKDIV);
    status |= DISPLAY_sendCommand(0x80);
    status |= DISPLAY_sendCommand(SSD1306_SETMULTIPLEXRATIO);
    status |= DISPLAY_sendCommand(0x3F);
    status |= DISPLAY_sendCommand(SSD1306_SETDISPLAYOFFSET);
    status |= DISPLAY_sendCommand(0x00);
    status |= DISPLAY_sendCommand(SSD1306_SETSTARTLINE | 0x00);
    status |= DISPLAY_sendCommand(SSD1306_CHARGEPUMP);
    status |= DISPLAY_sendCommand(0x14);
    status |= DISPLAY_sendCommand(SSD1306_MEMORYMODE);
    status |= DISPLAY_sendCommand(0x00);
    status |= DISPLAY_sendCommand(SSD1306_SETSEGMENTREMAP | 0x01);
    status |= DISPLAY_sendCommand(SSD1306_COMSCANDEC);
    status |= DISPLAY_sendCommand(SSD1306_SETCOMPINS);
    status |= DISPLAY_sendCommand(0x12);
    status |= DISPLAY_sendCommand(SSD1306_SETCONTRAST);
    status |= DISPLAY_sendCommand(0xCF);
    status |= DISPLAY_sendCommand(SSD1306_SETPRECHARGE);
    status |= DISPLAY_sendCommand(0xF1);
    status |= DISPLAY_sendCommand(SSD1306_SETVCOMDETECT);
    status |= DISPLAY_sendCommand(0x40);
    status |= DISPLAY_sendCommand(SSD1306_DISPLAYALLON_RESUME);
    status |= DISPLAY_sendCommand(SSD1306_NORMALDISPLAY);
    status |= DISPLAY_sendCommand(SSD1306_DISPLAYON);

    return status;
}

static status_t DISPLAY_renderBuffer(void)
{
    status_t status = kStatus_Success;

    for (uint8_t page = 0; page < DISPLAY_PAGES; page++)
    {
        status |= DISPLAY_sendCommand(SSD1306_PAGEADDR);
        status |= DISPLAY_sendCommand(page);
        status |= DISPLAY_sendCommand(page);
        status |= DISPLAY_sendCommand(SSD1306_COLUMNADDR);
        status |= DISPLAY_sendCommand(0);
        status |= DISPLAY_sendCommand(DISPLAY_WIDTH - 1);
        status |= DISPLAY_sendData(displayBuffer[page], DISPLAY_WIDTH);
    }

    return status;
}

static void DISPLAY_clearBuffer(void)
{
    memset(displayBuffer, 0x00, sizeof(displayBuffer));
}

status_t DISPLAY_init(void)
{
    /* Probe device */
    uint8_t probe[] = {0x00, 0xAE};  /* DISPLAYOFF probe */
    if (DISPLAY_i2cWrite(probe, sizeof(probe)) != kStatus_Success)
    {
        LOG_ERROR("DISPLAY: SSD1306 not found at 0x3C");
        displayReady = false;
        return kStatus_Fail;
    }

    /* Initialize display */
    DISPLAY_clearBuffer();
    status_t status = DISPLAY_initSequence();
    if (status != kStatus_Success)
    {
        LOG_ERROR("DISPLAY: initialization failed");
        displayReady = false;
        return status;
    }

    displayReady = true;
    return DISPLAY_renderBuffer();
}

status_t DISPLAY_write(const char *text)
{
    if (text == NULL || !displayReady)
    {
        return kStatus_InvalidArgument;
    }

    size_t length = strlen(text);
    if (length >= DISPLAY_TEXT_MAX)
    {
        length = DISPLAY_TEXT_MAX - 1;
    }

    memcpy(displayStatus, text, length);
    displayStatus[length] = '\0';

    DISPLAY_clearBuffer();
    /* Simple text rendering: display status text in buffer */
    /* This is a placeholder - full font rendering would require a font bitmap */

    return DISPLAY_renderBuffer();
}

status_t DISPLAY_showError(const char *text)
{
    if (text == NULL || !displayReady)
    {
        return kStatus_InvalidArgument;
    }

    char buffer[DISPLAY_TEXT_MAX];
    int result = snprintf(buffer, sizeof(buffer), "ERROR: %s", text);
    if (result < 0)
    {
        return kStatus_Fail;
    }

    return DISPLAY_write(buffer);
}

status_t DISPLAY_setStatus(const char *text)
{
    if (text == NULL)
    {
        return kStatus_InvalidArgument;
    }

    return DISPLAY_write(text);
}

void DISPLAY_task(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        if (displayReady)
        {
            if (DISPLAY_write(displayStatus) != kStatus_Success)
            {
                LOG_ERROR("DISPLAY: render failed");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
