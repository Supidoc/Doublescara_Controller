/************************************************************
 * @file    led.c
 * @brief   Implementation of LED control via GPIO PTB18
 * @author  rp
 * @date    12 Apr 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include "led.h"
#include "fsl_gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "fsl_common.h"
#include "MK22F51212_COMMON.h"

/************************************
 *     Private Macros / Defines		*
 ************************************/
#define LED_GPIO GPIOB
#define LED_PIN 18U

/*********************************************
 *      Public Function Implementations		 *
 *********************************************/

status_t LED_init(void)
{
    gpio_pin_config_t config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic  = 0U // Start aus
    };
    GPIO_PinInit(LED_GPIO, LED_PIN, &config);
    return kStatus_Success;
}

void LED_toggle(void)
{
    while (1)
    {
        LED_on();
        vTaskDelay(pdMS_TO_TICKS(5000)); // 5 Sekunden an

        LED_off();
        vTaskDelay(pdMS_TO_TICKS(5000)); // 5 Sekunden aus
    }
}

void LED_on(void)
{
    GPIO_PinWrite(LED_GPIO, LED_PIN, 1U); // High == ein
}

void LED_off(void)
{
    GPIO_PinWrite(LED_GPIO, LED_PIN, 0U); // Low == aus
}
