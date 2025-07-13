/*
 * Copyright 2025 NXP
 * 
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include "pin_mux.h"
#include "fsl_common.h"
#include "fsl_io_mux.h"
#include "fsl_gpio.h"
#include "LED.h"


#define LED_PIN 0U  // BLUE LED PIN

/*
* Blue LED INIT
*/

void LED_INIT(void)
{
    gpio_pin_config_t RGB_LED_config = {
        kGPIO_DigitalOutput,
        1,
    };
    IO_MUX_SetPinMux(IO_MUX_GPIO0);
    IO_MUX_SetPinMux(IO_MUX_GPIO1);
    IO_MUX_SetPinMux(IO_MUX_GPIO12);
    
    GPIO_PortInit(GPIO, RGB_GPIO_PORT);
    
    GPIO_PinInit(GPIO, RGB_GPIO_PORT, RGB_LED_RED_PIN, &RGB_LED_config);
    GPIO_PinInit(GPIO, RGB_GPIO_PORT, RGB_LED_GREEN_PIN, &RGB_LED_config);
    GPIO_PinInit(GPIO, RGB_GPIO_PORT, RGB_LED_BLUE_PIN, &RGB_LED_config);
    
    GPIO_PinWrite(GPIO, RGB_GPIO_PORT, RGB_LED_RED_PIN, 1U);
    GPIO_PinWrite(GPIO, RGB_GPIO_PORT, RGB_LED_GREEN_PIN, 0U);
    GPIO_PinWrite(GPIO, RGB_GPIO_PORT, RGB_LED_BLUE_PIN, 0U);
}

void LED_ON(uint8_t pin)
{
    // Turn LED on
    GPIO_PinWrite(GPIO, 0U, pin, 0U);
}

void LED_OFF(uint8_t pin)
{
    // Turn LED off
    GPIO_PinWrite(GPIO, 0U, pin, 1U);
}
