/*
 * Copyright 2025 NXP
 * 
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define RGB_GPIO_PORT 		0U

#define RGB_LED_RED_PIN 	1U
#define RGB_LED_GREEN_PIN 	12U
#define RGB_LED_BLUE_PIN 	0U

void LED_INIT(void);

void LED_ON(uint8_t pin);

void LED_OFF(uint8_t pin);
