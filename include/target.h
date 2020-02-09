/*
 * Copyright (C) 2017 Obermaier Johannes
 *
 * This Source Code Form is subject to the terms of the MIT License.
 * If a copy of the MIT License was not distributed with this file,
 * you can obtain one at https://opensource.org/licenses/MIT
 */

#ifndef INC_TARGET_H
#define INC_TARGET_H
#include <Arduino.h>

#define RCC_AHBENR_GPIO_RESET (RCC_AHBENR_GPIOAEN)
#define RCC_AHBENR_GPIO_POWER (RCC_AHBENR_GPIOAEN)

#define GPIO_RESET (GPIOA)
#define PIN_RESET (8u)

#define GPIO_POWER (GPIOA)
#define PIN_POWER (11u)

#define POWER_PIN PA11
#define RESET_PIN PA8

void targetSysCtrlInit( void );
void targetSysReset( void );
void targetSysUnReset( void );
void targetSysOff( void );
void targetSysOn( void );

#endif
