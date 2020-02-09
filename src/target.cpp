/*
 * Copyright (C) 2017 Obermaier Johannes
 *
 * This Source Code Form is subject to the terms of the MIT License.
 * If a copy of the MIT License was not distributed with this file,
 * you can obtain one at https://opensource.org/licenses/MIT
 */

#include "target.h"
#include "clk.h"

void targetSysCtrlInit( void )
{
    #ifdef stm32f0xx
	RCC->AHBENR |= RCC_AHBENR_GPIO_RESET;
	RCC->AHBENR |= RCC_AHBENR_GPIO_POWER;

	GPIO_RESET->MODER |= (0x01u << (PIN_RESET << 1u));
	GPIO_POWER->MODER |= (0x01u << (PIN_POWER << 1u));

	GPIO_RESET->OSPEEDR |= (0x03u << (PIN_RESET << 1u));
	GPIO_POWER->OSPEEDR |= (0x03u << (PIN_POWER << 1u));
#endif
    pinMode(POWER_PIN,OUTPUT);
    pinMode(RESET_PIN,OUTPUT);
    targetSysOn();
	targetSysUnReset();
    delay(100);
	targetSysOff();
	targetSysReset();

	return ;
}

void targetSysReset( void )
{
	//GPIO_RESET->BSRR = (0x01u << (PIN_RESET + GPIO_PIN_RESET));
  //  HAL_GPIO_WritePin(GPIO_RESET,PIN_RESET,GPIO_PIN_RESET);
    digitalWrite(RESET_PIN,LOW);
	return ;
}

void targetSysUnReset( void )
{
	//GPIO_RESET->BSRR = (0x01u << (PIN_RESET + GPIO_PIN_SET));
   // HAL_GPIO_WritePin(GPIO_RESET,PIN_RESET,GPIO_PIN_SET);
    digitalWrite(RESET_PIN,HIGH);
	return ;
}


void targetSysOff( void )
{
	//GPIO_POWER->BSRR = (0x01u << (PIN_POWER + GPIO_PIN_RESET));
   // HAL_GPIO_WritePin(GPIO_POWER,PIN_POWER,GPIO_PIN_RESET);
    digitalWrite(POWER_PIN,LOW);
	return ;
}

void targetSysOn( void )
{
	//GPIO_POWER->BSRR = (0x01u << (PIN_POWER + GPIO_PIN_SET));
   // HAL_GPIO_WritePin(GPIO_POWER,PIN_POWER,GPIO_PIN_SET);
    digitalWrite(POWER_PIN,HIGH);
	return ;
}
