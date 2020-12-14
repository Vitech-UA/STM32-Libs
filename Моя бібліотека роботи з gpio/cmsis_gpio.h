/*
 * cmsis_gpio.h
 *
 *  Created on: 14 дек. 2020 г.
 *      Author: viktor.starovit
 */

#ifndef CMSIS_GPIO_H_
#define CMSIS_GPIO_H_

//#define STM32F7
#define STM32F3
//#define STM32F1
//#define STM32F0

#ifdef STM32F7
#include "stm32f7xx.h"
#endif

#ifdef STM32F4
#include "stm32f4xx.h"
#endif

#ifdef STM32F3
#include "stm32f3xx.h"
#endif

#ifdef STM32F1
#include "stm32f1xx.h"
#endif

#ifdef STM32F0
#include "stm32f0xx.h"
#endif

typedef enum AF
{
	AF0,
	AF1,
	AF2,
	AF3,
	AF4,
	AF5,
	AF6,
	AF7,
	AF8,
	AF9,
	AF10,
	AF11,
	AF12,
	AF13,
	AF14,
	AF15
} AF_t;

typedef enum OutputType
{
	PushPull = 0, OpenDrain
} OT_t;

typedef enum OutputState
{
	RESET_PIN = 0, SET_PIN
} OutputState_t;

typedef enum PullUpDownState
{
	NoPullUpPullDown = 0, PullUp, PullDown
} PullUpDownState_t;

typedef enum GpioSpeed
{

	LowSpeed = 0, MediumSpeed, HighSpeed, VeryHighSpeed
} GpioSpeed_t;

void GpioEnableClk(GPIO_TypeDef *PORT);
void GpioSetAsAF(GPIO_TypeDef *PORT, uint16_t gpio_pin, AF_t AF);
void GpioSetAsGpOutput(GPIO_TypeDef *PORT, uint16_t gpio_pin, OT_t OT);
void GpioSetAsGpInput(GPIO_TypeDef *PORT, uint16_t gpio_pin,PullUpDownState_t PupPdwn);
void GpioSetSpeed(GPIO_TypeDef *PORT, uint16_t gpio_pin, GpioSpeed_t SpeedType);
void GpioWrite(GPIO_TypeDef *PORT, uint16_t gpio_pin, OutputState_t state);
void GpioSetPin(GPIO_TypeDef *PORT, uint16_t gpio_pin);
void GpioResetPin(GPIO_TypeDef *PORT, uint16_t gpio_pin);

void GpioToggle(GPIO_TypeDef *PORT, uint16_t gpio_pin);
#endif /* CMSIS_GPIO_H_ */
