/*
 * cmsis_gpio.c
 *
 *  Created on: 14 дек. 2020 г.
 *      Author: viktor.starovit
 */

#include "cmsis_gpio.h"

void GpioEnableClk(GPIO_TypeDef *PORT) {
#ifdef STM32F7
	switch ((int) PORT)
	{
	case (int) GPIOA:
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
		break;
	case (int) GPIOB:
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
		break;
	case (int) GPIOC:
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
		break;
	case (int) GPIOD:
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
		break;
	case (int) GPIOE:
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
		break;
	case (int) GPIOF:
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
		break;
	case (int) GPIOG:
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
		break;
	case (int) GPIOH:
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
		break;
	case (int) GPIOI:
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;
		break;
	default:
		break;
	}
#endif

#ifdef STM32F3
	switch ((int) PORT) {
	case (int) GPIOA:
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
		break;
	case (int) GPIOB:
		RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
		break;
	case (int) GPIOC:
		RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
		break;
	case (int) GPIOD:
		RCC->AHBENR |= RCC_AHBENR_GPIODEN;
		break;
	default:
		break;
	}
#endif

}

void GpioSetAsAF(GPIO_TypeDef *PORT, uint16_t gpio_pin, AF_t AF) {

	if (gpio_pin <= 7) // For gpio.0 ... gpio.7
			{
		PORT->MODER |= 0x02 << (gpio_pin * 2);
		PORT->AFR[0] |= AF << (gpio_pin * 4);
	} else if (gpio_pin > 7 && gpio_pin <= 15) // For gpio.8 ... gpio.15
			{
		PORT->MODER |= 0x02 << (gpio_pin * 2);
		PORT->AFR[1] |= AF << ((gpio_pin - 8) * 4);
	} else // For gpio.unkown
	{
		while (1) {
			// Infinite error loop :-(
		}
	}

}

void GpioSetAsGpOutput(GPIO_TypeDef *PORT, uint16_t gpio_pin, OT_t OT) {
	PORT->MODER |= 0x01 << (gpio_pin * 2);
	PORT->OTYPER |= OT << gpio_pin;
}
void GpioSetAsGpInput(GPIO_TypeDef *PORT, uint16_t gpio_pin,
		PullUpDownState_t PupPdwn) {
	PORT->MODER &= ~(0x03 << (gpio_pin * 2)); // Скидання бітового поля, відповідного піна призведе до його конфігурування як вхід
	PORT->PUPDR |= PupPdwn << (gpio_pin * 2);
}

void GpioToggle(GPIO_TypeDef *PORT, uint16_t gpio_pin) {
	PORT->ODR ^= (1 << gpio_pin);                 // Toggle ODR_Sate
}

void GpioWrite(GPIO_TypeDef *PORT, uint16_t gpio_pin, OutputState_t state) {
	if (state == SET_PIN) {
		PORT->BSRR |= (1 << gpio_pin);           // BIT SET
	} else {
#ifdef STM32F3
		//PORT->BRR |= (1<<gpio_pin); // Варіант 1
		PORT->BSRR |= ((1 << gpio_pin) << 16U); // Варіант 2
#endif

#ifdef STM32F7
		PORT->BSRR |= ((1 << gpio_pin) << 16U); // Варіант 2
#endif

	}
}

void GpioSetPin(GPIO_TypeDef *PORT, uint16_t gpio_pin) {
	PORT->BSRR |= (1 << gpio_pin);               // BIT SET
}

void GpioResetPin(GPIO_TypeDef *PORT, uint16_t gpio_pin) {

#ifdef STM32F3
	//PORT->BRR |= (1<<gpio_pin); // Варіант 1
	PORT->BSRR |= ((1 << gpio_pin) << 16U); // Варіант 2
#endif

#ifdef STM32F7
	PORT->BSRR |= (gpio_pin << 16U);          // BIT Reset
#endif
}

void GpioSetSpeed(GPIO_TypeDef *PORT, uint16_t gpio_pin, GpioSpeed_t SpeedType) {
	PORT->OSPEEDR |= SpeedType << (gpio_pin * 2);
}
