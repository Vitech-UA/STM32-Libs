/*
 * Gpio.cpp
 *
 *  Created on: 20 янв. 2021 г.
 *      Author: viktor.starovit
 */

#include "stm32f0xx.h"
#include "Gpio.h"

Gpio::Gpio(GPIO_TypeDef *PORT, uint16_t gpio_pin) {
	// Ініціалізація піна
	item_port = PORT;
	item_pin = gpio_pin;
	this->GpioEnableClk(); // Ввімкнути тактування вибраного порта

}
void Gpio::GpioEnableClk() {

	GPIO_TypeDef *PORT = this->item_port;
// Перед спробою ввімкнення тактування проходить перевірка, чи не було воно ввімкнене раніше.
	if (PORT == GPIOA) {
		if (!(RCC->AHBENR & RCC_AHBENR_GPIOAEN)) {
			RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
		}
	} else if (PORT == GPIOB) {
		if (!(RCC->AHBENR & RCC_AHBENR_GPIOBEN)) {
			RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
		}
	} else if (PORT == GPIOC) {
		if (!(RCC->AHBENR & RCC_AHBENR_GPIOCEN)) {
			RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
		}
	} else if (PORT == GPIOD) {
		if (!(RCC->AHBENR & RCC_AHBENR_GPIODEN)) {
			RCC->AHBENR |= RCC_AHBENR_GPIODEN;
		}
	}
}

void Gpio::GpioSetOutputType(OUTPUT_TYPE_t GpioOutputType) {
	this->item_port->OTYPER |= GpioOutputType << GpioOutputType;
}
void Gpio::GpioSetOutputSpeed(OUT_SPEED_t GpioOutputSpeed) {

	this->item_port->OSPEEDR |= (GpioOutputSpeed << (this->item_pin * 2));
}
void Gpio::Set(void) {
	this->item_port->BSRR |= (1 << this->item_pin); // BIT SET
}
void Gpio::Reset(void) {
	this->item_port->BSRR |= ((1 << this->item_pin) << 16U);
}
void Gpio::Toggle(void) {
	this->item_port->ODR ^= (1 << this->item_pin);
}
void Gpio::SetAsAF(GPIO_AF_t AlernateFunction) {

	if (this->item_pin <= 7) // For gpio0..gpio7
			{
		this->item_port->MODER |= 0x02 << (this->item_pin * 2);
		this->item_port->AFR[0] |= AlernateFunction << (this->item_pin * 4);
	} else if (this->item_pin > 7 && this->item_pin <= 15) // For gpio8..gpio15
			{
		this->item_port->MODER |= 0x02 << (this->item_pin * 2);
		this->item_port->AFR[1] |= AlernateFunction
				<< ((this->item_pin - 8) * 4);
	} else // For gpio.unkown
	{
		while (1) {
			// Infinite error loop :-(
		}
	}
}
void Gpio::SetAsAF(GPIO_AF_t AlernateFunction, OUTPUT_TYPE_t GpioOutputType) {
	this->GpioSetOutputType(GpioOutputType); //Set Output Type PP | OD
	this->GpioSetOutputSpeed(HIGH_SPEED);    //Default hi speed
	if (this->item_pin <= 7) // For gpio0..gpio7
			{
		this->item_port->MODER |= 0x02 << (this->item_pin * 2);
		this->item_port->AFR[0] |= AlernateFunction << (this->item_pin * 4);
	} else if (this->item_pin > 7 && this->item_pin <= 15) // For gpio8..gpio15
			{
		this->item_port->MODER |= 0x02 << (this->item_pin * 2);
		this->item_port->AFR[1] |= AlernateFunction
				<< ((this->item_pin - 8) * 4);
	} else // For gpio.unkown
	{
		while (1) {
			// Infinite error loop :-(
		}
	}
}
void Gpio::SetAsGenerapPurporseOutput(OUTPUT_TYPE_t GpioOutputType) {
	this->item_port->MODER |= 0x01 << (this->item_pin * 2);
	this->item_port->OTYPER |= GpioOutputType << this->item_pin;
}
void Gpio::SetAsInput(GPIO_PU_PD_t GpioPullUp_PullDown) {
	this->GpioSetOutputSpeed(HIGH_SPEED);
	this->item_port->MODER &= ~(0x03 << (this->item_pin * 2)); // Скидання бітового поля, відповідного піна призведе до його конфігурування як вхід
	this->item_port->PUPDR |= (GpioPullUp_PullDown << (this->item_pin * 2));

}
void Gpio::Deinit(void) {
	this->item_port->MODER &= ~(0x03 << (this->item_pin * 2)); // Скидання бітового поля MODER
	this->item_port->OTYPER &= ~(0x01 << this->item_pin);
	this->item_port->OSPEEDR &= ~(0x03 << (this->item_pin * 2));
	this->item_port->PUPDR &= ~(0x03 << (this->item_pin * 2));
}

bool Gpio::IsSet(void) {

	bool PinState;
	if (this->item_port->IDR & (0x01 << this->item_pin))
		PinState = true;
	else
		PinState = false;

	return PinState;

}
