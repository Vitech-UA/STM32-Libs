/*
 * ControlModule.cpp
 *
 *  Created on: Jan 23, 2021
 *      Author: Embed Viktor
 */

#include <ControlModule.h>

ControlModule::ControlModule(SPI_TypeDef *SPI_Item) {

}

void ControlModule::InitMax7219(SPI_TypeDef *SPI_Item) {

}

void Led::SetState(LED_STATE_t LedState) {
	switch (LedState) {
	case LED_OFF:
		this->Port->BSRR |= (1 << this->Pin) << 16U; // Reset
		break;
	case LED_ON:
		this->Port->BSRR |= (1 << this->Pin); // Set
		break;
	}
}

void Led::Config(GPIO_TypeDef *Led_Port, uint16_t Led_Pin) {

#if (STM32Fx == 0)
	/*Присвоюю йому пін & порт*/
	this->Port = Led_Port;
	this->Pin = Led_Pin;
	/*Ввімкнути тактування обраного GPIO*/
	GpioEnableClk(Led_Port);
	/*Встановити як вихід пуш-пул*/
	this->Port->MODER |= (0x01 << (Pin * 2)); // GPIO->To output
	this->Port->PUPDR |= (0x01 << (Pin * 2)); // PullUp(Св.діод керується катодом)
	this->Port->BSRR |= (1 << this->Pin) << 16U;
#endif
}

void GpioEnableClk(GPIO_TypeDef *PORT) {
#if (STM32Fx == 0)
	if (PORT == GPIOA) {
		if (!(RCC->AHBENR & RCC_AHBENR_GPIOAEN))
			RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	} else if (PORT == GPIOB) {
		if (!(RCC->AHBENR & RCC_AHBENR_GPIOBEN))
			RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	} else if (PORT == GPIOC) {
		if (!(RCC->AHBENR & RCC_AHBENR_GPIOCEN))
			RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	} else if (PORT == GPIOD) {
		if (!(RCC->AHBENR & RCC_AHBENR_GPIOCEN))
			RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	} else if (PORT == GPIOF) {
		if (!(RCC->AHBENR & RCC_AHBENR_GPIOFEN))
			RCC->AHBENR |= RCC_AHBENR_GPIOFEN;
	}

#endif
}

void Button::Config(GPIO_TypeDef *ButtonPort, uint16_t ButtonPin) {
	this->Port = ButtonPort;
	this->Pin = ButtonPin;
	GpioEnableClk(ButtonPort);
}
BUTTON_STATE_t Button::GetState(void) {

	if (this->Port->IDR & (0x01 << this->Pin))
		return RELEASED;
	else
		return PRESSED;

}

