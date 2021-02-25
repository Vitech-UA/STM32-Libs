/*
 * ControlModule.cpp
 *
 *  Created on: Jan 23, 2021
 *      Author: Embed Viktor
 */

#include <ControlModule.h>
#include <math.h>
#include <SPI.h>
//            'град'   't'    'F'  'E'    'r'  ' '   'S'
char CHAR[9] = { 0x63, 0x0F, 0x47, 0x4F, 0x05, 0x00, 0x6D };
/*                0    1      2     3     4     5     6    7     8      9 */
char NUM[10] = { 0x7E, 0x30, 0x6D, 0x79, 0x33, 0x5B, 0x5F, 0x70, 0x7F, 0x7B };

/*************************MAX7219*********************************/
ControlModule::ControlModule(SPI_TypeDef *Port) :
		SPI(Port) {

	this->nCS_High();
	this->Init();

}
void ControlModule::Init() {

	this->nCS_Low();
	this->Transmit(0x09, 0x00);
	this->Transmit(0x0B, this->dg - 1);
	this->Transmit(0x0A, 0x02);
	this->Transmit(0x0C, 0x01);
	this->Clear();
}

void Send2Byte(uint16_t data) {

}

void ControlModule::Transmit(uint8_t rg, uint8_t dt) {

	this->nCS_Low();
	this->SPI_ITEM->DR = (uint16_t) ((rg << 8) | dt);
	while ((this->SPI_ITEM->SR & SPI_SR_BSY) == SPI_SR_BSY)
		;
	this->nCS_High();
}
void ControlModule::Clear(void) {
	uint8_t i = this->dg;
	do {
		this->Transmit(i, 0x00);
	} while (--i);

}
void ControlModule::PrintInt(volatile uint16_t number) {
	volatile uint16_t n = 0;
	n = number;
	if (n >= 9999)
		n = 9999;
	if (n <= 0)
		n = 0;

	uint8_t DIG1 = n % 10000 / 1000;
	uint8_t DIG2 = n % 1000 / 100;
	uint8_t DIG3 = n % 100 / 10;
	uint8_t DIG4 = n % 10;

	this->Transmit(3, NUM[DIG1]);
	this->Transmit(2, NUM[DIG2]);
	this->Transmit(1, NUM[DIG3]);
	this->Transmit(4, NUM[DIG4]);

}

void ControlModule::PrintFloat(float number) {

	float Buffer = number;
	int Left = Buffer;
	float Drob = Buffer - Left;

	//Виводжу цілу частину
	uint8_t DIG3 = Left % 100 / 10;
	uint8_t DIG4 = Left % 10;
	this->Transmit(3, NUM[DIG3]);
	this->Transmit(2, NUM[DIG4] | 0x80);

	//Вивід дробної частини
	int Right = 0;

	Right = round(Drob * 100); // Округлюємо оскільки при діленні на число кратне 3 виникають небажані ефекти
	DIG3 = (int) Right % 100 / 10;
	DIG4 = (int) Right % 10;
	this->Transmit(1, NUM[DIG3]);
	this->Transmit(4, NUM[DIG4]);

}

void ControlModule::PrintChar(uint8_t rg, uint8_t dt) {
	switch (rg) {
	case 0:
		this->Transmit(3, dt);
		break;
	case 1:
		this->Transmit(2, dt);
		break;
	case 2:
		this->Transmit(1, dt);
		break;
	case 3:
		this->Transmit(4, dt);
		break;
	}

}

void ControlModule::SetBrightness(uint8_t Intensity) {
	if (Intensity > 15)
		Intensity = 15;
	if (Intensity < 0)
		Intensity = 0;
	this->Transmit(0x0A, Intensity);

}
/*************************LED************************************/
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

/*************************BUTTON************************************/
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

/*************************BUZZER************************************/
void Buzzer::Config(GPIO_TypeDef *BuzzerPort, uint16_t BuzzerPin) {
	this->Port = BuzzerPort;
	this->Pin = BuzzerPin;
	GpioEnableClk(BuzzerPort);

	/*Встановити як вихід пуш-пул*/
	this->Port->MODER |= (0x01 << (this->Pin * 2)); // GPIO->To output
	this->Port->PUPDR |= (0x01 << (this->Pin * 2)); // PullUp(Св.діод керується катодом)
	this->Port->BSRR |= (1 << this->Pin) << 16U;

}
void Buzzer::SetState(BUZZER_STATE_t BuzzerState) {
	switch (BuzzerState) {
	case ON:
		this->Port->BSRR |= (1 << this->Pin); // Reset
		break;
	case OFF:
		this->Port->BSRR |= (1 << this->Pin) << 16U; // Reset
		break;
	}
}

/*************************COMMON************************************/
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
