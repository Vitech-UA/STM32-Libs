/*
 * adc.cpp
 *
 *  Created on: 22 янв. 2021 г.
 *      Author: viktor.starovit
 */
#include "adc.h"

Adc::Adc(ADC_IN_t channel) {

	this->CurrentItemChannel = channel;

	//this->ConfigGpio(this->CurrentItemChannel);

	this->SetClockSource(ADC_PCLK_DivBy4);
	this->Calibrate();

	if (!(ADC1->CR & ADC_CR_ADEN)) {
		ADC1->CR |= ADC_CR_ADEN; // Вмикаю АЦП, якщо до цього часу він був вимкнений
	}
	ADC1->CFGR1 &= ~ADC_CFGR1_CONT; // 0: Single conversion mode, 1: Continuous conversion mode
	ADC1->CFGR1 &= ~ADC_CFGR1_EXTEN; // 00: Hardware trigger detection disabled (conversions can be started by software)
	ADC1->SMPR |= ADC_SMPR1_SMPR;       // 111: 239.5 ADC clock cycles

}

uint16_t Adc::GetValue(void) {

	uint16_t ADC_Result;

	ADC1->CHSELR &= ~(ADC_CHSELR_CHSEL);        // Вимикаю всі канали

	this->ConfigGpio(this->CurrentItemChannel); // Вмикаю лише один потрібний канал

	ADC1->CR |= ADC_CR_ADSTART;                 // пуск

	while (!(ADC1->ISR & ADC_ISR_EOC)) {

	}
	ADC_Result = ADC1->DR;

	return ADC_Result;                          //Вмпльовую результат
}

uint16_t Adc::GetMcuTemperature(void) {
	/* (0) Забороняю сканування всих каналів */
	/* (1) Вмикаю CHSEL16 для температурного сенсора */
	/* (2) виборка 239.5 тактів для перетворення в 17.1 мкС */
	/* (3) Дозвіл роботи сенсора температури (лише VBAT, Temp i VRefInt) */
	ADC1->CHSELR &= ~ADC_CHSELR_CHSEL; /* (0) */
	ADC1->CHSELR = ADC_CHSELR_CHSEL16; /* (1) */
	ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2; /* (2) */
	ADC1->CR |= ADC_CCR_TSEN; /* (3) */
	ADC1->CR |= ADC_CR_ADEN;
	ADC1->CR |= ADC_CCR_VREFEN;

	int32_t temperature;

	ADC1->CR |= ADC_CR_ADSTART;
	while ((ADC1->ISR & ADC_ISR_EOC) == 0);

	temperature = (((int32_t) ADC1->DR * VDD_APPLI / VDD_CALIB) - (int32_t) *TEMP30_CAL_ADDR);
	temperature = temperature * (int32_t) (110 - 30);
	temperature = temperature / (int32_t) (*TEMP110_CAL_ADDR - *TEMP30_CAL_ADDR);
	temperature = temperature + 30;
	return temperature;
}

void Adc::Calibrate() {
	/* (1) Переконуємось, що ADEN = 0 */
	/* (2) Виключаємо АЦП установкою ADDIS*/
	/* (3) Очищаємо DMAEN */
	/* (4) Запускаємо калібровку установкою ADCAL */
	/* (5) очікуємо поки ADCAL=0 */
	if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
	{
		ADC1->CR |= ADC_CR_ADDIS; /* (2) */
	}
	while ((ADC1->CR & ADC_CR_ADEN) != 0) {

	}
	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; /* (3) */
	ADC1->CR |= ADC_CR_ADCAL; /* (4) */
	while ((ADC1->CR & ADC_CR_ADCAL) != 0) /* (5) */
	{

	}
}

void Adc::ConfigGpio(ADC_IN_t channel) {

	switch (channel) {
	case IN0:
		if (!(RCC->AHBENR & RCC_AHBENR_GPIOAEN)) {
			RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // GPIOA clk enable.
		}
		GPIOA->MODER |= GPIO_MODER_MODER0; // PA0, 1:1 - Analog
		ADC1->CHSELR |= ADC_CHSELR_CHSEL0; // Вибираю канал 0
		break;
	case IN1:
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // GPIOA clk enable.
		GPIOA->MODER |= GPIO_MODER_MODER1; // PA1, 1:1 - Analog
		ADC1->CHSELR |= ADC_CHSELR_CHSEL1; // Вибираю канал 1
		break;
	case IN2:
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // GPIOA clk enable.
		GPIOA->MODER |= GPIO_MODER_MODER2; // PA2, 1:1 - Analog
		ADC1->CHSELR |= ADC_CHSELR_CHSEL2; // Вибираю канал 2
		break;
	case IN3:
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // GPIOA clk enable.
		GPIOA->MODER |= GPIO_MODER_MODER3; // PA3, 1:1 - Analog
		ADC1->CHSELR |= ADC_CHSELR_CHSEL3; // Вибираю канал 3
		break;
	case IN4:
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // GPIOA clk enable.
		GPIOA->MODER |= GPIO_MODER_MODER4; // PA4, 1:1 - Analog
		ADC1->CHSELR |= ADC_CHSELR_CHSEL4; // Вибираю канал 4
		break;
	case IN5:
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // GPIOA clk enable.
		GPIOA->MODER |= GPIO_MODER_MODER5; // PA5, 1:1 - Analog
		ADC1->CHSELR |= ADC_CHSELR_CHSEL5; // Вибираю канал 5
		break;
	case IN6:
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // GPIOA clk enable.
		GPIOA->MODER |= GPIO_MODER_MODER6; // PA6, 1:1 - Analog
		ADC1->CHSELR |= ADC_CHSELR_CHSEL6; // Вибираю канал 6
		break;
	case IN7:
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // GPIOA clk enable.
		GPIOA->MODER |= GPIO_MODER_MODER7; // PA7, 1:1 - Analog
		ADC1->CHSELR |= ADC_CHSELR_CHSEL7; // Вибираю канал 7
		break;
	case IN8:
		RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // GPIOB clk enable.
		GPIOB->MODER |= GPIO_MODER_MODER0; // PB0, 1:1 - Analog
		ADC1->CHSELR |= ADC_CHSELR_CHSEL8; // Вибираю канал 8
		break;
	case IN9:
		RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // GPIOB clk enable.
		GPIOB->MODER |= GPIO_MODER_MODER1; // PB1, 1:1 - Analog
		ADC1->CHSELR |= ADC_CHSELR_CHSEL9; // Вибираю канал 9
		break;
	case IN10:
		RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // GPIOC clk enable.
		GPIOB->MODER |= GPIO_MODER_MODER0; // PC0, 1:1 - Analog
		ADC1->CHSELR |= ADC_CHSELR_CHSEL10; // Вибираю канал 10
		break;
	case IN11:
		RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // GPIOC clk enable.
		GPIOB->MODER |= GPIO_MODER_MODER1; // PC1, 1:1 - Analog
		ADC1->CHSELR |= ADC_CHSELR_CHSEL11; // Вибираю канал 11
		break;
	case IN12:
		RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // GPIOC clk enable.
		GPIOB->MODER |= GPIO_MODER_MODER2; // PC2, 1:1 - Analog
		ADC1->CHSELR |= ADC_CHSELR_CHSEL12; // Вибираю канал 12
		break;
	case IN13:
		RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // GPIOC clk enable.
		GPIOB->MODER |= GPIO_MODER_MODER2; // PC3, 1:1 - Analog
		ADC1->CHSELR |= ADC_CHSELR_CHSEL13; // Вибираю канал 13
		break;
	case IN14:
		RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // GPIOC clk enable.
		GPIOB->MODER |= GPIO_MODER_MODER4; // PC4, 1:1 - Analog
		ADC1->CHSELR |= ADC_CHSELR_CHSEL14; // Вибираю канал 14
		break;
	case IN15:
		RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // GPIOC clk enable.
		GPIOB->MODER |= GPIO_MODER_MODER5; // PC5, 1:1 - Analog
		ADC1->CHSELR |= ADC_CHSELR_CHSEL15; // Вибираю канал 15
		break;

	default:
		break;
	}

}

void Adc::SetClockSource(ADC_CLOCK_SOURCE_t ADC_CLK_MODE) {
	/* This code selects the HSI14 as clock source. */
	/* (1) Вмикаємо тактування ADC */
	/* (2) Запускаємо HSI14 */
	/* (3) Очікую готовності HSI14 */
	/* (4) Вибираю HSI14 записом 00 в CKMODE (скидаю бітове поле) */

	if (!(RCC->APB2ENR & RCC_APB2ENR_ADC1EN)) { /* (1) */
		RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Вмикаю тактування АЦП, якщо до цього воно було вимкнене
	}
	RCC->CR2 |= RCC_CR2_HSI14ON; /* (2) */
	while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0) /* (3) */
	{

	}
	ADC1->CFGR2 &= (~ADC_CFGR2_CKMODE); /* (4) */

}

void Adc::AddToScan(ADC_IN_t channel) {

	this->ADC_ITEM->CHSELR |= (1 << channel); // Вмикаю потрібний канал

}
