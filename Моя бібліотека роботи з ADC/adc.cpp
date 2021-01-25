/*
 * adc.cpp
 *
 *  Created on: 22 янв. 2021 г.
 *      Author: viktor.starovit
 */
#include "adc.h"

Adc::Adc(ADC_IN_t channel) {
	//this->SetClockSource(ADC_PCLK_DivBy2);
	//this->Calibrate();
	//this->AddToScan(channel);

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // GPIOA clk enable.
	GPIOA->MODER |= GPIO_MODER_MODER1; // 1:1 - Analog

	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; //ADC1 clk enable
	this->SetClockSource(ADC_PCLK_DivBy4);

	this->Calibrate();

	ADC1->CR |= ADC_CR_ADEN;        // enable ADC
	ADC1->CFGR1 &= ~ADC_CFGR1_CONT; //0: Single conversion mode, 1: Continuous conversion mode

	ADC1->CFGR1 &= ~ADC_CFGR1_EXTEN;//00: Hardware trigger detection disabled (conversions can be started by software)

	ADC1->SMPR |= ADC_SMPR1_SMPR;   // 111: 239.5 ADC clock cycles

	ADC1->CHSELR |= ADC_CHSELR_CHSEL1; // Вибираю канал 1
}

void Adc::Init(ADC_IN_t channel) {

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

void Adc::SetClockSource(ADC_CLOCK_SOURCE_t ADC_CLK_MODE) {
	/* This code selects the HSI14 as clock source. */
	/* (1) Вмикаємо тактування ADC */
	/* (2) Запускаємо HSI14 */
	/* (3) Очікую готовності HSI14 */
	/* (4) Вибираю HSI14 записом 00 в CKMODE (скидаю бітове поле) */

	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; /* (1) */
	RCC->CR2 |= RCC_CR2_HSI14ON; /* (2) */
	while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0) /* (3) */
	{

	}
	ADC1->CFGR2 &= (~ADC_CFGR2_CKMODE); /* (4) */

}

void Adc::AddToScan(ADC_IN_t channel) {

	this->ADC_ITEM->CHSELR |= (1 << channel); // Вмикаю потрібний канал

}

