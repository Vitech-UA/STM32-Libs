/*
 * tsc.c
 *
 *  Created on: 30 нояб. 2020 г.
 *      Author: viktor.starovit
 *
 *
 */

#include "tsc.h"

void InitTscModule(void) {

	RCC->AHBENR |= RCC_AHBENR_TSEN;

	TSC->CR &= ~(TSC_CR_PGPSC); // Максимальна частота тактування модуля TSC
	/*
	 PGPSC[2:0]- подільник тактування TSC.
	 000: f HCLK <-
	 001: f HCLK /2
	 010: f HCLK /4
	 011: f HCLK /8
	 100: f HCLK /16
	 101: f HCLK /32
	 110: f HCLK /64
	 111: f HCLK /128
	 */

	TSC->CR |= TSC_CR_CTPH; // 16x t PGCLK
	/*
	 CTPH[3:0] - Час заряду ємнісного сенсора дотику.
	 0000: 1x t PGCLK
	 0001: 2x t PGCLK !!
	 ...
	 1111: 16x t PGCLK
	 */
	TSC->CR |= TSC_CR_CTPL; // 16x t PGCLK
	/*
	 CTPL[3:0]-Час передачі заряду від ємнісного сенсора дотику, до накопичувального конденсатора.
	 0000: 1x t PGCLK
	 0001: 2x t PGCLK !!!
	 ...
	 1111: 16x t PGCLK
	 */

	TSC->CR |= TSC_CR_MCV_2;// | TSC_CR_MCV_1;

	/*
	 MCV[2:0]-максимальна к-ть передач зарядів з ємнісного сенсору дотику в конденсатор-накопичувач,
	 після переповнення вказаного значення сканування зкануваняя зупиняється і спрацьовує флаг переповнення
	 000: 255
	 001: 511
	 010: 1023
	 011: 2047
	 100: 4095
	 101: 8191
	 110: 16383 !!!
	 111: not USE
	 */

	TSC->CR |= TSC_CR_TSCE; /* Вмикаємо модуль TSC */

	/* Вмикаю накопичувальні конденсатори */
	TSC->IOSCR = TSC_IOSCR_G2_IO4; // PA7 - Cs2 (TSC_G2IO4)
	TSC->IOSCR |= TSC_IOASCR_G1_IO1; // PA0 - Cs1 (TSC_G1IO1)

	/* Вмикаю ємнісні електроди */
	TSC->IOCCR = TSC_IOCCR_G2_IO3; // PA6 -  4  (TSC_G2IO3)
	TSC->IOCCR = TSC_IOCCR_G2_IO2; // PA5 -  3  (TSC_G2IO2)
	TSC->IOCCR = TSC_IOCCR_G2_IO1; // PA4 -  1  (TSC_G2IO1)
	TSC->IOCCR |= TSC_IOCCR_G1_IO4;  // PA3 -  2  (TSC_G1IO4)

	TSC->ICR |= (TSC_ICR_EOAIC) | (TSC_ICR_MCEIC); // Чистим флаги переривань
	// Вимикаю опитування першої і другої групи сенсорів
	TSC->IOGCSR |= (TSC_IOGCSR_G1E) | (TSC_IOGCSR_G2E);

	// Вимикаю трігери шмідта для виводів першої групи
	TSC->IOHCR &= ~((TSC_IOHCR_G1_IO1) | (TSC_IOHCR_G1_IO2) | (TSC_IOHCR_G1_IO3)
			| (TSC_IOHCR_G1_IO4));

	// Вимикаю трігери шмідта для виводів другої групи
	TSC->IOHCR &= ~((TSC_IOHCR_G2_IO1) | (TSC_IOHCR_G2_IO2) | (TSC_IOHCR_G2_IO3)
			| (TSC_IOHCR_G2_IO4));

}

void InitTscGpio(void) {

	//-Capacitors-//
	// PA0 - Cs1 (TSC_G1IO1)
	// PA7 - Cs2 (TSC_G2IO4)

	//-Electrodes-//
	// PA3 -  2  (TSC_G1IO4)
	// PA4 -  1  (TSC_G2IO1)
	// PA5 -  3  (TSC_G2IO2)
	// PA6 -  4  (TSC_G2IO3)

	/* Enable the peripheral clock of GPIOA */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	/* (1) Open drain for sampling */
	GPIOA->OTYPER |= GPIO_OTYPER_OT_0;
	GPIOA->OTYPER |= GPIO_OTYPER_OT_7;

	//Enable High speed mode for sampling capacitors GPIO
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7;

	/* (2) PP for channel */
	GPIOA->OTYPER &= ~ GPIO_OTYPER_OT_4;
	GPIOA->OTYPER &= ~ GPIO_OTYPER_OT_3;
	GPIOA->OTYPER &= ~ GPIO_OTYPER_OT_5;
	GPIOA->OTYPER &= ~ GPIO_OTYPER_OT_6;

	/* (3) Select AF mode (10) on PA0, PA3, PA4, PA5, PA6, PA7 */
	GPIOA->MODER |= GPIO_MODER_MODER0_1;
	GPIOA->MODER |= GPIO_MODER_MODER3_1;
	GPIOA->MODER |= GPIO_MODER_MODER4_1;
	GPIOA->MODER |= GPIO_MODER_MODER5_1;
	GPIOA->MODER |= GPIO_MODER_MODER6_1;
	GPIOA->MODER |= GPIO_MODER_MODER7_1;
	/* (4) AF3 for TSC signals */

	GPIOA->AFR[0] |= (1 << 0) | (1 << 1);   //PA0  - AF3
	GPIOA->AFR[0] |= (1 << 12) | (1 << 13); //PA3  - AF3
	GPIOA->AFR[0] |= (1 << 16) | (1 << 17); //PA4  - AF3
	GPIOA->AFR[0] |= (1 << 20) | (1 << 21); //PA5  - AF3
	GPIOA->AFR[0] |= (1 << 24) | (1 << 25); //PA6  - AF3
	GPIOA->AFR[0] |= (1 << 28) | (1 << 29); //PA7  - AF3

}

void InitOutputGpio(void) {
	// PB12, PB13, PB14, PB15;

	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enable PB_CLK

	// PB12 - Output
	GPIOB->MODER |= GPIO_MODER_MODER12_0; // General purporse output mode
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_12;  // Output PushPull

	// PB13 - Output
	GPIOB->MODER |= GPIO_MODER_MODER13_0; // General purporse output mode
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_13;  // Output PushPull

	// PB14 - Output
	GPIOB->MODER |= GPIO_MODER_MODER14_0; // General purporse output mode
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_14;  // Output PushPull

	// PB15 - Output
	GPIOB->MODER |= GPIO_MODER_MODER15_0; // General purporse output mode
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_15;  // Output PushPull

	//

}

bool Button1IsPressed(void) {
	int sensor_result = 0;
	TSC->IOCCR = TSC_IOCCR_G2_IO1; // На виводі G2_IO1 (PA4) ввімкнено сенсор.
	TSC->CR |= TSC_CR_START;       // Старт сканування.
	while ( TSC->ISR == 0)
		;                          // Чекаю очікування сканування.
	sensor_result = TSC->IOGXCR[1];
	TSC->IOCCR = 0x0;               //Вимикаю всі сенсори.
	TSC->ICR |= 0x3;                //Скидаю флаги переривання.
	for (uint32_t i = 50; i > 0; i--)
		;
	if (sensor_result < BUTTON_1_TRESHOLD) {
		return true;
	} else {
		return false;
	}
}

bool Button2IsPressed(void) {
	int sensor_result = 0;
	TSC->IOCCR = TSC_IOCCR_G1_IO4; // На виводі G1_IO4 (PA3) ввімкнено сенсор.
	TSC->CR |= TSC_CR_START;       // Старт сканування.
	while ( TSC->ISR == 0)
		;           // Чекаю очікування сканування.
	sensor_result = TSC->IOGXCR[0];
	TSC->IOCCR = 0x0;               //Вимикаю всі сенсори.
	TSC->ICR |= 0x3;                //Скидаю флаги переривання.
	for (uint32_t i = 50; i > 0; i--)
		;
	if (sensor_result < BUTTON_2_TRESHOLD) {
		return true;
	} else {
		return false;
	}
}

bool Button3IsPressed(void) {
	int sensor_result = 0;
	TSC->IOCCR = TSC_IOCCR_G2_IO3; // На виводі G2_IO3 (PA6) ввімкнено сенсор.
	TSC->CR |= TSC_CR_START;       // Старт сканування.
	while ( TSC->ISR == 0)
		;                          // Чекаю очікування сканування.
	sensor_result = TSC->IOGXCR[1];
	TSC->IOCCR = 0x0;               //Вимикаю всі сенсори.
	TSC->ICR |= 0x3;                //Скидаю флаги переривання.
	for (uint32_t i = 50; i > 0; i--)
		;
	if (sensor_result < BUTTON_3_TRESHOLD) {
		return true;
	} else {
		return false;
	}
}

bool Button4IsPressed(void) {
	int sensor_result = 0;
	TSC->IOCCR = TSC_IOCCR_G2_IO2; // На виводі G2_IO2 (PA5) ввімкнено сенсор.
	TSC->CR |= TSC_CR_START;       // Старт сканування.
	while ( TSC->ISR == 0)
		;                          // Чекаю очікування сканування.
	sensor_result = TSC->IOGXCR[1];
	TSC->IOCCR = 0x0;               //Вимикаю всі сенсори.
	TSC->ICR |= 0x3;                //Скидаю флаги переривання.
	for (uint32_t i = 50; i > 0; i--)
		;                //Пауза для стабилизации.
	if (sensor_result < BUTTON_4_TRESHOLD) {
		return true;
	} else {
		return false;
	}
}
