/*
 * tlc5940.c
 *
 *  Created on: Aug 17, 2020
 *      Author: viktor.starovit
 */

#include <tlс5940.h>
#include "stm32f767xx.h"

extern SPI_HandleTypeDef TLC5940_SPI_PORT;
extern TIM_HandleTypeDef TLC5940_PWM_TIMER;

void TLC5940_SetOneChanel(uint8_t channel, uint16_t val) {
	leds[channel] = (val & 0x0FFF);
}

//Затримка по-таймеру в мілісекундах
void delay(uint16_t d) {
	currentDelay = d;
	while (currentDelay)
		;
}

void TLC5940_Update() {

	//Вимикаємо генерацію ШИМ на пін GSCLK
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);

	BLANK_HIGH();
	delay(1);
	int8_t i;
	for (i = 15; i >= 0; i -= 2) {
		uint8_t send1 = 0;
		uint8_t send = leds[i] >> 4;
		HAL_SPI_Transmit(&hspi1, (uint8_t*) &send, 1, 100);

		send = (leds[i] & 0x000F);
		send <<= 4;
		send1 = (leds[i - 1]) >> 8;

		send |= send1;
		HAL_SPI_Transmit(&hspi1, (uint8_t*) &send, 1, 100);

		send = leds[i - 1];
		HAL_SPI_Transmit(&hspi1, (uint8_t*) &send, 1, 100);
	}

	delay(1);
	XLAT_HIGH();
	delay(1);
	BLANK_LOW();
	BLANK_HIGH();
	delay(1);
	clkCnt = 0;
	//Вмикаємо генерацію ШИМ на пін GSCLK
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

}

void TLC5940_Reset() // Всі канали в 0
{
	uint8_t i;
	for (i = 0; i < 16; i++) {
		leds[i] = 0x0000;
	}
	TLC5940_Update();
}

void TLC5940_Init(void) {
	clkCnt = 0;
	//Стартовий Сетап
	BLANK_LOW();
	//Вимикаю всі виходи
	TLC5940_Reset();

}

void SetRGB_G1(int R, int G, int B) {

	if (R >= MAX_PWM_VALUE)
		R = MAX_PWM_VALUE;
	if (G >= MAX_PWM_VALUE)
		G = MAX_PWM_VALUE;
	if (B >= MAX_PWM_VALUE)
		B = MAX_PWM_VALUE;

	TLC5940_SetOneChanel(1, R);

	TLC5940_SetOneChanel(2, G);

	TLC5940_SetOneChanel(3, B);

}

void SetRGB_G2(int R, int G, int B) {
	if (R >= MAX_PWM_VALUE)
		R = MAX_PWM_VALUE;
	if (G >= MAX_PWM_VALUE)
		G = MAX_PWM_VALUE;
	if (B >= MAX_PWM_VALUE)
		B = MAX_PWM_VALUE;

	TLC5940_SetOneChanel(4, R);

	TLC5940_SetOneChanel(5, G);

	TLC5940_SetOneChanel(6, B);

}

void SetRGB_G3(int R, int G, int B) {
	if (R >= MAX_PWM_VALUE)
		R = MAX_PWM_VALUE;
	if (G >= MAX_PWM_VALUE)
		G = MAX_PWM_VALUE;
	if (B >= MAX_PWM_VALUE)
		B = MAX_PWM_VALUE;

	TLC5940_SetOneChanel(7, R);

	TLC5940_SetOneChanel(8, G);

	TLC5940_SetOneChanel(9, B);

}

void SetRGB_G4(int R, int G, int B) {
	if (R >= MAX_PWM_VALUE)
		R = MAX_PWM_VALUE;
	if (G >= MAX_PWM_VALUE)
		G = MAX_PWM_VALUE;
	if (B >= MAX_PWM_VALUE)
		B = MAX_PWM_VALUE;

	TLC5940_SetOneChanel(10, R);

	TLC5940_SetOneChanel(11, G);

	TLC5940_SetOneChanel(12, B);

}

void SetRGB_G5(int R, int G, int B) {
	if (R >= MAX_PWM_VALUE)
		R = MAX_PWM_VALUE;
	if (G >= MAX_PWM_VALUE)
		G = MAX_PWM_VALUE;
	if (B >= MAX_PWM_VALUE)
		B = MAX_PWM_VALUE;

	TLC5940_SetOneChanel(13, R);

	TLC5940_SetOneChanel(14, G);

	TLC5940_SetOneChanel(15, B);

}
void TLC5940_Test(int del) {
	SetRGB_G1(0, 0, MAX_PWM_VALUE);
	TLC5940_Update();
	HAL_Delay(del);
	SetRGB_G1(0, MAX_PWM_VALUE, 0);
	TLC5940_Update();
	HAL_Delay(del);
	SetRGB_G1(MAX_PWM_VALUE, 0, 0);
	TLC5940_Update();
	HAL_Delay(del);
	SetRGB_G1(0, 0, 0);
	TLC5940_Update();
	HAL_Delay(del);

	SetRGB_G2(0, 0, MAX_PWM_VALUE);
	TLC5940_Update();
	HAL_Delay(del);
	SetRGB_G2(0, MAX_PWM_VALUE, 0);
	TLC5940_Update();
	HAL_Delay(del);
	SetRGB_G2(MAX_PWM_VALUE, 0, 0);
	TLC5940_Update();
	HAL_Delay(del);
	SetRGB_G2(0, 0, 0);
	TLC5940_Update();
	HAL_Delay(del);

	SetRGB_G3(0, 0, MAX_PWM_VALUE);
	TLC5940_Update();
	HAL_Delay(del);
	SetRGB_G3(0, MAX_PWM_VALUE, 0);
	TLC5940_Update();
	HAL_Delay(del);
	SetRGB_G3(MAX_PWM_VALUE, 0, 0);
	TLC5940_Update();
	HAL_Delay(del);
	SetRGB_G3(0, 0, 0);
	TLC5940_Update();
	HAL_Delay(del);

	SetRGB_G4(0, 0, MAX_PWM_VALUE);
	TLC5940_Update();
	HAL_Delay(del);
	SetRGB_G4(0, MAX_PWM_VALUE, 0);
	TLC5940_Update();
	HAL_Delay(del);
	SetRGB_G4(MAX_PWM_VALUE, 0, 0);
	TLC5940_Update();
	HAL_Delay(del);
	SetRGB_G4(0, 0, 0);
	TLC5940_Update();
	HAL_Delay(del);

	SetRGB_G5(0, 0, MAX_PWM_VALUE);
	TLC5940_Update();
	HAL_Delay(del);
	SetRGB_G5(0, MAX_PWM_VALUE, 0);
	TLC5940_Update();
	HAL_Delay(del);
	SetRGB_G5(MAX_PWM_VALUE, 0, 0);
	TLC5940_Update();
	HAL_Delay(del);
	SetRGB_G5(0, 0, 0);
	TLC5940_Update();
	HAL_Delay(del);
}
