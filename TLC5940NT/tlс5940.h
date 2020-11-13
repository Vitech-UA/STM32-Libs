/*
 * tls5940.h
 *
 *  Created on: Aug 17, 2020
 *      Author: viktor.starovit
 */

#ifndef TLС5940_H_
#define TLС5940_H_

#include "stm32f767xx.h"
#include "main.h"
#include "tlc5940_conf.h"





#define BLANK_HIGH() HAL_GPIO_WritePin(BLANK_PORT, BLANK_PIN, GPIO_PIN_SET)
#define BLANK_LOW() HAL_GPIO_WritePin(BLANK_PORT, BLANK_PIN, GPIO_PIN_RESET)

#define XLAT_HIGH() HAL_GPIO_WritePin(XLAT_PORT, XLAT_PIN, GPIO_PIN_SET)
#define XLAT_LOW() HAL_GPIO_WritePin(XLAT_PORT, XLAT_PIN, GPIO_PIN_RESET)

//Запис нових даних кольору в мікросхему TLC5950

void TLC5940_Update();
void TLC5940_SetOneChanel(uint8_t channel, uint16_t val);
void TLC5940_Reset();
void TLC5940_Init(void);

void SetRGB_G1(int R, int G, int B);
void SetRGB_G2(int R, int G, int B);
void SetRGB_G3(int R, int G, int B);
void SetRGB_G4(int R, int G, int B);
void SetRGB_G5(int R, int G, int B);

//Массив даних у якому відбуватиметься зсув і запис нових значень
uint16_t leds[15];

volatile uint16_t clkCnt;

#endif /* TLС5940_H_ */
