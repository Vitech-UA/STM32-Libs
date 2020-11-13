/**
*	TechMaker
*	https://techmaker.ua
*
*	STM32 LCD TFT Library for displays using SPI interface
*	based on Adafruit GFX & Adafruit TFT LCD libraries
*	15 Jan 2018 by Alexander Olenyev <sasha@techmaker.ua>
*
*	Changelog:
*		- v1.0 added support for ST7735 chips
*/

// Graphics library by ladyada/adafruit with init code from Rossum
// MIT license

/*
This is the core graphics library for all our displays, providing a common
set of graphics primitives (points, lines, circles, etc.).  It needs to be
paired with a hardware-specific library for each display device we carry
(to handle the lower-level functions).
Adafruit invests time and resources providing this open source code, please
support Adafruit & open-source hardware by purchasing products from Adafruit!
Copyright (c) 2013 Adafruit Industries.  All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __LCD_H
#define __LCD_H

#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include "registers.h"
#include "fonts.h"
#include "stm32f7xx_hal.h"

// Please uncomment one of the lines to select your LCD chip
#define ST7735			// works!!

// Please uncomment to draw BMP from SD Card
//#define USE_FATFS

#if !(defined(ST7735))
#error Please select your LCD chip in lcd.h, lines 56-58
#endif

#if defined (USE_FATFS)
#include "ff.h"
#endif

#define TFTWIDTH			240
#define TFTHEIGHT			240

#define TFTLCD_DELAY		0x00

// GPIO to control bus pin connections
#define LCD_CS_GPIO_PORT	GPIOB
#define LCD_CS_PIN			GPIO_PIN_14
#define LCD_CS_IDLE()		LCD_CS_GPIO_PORT->BSRR = LCD_CS_PIN						// CS_HIGH
#define LCD_CS_ACTIVE()		LCD_CS_GPIO_PORT->BSRR = (uint32_t)LCD_CS_PIN << 16U	// CS_LOW

#define LCD_CD_GPIO_PORT	GPIOH
#define LCD_CD_PIN			GPIO_PIN_6
#define LCD_CD_DATA()		LCD_CD_GPIO_PORT->BSRR = LCD_CD_PIN						// CD_HIGH
#define LCD_CD_COMMAND()	LCD_CD_GPIO_PORT->BSRR = (uint32_t)LCD_CD_PIN << 16U	// CD_LOW

#define LCD_RST_GPIO_PORT	GPIOA
#define LCD_RST_PIN			GPIO_PIN_11
#define LCD_RST_IDLE()		LCD_RST_GPIO_PORT->BSRR = LCD_RST_PIN					// RST_HIGH
#define LCD_RST_ACTIVE()	LCD_RST_GPIO_PORT->BSRR = (uint32_t)LCD_RST_PIN << 16U	// RST_LOW

#define LCD_SPI_TIMEOUT		10

#define swap(a, b)			do {\
								int16_t t = a;\
								a = b;\
								b = t;\
							} while(0)

#define byteswap16(a)		((a) >> 8) | ((a) << 8)

// Color definitions
#define	BLACK				0x0000
#define	BLUE				0x001F
#define	RED					0xF800
#define	GREEN				0x07E0
#define CYAN				0x07FF
#define MAGENTA				0xF81F
#define YELLOW				0xFFE0
#define WHITE				0xFFFF
#define LIGHTGRAY			0xCDB6

void LCD_Init(SPI_HandleTypeDef * hspi);
void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void LCD_Flood(uint16_t color, uint32_t len);
void LCD_FillScreen(uint16_t color);
void LCD_Reset(void);
void LCD_SetAddrWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_DrawBMP(uint16_t x, uint16_t y, const uint8_t *pArr);
#if defined (USE_FATFS)
void LCD_DrawBMPFromFile(uint16_t x, uint16_t y, FIL * pFile);
#endif

void LCD_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
void LCD_DrawFastHLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color);
void LCD_DrawFastVLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color);
void LCD_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void LCD_DrawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
void LCD_DrawCircleHelper( uint16_t x0, uint16_t y0, uint16_t r, uint8_t cornername, uint16_t color);
void LCD_FillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
void LCD_FillCircleHelper(uint16_t x0, uint16_t y0, uint16_t r, uint8_t cornername, uint16_t delta, uint16_t color);
void LCD_DrawTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void LCD_FillTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void LCD_DrawRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r, uint16_t color);
void LCD_FillRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r, uint16_t color);
void LCD_SetRGB();

void LCD_DrawChar(uint16_t x, uint16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t fontindex);
void LCD_Printf(const char *fmt, ...);

void LCD_SetCursor(uint16_t x, uint16_t y);
void LCD_SetTextSize(uint8_t s);
void LCD_SetTextColor(uint16_t c, uint16_t b);
void LCD_SetTextWrap(uint8_t w);
void LCD_SetRotation(uint8_t x);
uint8_t LCD_GetRotation(void);
int16_t LCD_GetCursorX(void);
int16_t LCD_GetCursorY(void);
uint16_t LCD_Color565(uint8_t r, uint8_t g, uint8_t b);
#endif /* __LCD_H */