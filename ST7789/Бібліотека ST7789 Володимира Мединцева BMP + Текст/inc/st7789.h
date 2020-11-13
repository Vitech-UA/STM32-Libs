#ifndef _ST7789_H
#define _ST7789_H

#define RGB565(r, g, b)         (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))

#include "stm32f1xx_hal.h"

#define DC_Pin GPIO_PIN_9
#define DC_GPIO_Port GPIOB

#define RES_Pin GPIO_PIN_1
#define RES_GPIO_Port GPIOB

#define ST7789_ColorMode_65K    0x50
#define ST7789_ColorMode_262K   0x60
#define ST7789_ColorMode_12bit  0x03
#define ST7789_ColorMode_16bit  0x05
#define ST7789_ColorMode_18bit  0x06
#define ST7789_ColorMode_16M    0x07

#define ST7789_Cmd_SWRESET      0x01
#define ST7789_Cmd_SLPIN        0x10
#define ST7789_Cmd_SLPOUT       0x11
#define ST7789_Cmd_COLMOD       0x3A
#define ST7789_Cmd_PTLON        0x12
#define ST7789_Cmd_NORON        0x13
#define ST7789_Cmd_INVOFF       0x20
#define ST7789_Cmd_INVON        0x21
#define ST7789_Cmd_GAMSET       0x26
#define ST7789_Cmd_DISPOFF      0x28
#define ST7789_Cmd_DISPON       0x29
#define ST7789_Cmd_CASET        0x2A
#define ST7789_Cmd_RASET        0x2B
#define ST7789_Cmd_RAMWR        0x2C
#define ST7789_Cmd_PTLAR        0x30
#define ST7789_Cmd_MADCTL       0x36
#define ST7789_Cmd_MADCTL_MY    0x80
#define ST7789_Cmd_MADCTL_MX    0x40
#define ST7789_Cmd_MADCTL_MV    0x20
#define ST7789_Cmd_MADCTL_ML    0x10
#define ST7789_Cmd_MADCTL_RGB   0x00
#define ST7789_Cmd_RDID1        0xDA
#define ST7789_Cmd_RDID2        0xDB
#define ST7789_Cmd_RDID3        0xDC
#define ST7789_Cmd_RDID4        0xDD
#define ST7789_MADCTL_MY        0x80
#define ST7789_MADCTL_MX        0x40
#define ST7789_MADCTL_MV        0x20
#define ST7789_MADCTL_ML        0x10
#define ST7789_MADCTL_BGR       0x08
#define ST7789_MADCTL_MH        0x04

#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0 
#define WHITE    0xFFFF

// �������� ������� ������������ �����/�������� �����������
#define ST7789_X_Start          0
#define ST7789_Y_Start          0

void ST7789_Init(uint8_t Width, uint8_t Height);
void ST7789_HardReset(void);
void ST7789_SoftReset(void);
void ST7789_SendCmd(uint8_t Cmd);
void ST7789_SendData(uint8_t Data);
void ST7789_SleepModeEnter(void);
void ST7789_SleepModeExit(void);
void ST7789_ColorModeSet(uint8_t ColorMode);
void ST7789_MemAccessModeSet(uint8_t Rotation, uint8_t VertMirror,
		uint8_t HorizMirror, uint8_t IsBGR);
void ST7789_InversionMode(uint8_t Mode);
void ST7789_FillScreen(uint16_t color);
void ST7789_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void ST7789_SetWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void ST7789_RamWrite(uint16_t *pBuff, uint16_t Len);
void ST7789_ColumnSet(uint16_t ColumnStart, uint16_t ColumnEnd);
void ST7789_RowSet(uint16_t RowStart, uint16_t RowEnd);
void ST7789_SetBL(uint8_t Value);
void ST7789_DisplayPower(uint8_t On);
void ST7789_DrawRectangle(int16_t x1, int16_t y1, int16_t x2, int16_t y2,
		uint16_t color);
void ST7789_DrawRectangleFilled(int16_t x1, int16_t y1, int16_t x2, int16_t y2,
		uint16_t fillcolor);
void ST7789_DrawLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2,
		uint16_t color);
void SwapInt16Values(int16_t *pValue1, int16_t *pValue2);
void ST7789_DrawLine_Slow(int16_t x1, int16_t y1, int16_t x2, int16_t y2,
		uint16_t color);
void ST7789_DrawPixel(int16_t x, int16_t y, uint16_t color);
void ST7789_DrawCircleFilled(int16_t x0, int16_t y0, int16_t radius,
		uint16_t fillcolor);
void ST7789_DrawCircle(int16_t x0, int16_t y0, int16_t radius, uint16_t color);
void ST7789_DrawBMP(uint16_t x, uint16_t y, const uint8_t *pArr);
void LCD_WriteMultiple(uint8_t *pData, uint32_t size);
uint16_t LCD_Color565(uint8_t r, uint8_t g, uint8_t b);

/* Прототипи функцій рпоботи з текстом */

void ST7789_DrawChar_5x8(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, unsigned char c);
void ST7789_DrawChar_7x11(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, unsigned char c);
void ST7789_print_5x8(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, char *str);
void ST7789_print_7x11(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, char *str);
#endif
