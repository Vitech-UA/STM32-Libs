/*
 * ST7789.h
 *
 *  Created on: Mar 21, 2021
 *      Author: Embed Viktor
 */

#ifndef ST7789_H_
#define ST7789_H_

#include "SPI.h"
#include "delay.h"

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

#define Width 240
#define Height 240

#define ST7789_X_Start          0
#define ST7789_Y_Start          0

class ST7789: public SPI {
public:
	ST7789(SPI_TypeDef *Port, SPI_DataSize_t size, GPIO_TypeDef *BlinkPort,
			uint16_t BlinkPin, GPIO_TypeDef *ResetPort, uint16_t ResetPin,
			GPIO_TypeDef *DcPort, uint16_t DcPin);
	void BrightnessEnable(void);
	void BrightnessDisable(void);
	void FillScreen(uint16_t color);
private:
	void ResetLow(void);
	void ResetHigh(void);
	void DC_Data(void);
	void DC_Command(void);
	void Write(uint8_t Data);
	void WriteCMD(uint8_t CMD);
	void HardReset(void);
	void SoftReset(void);
	void SendData(uint8_t data);
	void SleepModeExit(void);
	void ColorModeSet(uint8_t ColorMode);
	void MemAccessModeSet(uint8_t Rotation, uint8_t VertMirror,
			uint8_t HorizMirror, uint8_t IsBGR);
	void InversionMode(uint8_t Mode);

	void FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
	void SetWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
	void RamWrite(uint16_t *pBuff, uint16_t Len);
	void ColumnSet(uint16_t ColumnStart, uint16_t ColumnEnd);
	void RowSet(uint16_t RowStart, uint16_t RowEnd);
	void SetBL(uint8_t Value);
	void DisplayPower(uint8_t On);
	GPIO_TypeDef *ItemBlinkPort;
	uint16_t ItemBlinkPin;
	GPIO_TypeDef *ItemResetPort;
	uint16_t ItemResetPin;
	GPIO_TypeDef *ItemDcPort;
	uint16_t ItemDcPin;
	Gpio Blink;
	Gpio Reset;
	Gpio Dc;
	uint8_t ST7789_Width;
	uint8_t ST7789_Height;
};

#endif /* ST7789_H_ */
