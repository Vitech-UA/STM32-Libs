/*
 * ST7789.cpp
 *
 *  Created on: Mar 21, 2021
 *      Author: Embed Viktor
 */

#include "ST7789.h"

/*
 Розпіновка тестової плати
 MOSI - PA7
 SCK - PA5
 RES - PC5
 DC - PC4
 BLK - PA4
 */

ST7789::ST7789(SPI_TypeDef *Port, SPI_DataSize_t size, GPIO_TypeDef *BlinkPort,
		uint16_t BlinkPin, GPIO_TypeDef *ResetPort, uint16_t ResetPin,
		GPIO_TypeDef *DcPort, uint16_t DcPin) :
		SPI(Port, size) {

	this->ItemBlinkPort = BlinkPort;
	this->ItemBlinkPin = BlinkPin;
	this->Blink = Gpio(this->ItemBlinkPort, this->ItemBlinkPin);
	this->Blink.SetAsGenerapPurporseOutput(OUTPUT_PP);

	this->ItemResetPort = ResetPort;
	this->ItemResetPin = ResetPin;
	this->Reset = Gpio(this->ItemResetPort, this->ItemResetPin);
	this->Reset.SetAsGenerapPurporseOutput(OUTPUT_PP);

	this->ItemDcPort = DcPort;
	this->ItemDcPin = DcPin;
	this->Dc = Gpio(this->ItemDcPort, this->ItemDcPin);
	this->Dc.SetAsGenerapPurporseOutput(OUTPUT_PP);

	this->ST7789_Width = Width;
	this->ST7789_Height = Height;
	this->HardReset();
	this->SoftReset();
	this->SleepModeExit();
	this->ColorModeSet(ST7789_ColorMode_65K | ST7789_ColorMode_16bit);
	delayMs(10);
	this->MemAccessModeSet(4, 1, 1, 0);
	delayMs(10);
	this->InversionMode(1);
	delayMs(10);
	this->FillScreen(GREEN);
    this->SetBL(10);
    this->DisplayPower(1);
    delayMs(10);

}

void ST7789::BrightnessEnable(void) {
	this->Blink.Set();
}
void ST7789::BrightnessDisable(void) {
	this->Blink.Reset();

}

void ST7789::Write(uint8_t Data) {

	this->TransmitBlocking(Data);
}

void ST7789::HardReset(void) {
	this->Reset.Reset(); // ResetPin low
	delayMs(10);
	this->Reset.Set();
	delayMs(150);

}
void ST7789::SoftReset(void) {
	this->WriteCMD(ST7789_Cmd_SWRESET);
	delayMs(130);
}

void ST7789::SendData(uint8_t data) {
	this->Dc.Set();
	*((__IO uint8_t*) &SPI1->DR) = data;
}

void ST7789::SleepModeExit(void) {
	this->WriteCMD(ST7789_Cmd_SLPOUT);
	delayMs(500);
}

void ST7789::ColorModeSet(uint8_t ColorMode) {
	this->WriteCMD(ST7789_Cmd_COLMOD);
	this->SendData(ColorMode & 0x77);
}

void ST7789::WriteCMD(uint8_t CMD) {
	this->DC_Command();
	this->Write(CMD);
}

void ST7789::MemAccessModeSet(uint8_t Rotation, uint8_t VertMirror,
		uint8_t HorizMirror, uint8_t IsBGR) {
	uint8_t Value;
	Rotation &= 7;

	this->WriteCMD(ST7789_Cmd_MADCTL);

	switch (Rotation) {
	case 0:
		Value = 0;
		break;
	case 1:
		Value = ST7789_MADCTL_MX;
		break;
	case 2:
		Value = ST7789_MADCTL_MY;
		break;
	case 3:
		Value = ST7789_MADCTL_MX | ST7789_MADCTL_MY;
		break;
	case 4:
		Value = ST7789_MADCTL_MV;
		break;
	case 5:
		Value = ST7789_MADCTL_MV | ST7789_MADCTL_MX;
		break;
	case 6:
		Value = ST7789_MADCTL_MV | ST7789_MADCTL_MY;
		break;
	case 7:
		Value = ST7789_MADCTL_MV | ST7789_MADCTL_MX | ST7789_MADCTL_MY;
		break;
	}

	if (VertMirror)
		Value = ST7789_MADCTL_ML;
	if (HorizMirror)
		Value = ST7789_MADCTL_MH;

	if (IsBGR)
		Value |= ST7789_MADCTL_BGR;

	this->SendData(Value);
}

void ST7789::InversionMode(uint8_t Mode) {
	if (Mode)
		this->WriteCMD(ST7789_Cmd_INVON);
	else
		this->WriteCMD(ST7789_Cmd_INVOFF);
}

void ST7789::ResetLow(void) {
	this->Reset.Reset();
}
void ST7789::ResetHigh(void) {
	this->Reset.Set();
}
void ST7789::DC_Data(void) {
	this->Dc.Set();
}
void ST7789::DC_Command(void) {
	this->Dc.Reset();
}

void ST7789::FillScreen(uint16_t color) {
	this->FillRect(0, 0, ST7789_Width, ST7789_Height, color);
}
void ST7789::FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
	if ((x >= ST7789_Width) || (y >= ST7789_Height))
		return;
	if ((x + w) > ST7789_Width)
		w = ST7789_Width - x;
	if ((y + h) > ST7789_Height)
		h = ST7789_Height - y;
	this->SetWindow(x, y, x + w - 1, y + h - 1);
	for (uint32_t i = 0; i < (h * w); i++)
		this->RamWrite(&color, 1);
}
void ST7789::SetWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
	this->ColumnSet(x0, x1);
	this->RowSet(y0, y1);
	this->WriteCMD(ST7789_Cmd_RAMWR);
}

void ST7789::RamWrite(uint16_t *pBuff, uint16_t Len) {
	while (Len--) {
		this->SendData(*pBuff >> 8);
		this->SendData(*pBuff & 0xFF);
	}
}
void ST7789::ColumnSet(uint16_t ColumnStart, uint16_t ColumnEnd) {
	if (ColumnStart > ColumnEnd)
		return;
	if (ColumnEnd > ST7789_Width)
		return;

	ColumnStart += ST7789_X_Start;
	ColumnEnd += ST7789_X_Start;

	this->WriteCMD(ST7789_Cmd_CASET);
	this->SendData(ColumnStart >> 8);
	this->SendData(ColumnStart & 0xFF);
	this->SendData(ColumnEnd >> 8);
	this->SendData(ColumnEnd & 0xFF);
}

void ST7789::RowSet(uint16_t RowStart, uint16_t RowEnd) {
	if (RowStart > RowEnd)
		return;
	if (RowEnd > ST7789_Height)
		return;

	RowStart += ST7789_Y_Start;
	RowEnd += ST7789_Y_Start;

	this->WriteCMD(ST7789_Cmd_RASET);
	this->SendData(RowStart >> 8);
	this->SendData(RowStart & 0xFF);
	this->SendData(RowEnd >> 8);
	this->SendData(RowEnd & 0xFF);
}

void ST7789::SetBL(uint8_t Value) {
	if (Value > 100)
		Value = 100;
}

void ST7789::DisplayPower(uint8_t On) {
	if (On)
		this->WriteCMD(ST7789_Cmd_DISPON);
	else
		this->WriteCMD(ST7789_Cmd_DISPOFF);
}
