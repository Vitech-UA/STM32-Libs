/*
 * ST7789.cpp
 *
 *  Created on: Mar 21, 2021
 *      Author: Embed Viktor
 */

#include "ST7789.h"

static font_t* fonts[] = {
#ifdef USE_FONT8
		&Font8,
#endif
#ifdef USE_FONT12
		&Font12,
#endif
#ifdef USE_FONT16
		&Font16,
#endif
#ifdef USE_FONT20
		&Font20,
#endif
#ifdef USE_FONT24
		&Font24,
#endif
	};

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

	this->ST7789_Width = WIDTH;
	this->ST7789_Height = HEIGHT;
	this->HardReset();
	this->SoftReset();
	this->SleepModeExit();
	this->ColorModeSet(ST7789_ColorMode_65K | ST7789_ColorMode_16bit);
	delayMs(10);
	this->MemAccessModeSet(4, 1, 1, 0);
	delayMs(10);
	this->InversionMode(1);
	delayMs(10);
	this->FillScreen(BLACK);
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
void ST7789::DrawPixel(int16_t x, int16_t y, uint16_t color) {
	if ((x < 0) || (x >= ST7789_Width) || (y < 0) || (y >= ST7789_Height))
		return;

	this->SetWindow(x, y, x, y);
	this->RamWrite(&color, 1);
}

void ST7789::DrawChar_5x8(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, unsigned char c)
{
  if((x >= 240) || (y >= 240) || ((x + 4) < 0) || ((y + 7) < 0)) return;
  if(c<128)            c = c-32;
  if(c>=144 && c<=175) c = c-48;
  if(c>=128 && c<=143) c = c+16;
  if(c>=176 && c<=191) c = c-48;
  if(c>191)  return;
  for (uint8_t i=0; i<6; i++ )
	{
	uint8_t line;
    if (i == 5) line = 0x00;
    else line = font[(c*5)+i];
		for (uint8_t j = 0; j<8; j++)
		{
			if (line & 0x01) this->DrawPixel(x + i, y + j, TextColor);
			else if (!TransparentBg) this->DrawPixel(x + i, y + j, BgColor);
			line >>= 1;
		}
	}
}

void ST7789::DrawChar_7x11(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, unsigned char c)
{
	uint8_t i,j;
  uint8_t buffer[11];

  if((x >= 240) || (y >= 240) || ((x + 4) < 0) || ((y + 7) < 0)) return;

	// Copy selected simbol to buffer
	memcpy(buffer,&font7x11[(c-32)*11],11);
	for(j=0;j<11;j++)
	{
		for(i=0;i<7;i++)
		{
			if ((buffer[j] & (1<<i)) == 0)
			{
				if (!TransparentBg) this->DrawPixel(x + i, y + j, BgColor);
			}
			else this->DrawPixel(x + i, y + j, TextColor);
		}
	}
}

void ST7789::Print_5x8(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, char *str)
{
  unsigned char type = *str;
  if (type>=128) x = x - 3;
  while (*str)
	{
	  this->DrawChar_5x8(x, y, TextColor, BgColor, TransparentBg, *str++);
    unsigned char type = *str;
    if (type>=128) x=x+3;
    else x=x+6;
  }
}

void ST7789::Print_7x11(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, char *str)
{
  unsigned char type = *str;
  if (type>=128) x = x - 3;
  while (*str)
	{
	  this->DrawChar_7x11(x, y, TextColor, BgColor, TransparentBg, *str++);
    unsigned char type = *str;
    if (type>=128) x=x+8;
    else x=x+8;
  }
}

inline uint16_t ST7789::Color565(uint8_t r, uint8_t g, uint8_t b) {
	return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

void ST7789::DrawChar(uint16_t x, uint16_t y, unsigned char c, uint16_t color,
		uint16_t bg, uint8_t fontindex) {
	uint16_t height, width, bytes;
	uint8_t offset;
	uint32_t charindex = 0;
	uint8_t *pchar;
	uint32_t line = 0;

	height = fonts[fontindex]->Height;
	width = fonts[fontindex]->Width;

	if ((x >= this->ST7789_Width) || // Clip right
			(y >= this->ST7789_Height) || // Clip bottom
			((x + width - 1) < 0) || // Clip left
			((y + height - 1) < 0))   // Clip top
		return;

	bytes = (width + 7) / 8;
	if (c < ' ')
		c = ' ';
#ifndef USE_CP1251
	else if (c > '~')
		c = ' ';
#endif
	charindex = (c - ' ') * height * bytes;
	offset = 8 * bytes - width;

	for (uint32_t i = 0; i < height; i++) {
		pchar = ((uint8_t *) &fonts[fontindex]->table[charindex]
				+ (width + 7) / 8 * i);
		switch (bytes) {
		case 1:
			line = pchar[0];
			break;
		case 2:
			line = (pchar[0] << 8) | pchar[1];
			break;
		case 3:
		default:
			line = (pchar[0] << 16) | (pchar[1] << 8) | pchar[2];
			break;
		}
		for (uint32_t j = 0; j < width; j++) {
			if (line & (1 << (width - j + offset - 1))) {
				this->DrawPixel((x + j), y, color);
			} else {
				this->DrawPixel((x + j), y, bg);
			}
		}
		y++;
	}
}

void ST7789::Printf(uint16_t m_cursor_x, uint16_t m_cursor_y,uint16_t m_textcolor,
		uint16_t m_textbgcolor, uint8_t fontIndex, const char *fmt, ...) {
	static char buf[256];
	char *p;
	va_list lst;

	va_start(lst, fmt);
	vsprintf(buf, fmt, lst);
	va_end(lst);

	volatile uint16_t height, width;

	height = fonts[fontIndex]->Height;
	width = fonts[fontIndex]->Width;

	p = buf;
	while (*p) {
		if (*p == '\n') {
			m_cursor_y += height;
			m_cursor_x = 0;
		} else if (*p == '\r') {
			m_cursor_x = 0;
		} else if (*p == '\t') {
			m_cursor_x += width * 4;
		} else {
			if (m_cursor_x == 0) {
				this->SetWindow(0,m_cursor_y, this->ST7789_Width - 1,
						m_cursor_y + height);
				this->Flood(m_textbgcolor, (long) this->ST7789_Width * height);
				this->SetWindow(0, 0, this->ST7789_Width - 1, this->ST7789_Height - 1);
			}
			if (m_cursor_y >= (this->ST7789_Height - height)) {
				m_cursor_y = 0;
				this->FillScreen(this->m_textbgcolor);
			}
			this->DrawChar(m_cursor_x, m_cursor_y, *p, m_textcolor, m_textbgcolor,
					fontIndex);
			m_cursor_x += width;
			if (m_wrap && (m_cursor_x > (this->ST7789_Width - width))) {
				m_cursor_y += height;
				m_cursor_x = 0;
			}
		}
		p++;
	}
}


void ST7789::Flood(uint16_t color, uint32_t len) {
	//LCD_CS_LOW

	len--;
	while (len--) {
		this->WriteMultiple((uint8_t *) &color, 2);
	}
	//LCD_CS_HIGH
}
void ST7789::WriteMultiple(uint8_t * pData, uint32_t size) {
	if (size == 1) {
		/* Only 1 byte to be sent to LCD - general interface can be used */
		/* Send Data */
		this->Write(*pData);
	} else {
		/* Several data should be sent in a raw */
		/* Direct SPI accesses for optimization */
		for (uint32_t counter = size; counter != 0; counter -= 2) {
			while (((this->SPI_ITEM->SR) & SPI_SR_TXE) != SPI_SR_TXE) {
			}
			/* Need to invert bytes for LCD*/
			*((__IO uint8_t*) &(this->SPI_ITEM->DR)) = *(pData + 1);
			while (((this->SPI_ITEM->SR) & SPI_SR_TXE) != SPI_SR_TXE) {
			}
			*((__IO uint8_t*) &(this->SPI_ITEM->DR)) = *pData;
			pData += 2;
		}

		/* Wait until the bus is ready before releasing Chip select */
		while (((this->SPI_ITEM->SR) & SPI_SR_BSY) != RESET) {
		}
	}
}
