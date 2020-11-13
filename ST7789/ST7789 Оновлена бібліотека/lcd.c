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

#include "lcd.h"

// Hold pointer to inited HAL SPI device
static SPI_HandleTypeDef * LCD_hspi;

static int16_t m_width;
static int16_t m_height;
static int16_t m_cursor_x;
static int16_t m_cursor_y;

static uint16_t m_textcolor;
static uint16_t m_textbgcolor;
static uint8_t m_font;
static uint8_t m_rotation;
static uint8_t m_wrap;

static font_t * fonts[] = {
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
const static uint8_t fontsNum = sizeof(fonts) / sizeof(fonts[0]);

inline static void LCD_Write(uint8_t data);
inline static void LCD_WriteMultiple(uint8_t * pData, uint32_t size);
inline static void LCD_WriteCmd(uint8_t addr);
inline static void LCD_WriteData(uint8_t * pData, uint32_t size);
inline static void LCD_WriteRegister(uint8_t addr, uint8_t * pData,
		uint32_t size);

// Initialization command tables for different LCD controllers
#if	defined(ST7735)
static const uint16_t ST7735_regValues[] = {

	ST7735_SLPOUT, 0, /* Out of sleep mode, 0 args, no delay */
	ST7735_FRMCTR1, 3, 0x01, 0x2C, 0x2d, //2d, /* Frame rate ctrl - normal mode, 3 args:Rate = fosc/(1x2+40) * (LINE+2C+2D)*/
	ST7735_FRMCTR2, 3, 0x01, 0x2C, 0x2d, /* Frame rate control - idle mode, 3 args:Rate = fosc/(1x2+40) * (LINE+2C+2D) */
	ST7735_FRMCTR3, 6, 0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D, /* Frame rate ctrl - partial mode, 6 args: Dot inversion mode, Line inversion mode */
	ST7735_INVCTR, 1, 0x07, /* Display inversion ctrl, 1 arg, no delay: No inversion */
	ST7735_PWCTR1, 3, 0xA2, 0x02, 0x84, /* Power control, 3 args, no delay: -4.6V , AUTO mode */
	ST7735_PWCTR2, 1, 0xC5, /* Power control, 1 arg, no delay: VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD */
	ST7735_PWCTR3, 2, 0x0A, 0x00, /* Power control, 2 args, no delay: Opamp current small, Boost frequency */
	ST7735_PWCTR4, 2, 0x8A, 0x2A, /* Power control, 2 args, no delay: BCLK/2, Opamp current small & Medium low */
	ST7735_PWCTR5, 2, 0x8A, 0xEE, /* Power control, 2 args, no delay */
	ST7735_VMCTR1, 1, 0x3C,//0E /* Power control, 1 arg, no delay */
	ST7735_INVON, 0, /* Don't invert display, no args, no delay */
	ST7735_COLMOD, 1, 0x05, /* Set color mode, 1 arg, no delay: 16-bit color */
	ST7735_CASET, 4, 0x00, 0x00, 0x00, 0xEF, /* Column addr set, 4 args, no delay: XSTART = 0, XEND = 239 */
	ST7735_RASET, 4, 0x00, 0x00, 0x00, 0xEF, /* Row addr set, 4 args, no delay: YSTART = 0, YEND = 239 */
	//ST7735_GAMCTRP1, 16, 0x02, 0x1C, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d, 0x29, 0x25,0x2B, 0x39, 0x00, 0x01, 0x03, 0x10, /* Magical unicorn dust, 16 args, no delay */
	//ST7735_GAMCTRN2, 16, 0x00, 0x1D, 0x07, 0x06, 0x2E, 0xFF, 0xFF, 0xFF,0xFF, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10, /* Sparkles and rainbows, 16 args, no delay */
	ST7735_DISPON, 0, /* Main screen turn on, no delay */
	ST7735_MADCTL, 1, ST7735_MADCTL_MX | ST7735_MADCTL_MY /* Memory access control: MY = 1, MX = 1, MV = 0, ML = 0 */
};
#elif
#endif

/**
 * \brief Send one byte over SPI
 *
 * \param data	8-Bit Data
 *
 * \return void
 */
inline static void LCD_Write(uint8_t data) {
	/* Check the communication status */
	if(HAL_SPI_Transmit(LCD_hspi, &data, 1, LCD_SPI_TIMEOUT) != HAL_OK) {
		/* Execute user timeout callback */
		Error_Handler();
	}
}

/**
 * \brief Sends multiple bytes over SPI
 *
 * \param pData		pointer on data array
 * \param size		amount of bytes to transmit
 *
 * \return void
 */

inline static void LCD_WriteMultiple(uint8_t * pData, uint32_t size) {
	if (size == 1) {
		/* Only 1 byte to be sent to LCD - general interface can be used */
		/* Send Data */
		LCD_Write(*pData);
	} else {
		/* Several data should be sent in a raw */
		/* Direct SPI accesses for optimization */
		for (uint32_t counter = size; counter != 0; counter -= 2) {
			while (((LCD_hspi->Instance->SR) & SPI_FLAG_TXE) != SPI_FLAG_TXE) {
			}
			/* Need to invert bytes for LCD*/
			*((__IO uint8_t*) &(LCD_hspi->Instance->DR)) = *(pData + 1);
			while (((LCD_hspi->Instance->SR) & SPI_FLAG_TXE) != SPI_FLAG_TXE) {
			}
			*((__IO uint8_t*) &(LCD_hspi->Instance->DR)) = *pData;
			pData += 2;
		}

		/* Wait until the bus is ready before releasing Chip select */
		while (((LCD_hspi->Instance->SR) & SPI_FLAG_BSY) != RESET) {
		}
	}
}

/**
 * \brief Selects register
 *
 * \param addr		Register address (8-bit address)
 *
 * \return void
 */
inline static void LCD_WriteCmd(uint8_t addr) {
	LCD_CD_COMMAND();
	LCD_Write(addr);
}

/**
 * \brief Writes multiple bytes of 8-Bit data
 *
 * \param pData		pointer on data array
 * \param size		amount of bytes to trasmit
 *
 * \return void
 */

inline static void LCD_WriteData(uint8_t * pData, uint32_t size) {
	LCD_CD_DATA();
	LCD_WriteMultiple(pData, size);
}

/**
 * \brief Writes n bytes of data to register (8-bit address)
 *
 * \param addr		Register
 * \param pData		Pointer to data
 * \param size		Size
 *
 * \return void
 */
inline static void LCD_WriteRegister(uint8_t addr, uint8_t * pData,
		uint32_t size) {
	LCD_WriteCmd(addr);
	LCD_WriteData(pData, size);
}

/**
 * \brief LCD Initialization
 *
 * \param
 *
 * \return void
 */
void LCD_Init(SPI_HandleTypeDef * hspi) {

	LCD_hspi = hspi;
	LCD_FillScreen(BLACK);
	m_width = TFTWIDTH;
	m_height = TFTHEIGHT;
	m_rotation = 0;
	m_cursor_y = m_cursor_x = 0;
	m_font = 0;
	m_textcolor = m_textbgcolor = 0xFFFF;
	m_wrap = 1;

	LCD_Reset();
	HAL_Delay(50);

	LCD_CS_ACTIVE();

#if	defined(ST7735)
	uint8_t i = 0;
	uint16_t r, len;
	while (i < sizeof(ST7735_regValues) / sizeof(ST7735_regValues[0])) {
		r = ST7735_regValues[i++];
		len = ST7735_regValues[i++];
		if (r == TFTLCD_DELAY) {
			HAL_Delay(len);
		} else {
			LCD_WriteCmd(r);
			LCD_CD_DATA();
			for (uint8_t d = 0; d < len; d++) {
				LCD_Write(ST7735_regValues[i++]);
			}
		}
	}

	LCD_SetRotation(m_rotation);
	LCD_SetAddrWindow(0, 0, m_width - 1, m_height - 1);
#elif
#endif

	LCD_FillScreen(BLACK);
	LCD_SetTextSize(0);
	LCD_SetTextColor(WHITE, BLACK);
}

/**
 * \brief Draws a point at the specified coordinates
 *
 * \param x		x-Coordinate
 * \param y		y-Coordinate
 * \param color	Color
 *
 * \return void
 */
void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
	// Clip
	if ((x < 0) || (y < 0) || (x >= m_width) || (y >= m_height))
		return;

	LCD_CS_ACTIVE();

#if	defined(ST7735)
	LCD_WriteRegister(ST7735_CASET, (uint8_t *) &x, 2);
	LCD_WriteRegister(ST7735_RASET, (uint8_t *) &y, 2);
	LCD_WriteRegister(ST7735_RAMWR, (uint8_t *) &color, 2);
#elif
#endif

	LCD_CS_IDLE();
}

/**
 * \brief Flood
 *
 * \param color	Color
 * \param len	Length
 *
 * \return void
 */
void LCD_Flood(uint16_t color, uint32_t len) {
	LCD_CS_ACTIVE();
#if	defined(ST7735)
	LCD_WriteRegister(ST7735_RAMWR, (uint8_t *) &color, 2);
#elif
#endif
	len--;
	while (len--) {
		LCD_WriteMultiple((uint8_t *) &color, 2);
	}
	LCD_CS_IDLE();
}

/**
 * \brief Fills the screen with the specified color
 *
 * \param color	Color
 *
 * \return void
 */
void LCD_FillScreen(uint16_t color) {
#if defined(ST7735)
	LCD_SetAddrWindow(0, 0, m_width - 1, m_height - 1);
#elif
#endif
	LCD_Flood(color, (long) TFTWIDTH * (long) TFTHEIGHT);
}

/**
 * \brief Resets the Display
 *
 * \param
 *
 * \return void
 */
void LCD_Reset(void) {
	LCD_CS_IDLE();
	LCD_CD_DATA();
	LCD_RST_ACTIVE();
	HAL_Delay(10);
	LCD_RST_IDLE();
	HAL_Delay(120);
	LCD_CS_ACTIVE();
	LCD_WriteCmd(ST7735_SWRESET);
	HAL_Delay(120);
	LCD_CS_IDLE();
}

/**
 * \brief Sets window address
 *
 * \param x1
 * \param y1
 * \param x2
 * \param y2
 *
 * \return void
 */
void LCD_SetAddrWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
	LCD_CS_ACTIVE();
#if	defined(ST7735)
	LCD_WriteCmd(ST7735_CASET);
	LCD_CD_DATA();
	LCD_WriteMultiple((uint8_t *) &x1, 2);
	LCD_WriteMultiple((uint8_t *) &x2, 2);
	LCD_WriteCmd(ST7735_RASET);
	LCD_CD_DATA();
	LCD_WriteMultiple((uint8_t *) &y1, 2);
	LCD_WriteMultiple((uint8_t *) &y2, 2);
#elif
#endif
	LCD_CS_IDLE();
}

/**
 * \brief Set display rotation
 *
 * \param x	rotation
 *
 * \return void
 */
void LCD_SetRotation(uint8_t x) {
	m_rotation = (x % 4);
	switch (m_rotation) {
	case 0:
	case 2:
		m_width = TFTWIDTH;
		m_height = TFTHEIGHT;
		break;
	case 1:
	case 3:
		m_width = TFTHEIGHT;
		m_height = TFTWIDTH;
		break;
	}
	LCD_CS_ACTIVE();
#if	defined(ST7735)
	uint8_t t;
	switch (m_rotation) {
	default:
		t = ST7735_MADCTL_MX | ST7735_MADCTL_MY;
		break;
	case 1:
		t = ST7735_MADCTL_MY | ST7735_MADCTL_MV;
		break;
	case 2:
		t = 0;
		break;
	case 3:
		t = ST7735_MADCTL_MX | ST7735_MADCTL_MV;
		break;
	}
	LCD_WriteRegister(ST7735_MADCTL, &t, 1); // MADCTL
	LCD_SetAddrWindow(0, 0, m_width - 1, m_height - 1);
#elif
#endif
	LCD_CS_IDLE();
}

/**
 * \brief  Draws a BMP picture loaded in the STM32 MCU internal memory.
 *
 * \param xPos		Bmp X position in the LCD
 * \param yPos		Bmp Y position in the LCD
 * \param pArr		Pointer to Bmp picture address
 * \return void
 */
void LCD_DrawBMP(uint16_t x, uint16_t y, const uint8_t *pArr)
{
	uint8_t* pBmp;
	uint8_t *start;
	uint8_t *end;
	uint32_t offset = 0, size = 0;
	int32_t height = 0, width = 0;
	uint16_t colordepth = 0;
	uint16_t color = 0;

	pBmp = (uint8_t *) pArr;
	/* Read bitmap size */
	size = *(volatile uint16_t *) (pBmp + 2);
	size |= (*(volatile uint16_t *) (pBmp + 4)) << 16;
	/* Get bitmap data address offset */
	offset = *(volatile uint16_t *) (pBmp + 10);
	offset |= (*(volatile uint16_t *) (pBmp + 12)) << 16;
	/* Read bitmap width */
	width = *(uint16_t *) (pBmp + 18);
	width |= (*(uint16_t *) (pBmp + 20)) << 16;
	/* Read bitmap height */
	height = *(uint16_t *) (pBmp + 22);
	height |= (*(uint16_t *) (pBmp + 24)) << 16;
	/* Read color depth */
	colordepth = *(uint16_t *) (pBmp + 28);

	start = (uint8_t *) pBmp + offset;
	end = (uint8_t *) pBmp + size;

	LCD_SetAddrWindow(x, y, x + width - 1, y + abs(height) - 1);
	LCD_CS_ACTIVE();
#if	defined(ST7735)
	LCD_WriteCmd(ST7735_RAMWR); // Write data to GRAM
#elif
#endif
	LCD_CD_DATA();
	if (colordepth == 16) {
		if (height < 0) {
			pBmp = start;
			while (pBmp < end) {
				LCD_WriteMultiple(pBmp, 2);
				pBmp += 2;
			}
		} else {
			pBmp = end - 1;
			while (pBmp >= start) {
				LCD_WriteMultiple(pBmp - 1, 2);
				pBmp -= 2;
			}
		}
	} else if (colordepth == 24) {
		if (height < 0) {
			pBmp = start;
			while (pBmp < end) {
				color = LCD_Color565(*(pBmp), *(pBmp + 1), *(pBmp + 2));
				LCD_WriteMultiple((uint8_t *) &color, 2);
				pBmp += 3;
			}
		} else {
			pBmp = end - 1;
			while (pBmp >= start) {
				color = LCD_Color565(*(pBmp), *(pBmp - 1), *(pBmp - 2));
				LCD_WriteMultiple((uint8_t *) &color, 2);
				pBmp -= 3;
			}
		}
	}
	LCD_CS_IDLE();
	LCD_SetAddrWindow(0, 0, m_width - 1, m_height - 1);
}

#if defined(USE_FATFS)
/**
 * \brief  Draws a bitmap picture from FatFs file.
 *
 * \param  xPos: Bmp X position in the LCD
 * \param  yPos: Bmp Y position in the LCD
 * \param  pFile: Pointer to FIL object with bmp picture
 * \retval None
 */
void LCD_DrawBMPFromFile(int16_t x, int16_t y, FIL * pFile) {
	const uint32_t clusterSize = 19200; // 9600 pixels at a time (almost 19kB)
	uint8_t buf[clusterSize];
	uint32_t readBytes = 0, clusterNum = 0, clusterTotal = 0;
	uint8_t* pBmp;
	uint8_t* start;
	uint8_t* end;
	uint32_t offset = 0, size = 0;
	int32_t height = 0, width = 0;
	uint16_t colordepth = 0;
	uint16_t color = 0;

	/* Read BMP header: 54 bytes = 14 bytes header + 40 bytes DIB header (assuming BITMAPINFOHEADER) */
	f_read(pFile, buf, 54, (UINT *) &readBytes);
	if (readBytes != 54) {
		return;
	}
	pBmp = buf;
	/* Read bitmap size */
	size = *(volatile uint16_t *) (pBmp + 2);
	size |= (*(volatile uint16_t *) (pBmp + 4)) << 16;
	/* Get bitmap data address offset */
	offset = *(volatile uint16_t *) (pBmp + 10);
	offset |= (*(volatile uint16_t *) (pBmp + 12)) << 16;
	/* Calculate total number of clusters to read */
	if ((size - offset) % clusterSize) {
		clusterTotal = (size - offset) / clusterSize + 1;
	} else {
		clusterTotal = (size - offset) / clusterSize;
	}
	/* Read bitmap width */
	width = *(uint16_t *) (pBmp + 18);
	width |= (*(uint16_t *) (pBmp + 20)) << 16;
	/* Read bitmap height */
	height = *(uint16_t *) (pBmp + 22);
	height |= (*(uint16_t *) (pBmp + 24)) << 16;
	/* Read color depth */
	colordepth = *(uint16_t *) (pBmp + 28);

	/* Start drawing */
	LCD_SetAddrWindow(x, y, x + width - 1, y + abs(height) - 1);
	LCD_CS_ACTIVE();
#if	defined(ST7735)
	LCD_WriteReg(ST7735_RAMWR); // Write data to GRAM
#elif
#endif
	LCD_CD_DATA();
	if (height < 0) {
		/* Top-bottom file */
		/* Move read pointer to beginning of pixel data */
		f_lseek(pFile, offset);
		while (clusterNum <= clusterTotal) {
			/* Read new cluster */
			f_read(pFile, buf, clusterSize, (UINT *) &readBytes);
			start = buf;
			end = buf + readBytes;
			pBmp = start;
			/* Draw image */
			if (colordepth == 16) {
				while (pBmp < end) {
					LCD_WriteMultiple(pBmp, 2);
					pBmp += 2;
				}
			} else if (colordepth == 24) {
				while (pBmp < end) {
					color = LCD_Color565(*(pBmp), *(pBmp + 1), *(pBmp + 2));
					LCD_WriteMultiple((uint8_t *) &color, 2);
					pBmp += 3;
				}
			}
			clusterNum++;
		}
	} else {
		/* Bottom-top file */
		clusterNum = clusterTotal;
		while (clusterNum > 0) {
			f_lseek(pFile, offset + (clusterNum - 1) * clusterSize);
			f_read(pFile, buf, clusterSize, (UINT *) &readBytes);
			start = buf;
			end = buf + readBytes;
			pBmp = end - 1;
			if (colordepth == 16) {
				while (pBmp >= start) {
					LCD_WriteMultiple(pBmp - 1, 2);
					pBmp -= 2;
				}
			} else if (colordepth == 24) {
				while (pBmp >= start) {
					color = LCD_Color565(*(pBmp), *(pBmp - 1), *(pBmp - 2));
					LCD_WriteMultiple((uint8_t *) &color, 2);
					pBmp -= 3;
				}
			}
			clusterNum--;
		}
	}
	LCD_CS_IDLE();
	LCD_SetAddrWindow(0, 0, m_width - 1, m_height - 1);
}
#endif

/**
 * \brief Draws a line connecting the two points specified by the coordinate pairs
 *
 * \param x0	The x-coordinate of the first point
 * \param y0	The y-coordinate of the first point
 * \param x1	The x-coordinate of the second point
 * \param y1	The y-coordinate of the second point.
 * \param color	Color
 *
 * \return void
 */
void LCD_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
		uint16_t color) {
	// Bresenham's algorithm - thx wikpedia

	int16_t steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) {
		swap(x0, y0);
		swap(x1, y1);
	}

	if (x0 > x1) {
		swap(x0, x1);
		swap(y0, y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1) {
		ystep = 1;
	} else {
		ystep = -1;
	}

	for (; x0 <= x1; x0++) {
		if (steep) {
			LCD_DrawPixel(y0, x0, color);
		} else {
			LCD_DrawPixel(x0, y0, color);
		}
		err -= dy;
		if (err < 0) {
			y0 += ystep;
			err += dx;
		}
	}
}

/**
 * \brief Draws a horizontal line
 *
 * \param x			The x-coordinate of the first point
 * \param y			The y-coordinate of the first point
 * \param length	Length of the line
 * \param color	Color
 *
 * \return void
 */
void LCD_DrawFastHLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color) {
	int16_t x2;

	// Initial off-screen clipping
	if ((length <= 0) || (y < 0) || (y >= m_height) || (x >= m_width) || ((x2 =
			(x + length - 1)) < 0))
		return;

	if (x < 0) { // Clip left
		length += x;
		x = 0;
	}

	if (x2 >= m_width) { // Clip right
		x2 = m_width - 1;
		length = x2 - x + 1;
	}

	LCD_SetAddrWindow(x, y, x2, y);
	LCD_Flood(color, length);
	LCD_SetAddrWindow(0, 0, m_width - 1, m_height - 1);

}

/**
 * \brief Draws a vertical line
 *
 * \param x		The x-coordinate of the first point
 * \param y		The y-coordinate of the first point
 * \param h		High of the line
 * \param color	Color
 *
 * \return void
 */
void LCD_DrawFastVLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color) {
	int16_t y2;

	// Initial off-screen clipping
	if ((length <= 0) || (x < 0) || (x >= m_width) || (y >= m_height) || ((y2 =
			(y + length - 1)) < 0))
		return;

	if (y < 0) { // Clip top
		length += y;
		y = 0;
	}
	if (y2 >= m_height) { // Clip bottom
		y2 = m_height - 1;
		length = y2 - y + 1;
	}
	LCD_SetAddrWindow(x, y, x, y2);
	LCD_Flood(color, length);
	LCD_SetAddrWindow(0, 0, m_width - 1, m_height - 1);
}

/**
 * \brief Draws a rectangle specified by a coordinate pair, a width, and a height.
 *
 * \param x			The x-coordinate of the upper-left corner of the rectangle to draw
 * \param y			The y-coordinate of the upper-left corner of the rectangle to draw
 * \param w			Width of the rectangle to draw
 * \param h			Height of the rectangle to draw
 * \param color		Color
 *
 * \return void
 */
void LCD_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
		uint16_t color) {
	LCD_DrawFastHLine(x, y, w, color);
	LCD_DrawFastHLine(x, y + h - 1, w, color);
	LCD_DrawFastVLine(x, y, h, color);
	LCD_DrawFastVLine(x + w - 1, y, h, color);
}

/**
 * \brief Draws a filled rectangle specified by a coordinate pair, a width, and a height.
 *
 * \param x				The x-coordinate of the upper-left corner of the rectangle to draw
 * \param y				The y-coordinate of the upper-left corner of the rectangle to draw
 * \param w				Width of the rectangle to draw
 * \param h				Height of the rectangle to draw
 * \param color			Color
 *
 * \return void
 */
void LCD_FillRect(uint16_t x, uint16_t y1, uint16_t w, uint16_t h,
		uint16_t color) {
	int16_t x2, y2;

	// Initial off-screen clipping
	if ((w <= 0) || (h <= 0) || (x >= m_width) || (y1 >= m_height)
			|| ((x2 = x + w - 1) < 0) || ((y2 = y1 + h - 1) < 0))
		return;
	if (x < 0) { // Clip left
		w += x;
		x = 0;
	}
	if (y1 < 0) { // Clip top
		h += y1;
		y1 = 0;
	}
	if (x2 >= m_width) { // Clip right
		x2 = m_width - 1;
		w = x2 - x + 1;
	}
	if (y2 >= m_height) { // Clip bottom
		y2 = m_height - 1;
		h = y2 - y1 + 1;
	}

	LCD_SetAddrWindow(x, y1, x2, y2);
	LCD_Flood(color, (uint32_t) w * (uint32_t) h);
	LCD_SetAddrWindow(0, 0, m_width - 1, m_height - 1);
}

/**
 * \brief Draws an circle defined by a pair of coordinates and radius
 *
 * \param x0		The x-coordinate
 * \param y0		The y-coordinate
 * \param r			Radius
 * \param color		Color
 *
 * \return void
 */
void LCD_DrawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	LCD_DrawPixel(x0, y0 + r, color);
	LCD_DrawPixel(x0, y0 - r, color);
	LCD_DrawPixel(x0 + r, y0, color);
	LCD_DrawPixel(x0 - r, y0, color);

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		LCD_DrawPixel(x0 + x, y0 + y, color);
		LCD_DrawPixel(x0 - x, y0 + y, color);
		LCD_DrawPixel(x0 + x, y0 - y, color);
		LCD_DrawPixel(x0 - x, y0 - y, color);
		LCD_DrawPixel(x0 + y, y0 + x, color);
		LCD_DrawPixel(x0 - y, y0 + x, color);
		LCD_DrawPixel(x0 + y, y0 - x, color);
		LCD_DrawPixel(x0 - y, y0 - x, color);
	}
}

/**
 * \brief Helper function drawing rounded corners
 *
 * \param x0			The x-coordinate
 * \param y0			The y-coordinate
 * \param r				Radius
 * \param cornername	Corner (1, 2, 3, 4)
 * \param color			Color
 *
 * \return void
 */
void LCD_DrawCircleHelper(uint16_t x0, uint16_t y0, uint16_t r,
		uint8_t cornername, uint16_t color) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;
		if (cornername & 0x4) {
			LCD_DrawPixel(x0 + x, y0 + y, color);
			LCD_DrawPixel(x0 + y, y0 + x, color);
		}
		if (cornername & 0x2) {
			LCD_DrawPixel(x0 + x, y0 - y, color);
			LCD_DrawPixel(x0 + y, y0 - x, color);
		}
		if (cornername & 0x8) {
			LCD_DrawPixel(x0 - y, y0 + x, color);
			LCD_DrawPixel(x0 - x, y0 + y, color);
		}
		if (cornername & 0x1) {
			LCD_DrawPixel(x0 - y, y0 - x, color);
			LCD_DrawPixel(x0 - x, y0 - y, color);
		}
	}
}

/**
 * \brief Draws a filled circle defined by a pair of coordinates and radius
 *
 * \param x0		The x-coordinate
 * \param y0		The y-coordinate
 * \param r			Radius
 * \param color		Color
 *
 * \return void
 */
void LCD_FillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color) {
	LCD_DrawFastVLine(x0, y0 - r, 2 * r + 1, color);
	LCD_FillCircleHelper(x0, y0, r, 3, 0, color);
}

/**
 * \brief Helper function to draw a filled circle
 *
 * \param x0			The x-coordinate
 * \param y0			The y-coordinate
 * \param r				Radius
 * \param cornername	Corner (1, 2, 3, 4)
 * \param delta			Delta
 * \param color			Color
 *
 * \return void
 */
void LCD_FillCircleHelper(uint16_t x0, uint16_t y0, uint16_t r,
		uint8_t cornername, uint16_t delta, uint16_t color) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		if (cornername & 0x1) {
			LCD_DrawFastVLine(x0 + x, y0 - y, 2 * y + 1 + delta, color);
			LCD_DrawFastVLine(x0 + y, y0 - x, 2 * x + 1 + delta, color);
		}
		if (cornername & 0x2) {
			LCD_DrawFastVLine(x0 - x, y0 - y, 2 * y + 1 + delta, color);
			LCD_DrawFastVLine(x0 - y, y0 - x, 2 * x + 1 + delta, color);
		}
	}
}

/**
 * \brief Draws a triangle specified by the coordinate pairs.
 *
 * \param x0		The x-coordinate of the first point
 * \param y0		The y-coordinate of the first point
 * \param x1		The x-coordinate of the second point
 * \param y1		The y-coordinate of the second point
 * \param x2		The x-coordinate of the third point
 * \param y2		The y-coordinate of the third point
 * \param color		Color
 *
 * \return void
 */
void LCD_DrawTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
		uint16_t x2, uint16_t y2, uint16_t color) {
	LCD_DrawLine(x0, y0, x1, y1, color);
	LCD_DrawLine(x1, y1, x2, y2, color);
	LCD_DrawLine(x2, y2, x0, y0, color);
}

/**
 * \brief Draws a filled triangle specified by the coordinate pairs.
 *
 * \param x0		The x-coordinate of the first point
 * \param y0		The y-coordinate of the first point
 * \param x1		The x-coordinate of the second point
 * \param y1		The y-coordinate of the second point
 * \param x2		The x-coordinate of the third point
 * \param y2		The y-coordinate of the third point
 * \param color		Color
 *
 * \return void
 */
void LCD_FillTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
		uint16_t x2, uint16_t y2, uint16_t color) {
	int16_t a, b, y, last;

	// Sort coordinates by Y order (y2 >= y1 >= y0)
	if (y0 > y1) {
		swap(y0, y1);
		swap(x0, x1);
	}
	if (y1 > y2) {
		swap(y2, y1);
		swap(x2, x1);
	}
	if (y0 > y1) {
		swap(y0, y1);
		swap(x0, x1);
	}

	if (y0 == y2) { // Handle awkward all-on-same-line case as its own thing
		a = b = x0;
		if (x1 < a)
			a = x1;
		else if (x1 > b)
			b = x1;
		if (x2 < a)
			a = x2;
		else if (x2 > b)
			b = x2;
		LCD_DrawFastHLine(a, y0, b - a + 1, color);
		return;
	}

	int16_t dx01 = x1 - x0, dy01 = y1 - y0, dx02 = x2 - x0, dy02 = y2 - y0,
			dx12 = x2 - x1, dy12 = y2 - y1;
	int32_t sa = 0, sb = 0;

	// For upper part of triangle, find scanline crossings for segments
	// 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
	// is included here (and second loop will be skipped, avoiding a /0
	// error there), otherwise scanline y1 is skipped here and handled
	// in the second loop...which also avoids a /0 error here if y0=y1
	// (flat-topped triangle).
	if (y1 == y2)
		last = y1;   // Include y1 scanline
	else
		last = y1 - 1; // Skip it

	for (y = y0; y <= last; y++) {
		a = x0 + sa / dy01;
		b = x0 + sb / dy02;
		sa += dx01;
		sb += dx02;
		/* longhand:
		 a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
		 b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
		 */
		if (a > b)
			swap(a, b);
		LCD_DrawFastHLine(a, y, b - a + 1, color);
	}

	// For lower part of triangle, find scanline crossings for segments
	// 0-2 and 1-2.  This loop is skipped if y1=y2.
	sa = dx12 * (y - y1);
	sb = dx02 * (y - y0);
	for (; y <= y2; y++) {
		a = x1 + sa / dy12;
		b = x0 + sb / dy02;
		sa += dx12;
		sb += dx02;
		/* longhand:
		 a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
		 b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
		 */
		if (a > b)
			swap(a, b);
		LCD_DrawFastHLine(a, y, b - a + 1, color);
	}
}

/**
 * \brief Draws a rectangle with rounded corners specified by a coordinate pair, a width, and a height.
 *
 * \param x			The x-coordinate of the upper-left corner of the rectangle to draw
 * \param y			The y-coordinate of the upper-left corner of the rectangle to draw
 * \param w			Width of the rectangle to draw
 * \param h			Height of the rectangle to draw
 * \param r			Radius
 * \param color		Color
 *
 * \return void
 */
void LCD_DrawRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
		uint16_t r, uint16_t color) {
	// smarter version
	LCD_DrawFastHLine(x + r, y, w - 2 * r, color); // Top
	LCD_DrawFastHLine(x + r, y + h - 1, w - 2 * r, color); // Bottom
	LCD_DrawFastVLine(x, y + r, h - 2 * r, color); // Left
	LCD_DrawFastVLine(x + w - 1, y + r, h - 2 * r, color); // Right
	// draw four corners
	LCD_DrawCircleHelper(x + r, y + r, r, 1, color);
	LCD_DrawCircleHelper(x + w - r - 1, y + r, r, 2, color);
	LCD_DrawCircleHelper(x + w - r - 1, y + h - r - 1, r, 4, color);
	LCD_DrawCircleHelper(x + r, y + h - r - 1, r, 8, color);
}

/**
 * \brief Draws a filled rounded rectangle specified by a coordinate pair, a width, and a height.
 *
 * \param x				The x-coordinate of the upper-left corner of the rectangle to draw
 * \param y				The y-coordinate of the upper-left corner of the rectangle to draw
 * \param w				Width of the rectangle to draw
 * \param h				Height of the rectangle to draw
 * \param r				Radius
 * \param fillcolor		Color
 *
 * \return void
 */
void LCD_FillRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
		uint16_t r, uint16_t color) {
	// smarter version
	LCD_FillRect(x + r, y, w - 2 * r, h, color);

	// draw four corners
	LCD_FillCircleHelper(x + w - r - 1, y + r, r, 1, h - 2 * r - 1, color);
	LCD_FillCircleHelper(x + r, y + r, r, 2, h - 2 * r - 1, color);
}

/**
 * \brief Draws a character at the specified coordinates
 *
 * \param x			The x-coordinate
 * \param y			The y-coordinate
 * \param c			Character
 * \param color		Character color
 * \param bg		Background color
 * \param size		Character Size
 *
 * \return void
 */
void LCD_DrawChar(uint16_t x, uint16_t y, unsigned char c, uint16_t color,
		uint16_t bg, uint8_t fontindex) {
	uint16_t height, width, bytes;
	uint8_t offset;
	uint32_t charindex = 0;
	uint8_t *pchar;
	uint32_t line = 0;

	height = fonts[fontindex]->Height;
	width = fonts[fontindex]->Width;

	if ((x >= m_width) || // Clip right
			(y >= m_height) || // Clip bottom
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
				LCD_DrawPixel((x + j), y, color);
			} else {
				LCD_DrawPixel((x + j), y, bg);
			}
		}
		y++;
	}
}

/**
 * \brief Print the specified Text
 *
 * \param fmt	Format text
 * \param
 *
 * \return void
 */
void LCD_Printf(const char *fmt, ...) {
	static char buf[256];
	char *p;
	va_list lst;

	va_start(lst, fmt);
	vsprintf(buf, fmt, lst);
	va_end(lst);

	volatile uint16_t height, width;
	height = fonts[m_font]->Height;
	width = fonts[m_font]->Width;

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
				LCD_SetAddrWindow(0, m_cursor_y, m_width - 1,
						m_cursor_y + height);
				LCD_Flood(m_textbgcolor, (long) m_width * height);
				LCD_SetAddrWindow(0, 0, m_width - 1, m_height - 1);
			}
			if (m_cursor_y >= (m_height - height)) {
				m_cursor_y = 0;
				LCD_FillScreen(m_textbgcolor);
			}
			LCD_DrawChar(m_cursor_x, m_cursor_y, *p, m_textcolor, m_textbgcolor,
					m_font);
			m_cursor_x += width;
			if (m_wrap && (m_cursor_x > (m_width - width))) {
				m_cursor_y += height;
				m_cursor_x = 0;
			}
		}
		p++;
	}
}

/**
 * \brief Sets the cursor coordinates
 *
 * \param x		The x-coordinate
 * \param y		The y-coordinate
 *
 * \return void
 */
void LCD_SetCursor(uint16_t x, uint16_t y) {
	m_cursor_x = x;
	m_cursor_y = y;
}

/**
 * \brief Sets the text size
 *
 * \param s	Size
 *
 * \return void
 */
void LCD_SetTextSize(uint8_t s) {
	if (s < 0) {
		m_font = 0;
	} else if (s >= fontsNum) {
		m_font = fontsNum - 1;
	} else {
		m_font = s;
	}
}

/**
 * \brief Sets the text color
 *
 * \param c		Text color
 * \param b		Background color
 *
 * \return void
 */
void LCD_SetTextColor(uint16_t c, uint16_t b) {
	m_textcolor = c;
	m_textbgcolor = b;
}

/**
 * \brief Set Text wrap
 *
 * \param w
 *
 * \return void
 */
void LCD_SetTextWrap(uint8_t w) {
	m_wrap = w;
}

/**
 * \brief Get display rotation
 *
 * \param
 *
 * \return uint8_t rotation
 */
uint8_t LCD_GetRotation() {
	return m_rotation;
}

/**
 * \brief Gets the cursor x-coordinate
 *
 * \param 		
 *
 * \return int16_t x-coordinate
 */
int16_t LCD_GetCursorX(void) {
	return m_cursor_x;
}

/**
 * \brief Gets the cursor Y-coordinate
 *
 * \param 		
 *
 * \return int16_t y-coordinate
 */
int16_t LCD_GetCursorY(void) {
	return m_cursor_y;
}

/**
 * \brief Calucalte 16Bit-RGB
 *
 * \param r	Red
 * \param g	Green
 * \param b	Blue
 *
 * \return uint16_t	16Bit-RGB
 */
inline uint16_t LCD_Color565(uint8_t r, uint8_t g, uint8_t b) {
	return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}






