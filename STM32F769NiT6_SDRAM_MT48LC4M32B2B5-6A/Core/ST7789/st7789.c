#include "stm32f7xx_hal.h"
#include <st7789.h>
#include <string.h>
#include <stdlib.h>
#include "font5x7.h"
#include "font7x11.h"

extern SPI_HandleTypeDef hspi5;


//----------------------Змінні для роботи з DMA---------------
#ifdef USE_DMA

extern volatile uint8_t dma_spi_fl;
extern volatile uint32_t dma_spi_cnt;
uint8_t frame_buffer[65535] = { 0 };
uint8_t lcd_clear_frame_buffer[115200] = { 0x00, };
#endif
//------------------------------------------------------------------------------------------------//
uint8_t ST7789_Width, ST7789_Height;
//------------------------------------------------------------------------------------------------//
void ST7789_Init(uint8_t Width, uint8_t Height) {
	//vIPSSelectAllIps(); /* Перед ініціалізацією треба заселектити всі IPS-екрани */
	ST7789_Width = Width;
	ST7789_Height = Height;

	ST7789_HardReset();
	ST7789_SoftReset();
	ST7789_SleepModeExit();

	ST7789_ColorModeSet(ST7789_ColorMode_65K | ST7789_ColorMode_16bit);
	HAL_Delay(10);
	ST7789_MemAccessModeSet(4, 1, 1, 0);
	HAL_Delay(10);
	ST7789_InversionMode(1);
	HAL_Delay(10);
	ST7789_FillScreen(0);
	ST7789_SetBL(10);
	ST7789_DisplayPower(1);
	HAL_Delay(100);
}
//------------------------------------------------------------------------------------------------//
void ST7789_HardReset(void) {
	HAL_GPIO_WritePin(RES_GPIO_Port, RES_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(RES_GPIO_Port, RES_Pin, GPIO_PIN_SET);
	HAL_Delay(150);
}
//------------------------------------------------------------------------------------------------//
void ST7789_SoftReset(void) {
	ST7789_SendCmd(ST7789_Cmd_SWRESET);
	HAL_Delay(130);
}
//------------------------------------------------------------------------------------------------//
void ST7789_SendCmd(uint8_t Cmd) {
	HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);

	/* Several data should be sent in a raw */
	/* Direct SPI accesses for optimization */

	if (HAL_SPI_Transmit(&hspi5, &Cmd, 1, 100) != HAL_OK) {

		Error_Handler();
	}

}
//------------------------------------------------------------------------------------------------//
void ST7789_SendData(uint8_t Data) {
	HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET);

	/*³������� ����� ����� HAL*/
	/* if(HAL_SPI_Transmit(&hspi2, &Data, 1, 100) != HAL_OK) {

	 Error_Handler();
	 }
	 */
	/*³������� ����� ����� CMSIS*/
	*((__IO uint8_t*) &SPI5->DR) = Data;
}
//------------------------------------------------------------------------------------------------//
void ST7789_SleepModeEnter(void) {
	ST7789_SendCmd(ST7789_Cmd_SLPIN);
	HAL_Delay(500);
}
//------------------------------------------------------------------------------------------------//
void ST7789_SleepModeExit(void) {
	ST7789_SendCmd(ST7789_Cmd_SLPOUT);
	HAL_Delay(500);
}
//------------------------------------------------------------------------------------------------//
void ST7789_ColorModeSet(uint8_t ColorMode) {
	ST7789_SendCmd(ST7789_Cmd_COLMOD);
	ST7789_SendData(ColorMode & 0x77);
}
//------------------------------------------------------------------------------------------------//
void ST7789_MemAccessModeSet(uint8_t Rotation, uint8_t VertMirror,
		uint8_t HorizMirror, uint8_t IsBGR) {
	uint8_t Value;
	Rotation &= 7;

	ST7789_SendCmd(ST7789_Cmd_MADCTL);

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

	ST7789_SendData(Value);
}
//------------------------------------------------------------------------------------------------//
void ST7789_InversionMode(uint8_t Mode) {
	if (Mode)
		ST7789_SendCmd(ST7789_Cmd_INVON);
	else
		ST7789_SendCmd(ST7789_Cmd_INVOFF);
}
//------------------------------------------------------------------------------------------------//
void ST7789_FillScreen(uint16_t color) {
	ST7789_FillRect(0, 0, ST7789_Width, ST7789_Height, color);
}
//------------------------------------------------------------------------------------------------//
void ST7789_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
	// 0 120 239 120
	if ((x >= ST7789_Width) || (y >= ST7789_Height))
		return;
	if ((x + w) > ST7789_Width)
		w = ST7789_Width - x;
	if ((y + h) > ST7789_Height)
		h = ST7789_Height - y;
	ST7789_SetWindow(x, y, x + w - 1, y + h - 1);
	for (uint32_t i = 0; i < (h * w); i++)
		ST7789_RamWrite(&color, 1);
}
//------------------------------------------------------------------------------------------------//
void ST7789_SetWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
	ST7789_ColumnSet(x0, x1);
	ST7789_RowSet(y0, y1);
	ST7789_SendCmd(ST7789_Cmd_RAMWR);
}
//------------------------------------------------------------------------------------------------//
void ST7789_RamWrite(uint16_t *pBuff, uint16_t Len) {
	while (Len--) {
		ST7789_SendData(*pBuff >> 8);
		ST7789_SendData(*pBuff & 0xFF);
	}
}
//------------------------------------------------------------------------------------------------//
void ST7789_ColumnSet(uint16_t ColumnStart, uint16_t ColumnEnd) {
	if (ColumnStart > ColumnEnd)
		return;
	if (ColumnEnd > ST7789_Width)
		return;

	ColumnStart += ST7789_X_Start;
	ColumnEnd += ST7789_X_Start;

	ST7789_SendCmd(ST7789_Cmd_CASET);
	ST7789_SendData(ColumnStart >> 8);
	ST7789_SendData(ColumnStart & 0xFF);
	ST7789_SendData(ColumnEnd >> 8);
	ST7789_SendData(ColumnEnd & 0xFF);
}
//------------------------------------------------------------------------------------------------//
void ST7789_RowSet(uint16_t RowStart, uint16_t RowEnd) {
	if (RowStart > RowEnd)
		return;
	if (RowEnd > ST7789_Height)
		return;

	RowStart += ST7789_Y_Start;
	RowEnd += ST7789_Y_Start;

	ST7789_SendCmd(ST7789_Cmd_RASET);
	ST7789_SendData(RowStart >> 8);
	ST7789_SendData(RowStart & 0xFF);
	ST7789_SendData(RowEnd >> 8);
	ST7789_SendData(RowEnd & 0xFF);
}
//------------------------------------------------------------------------------------------------//
void ST7789_SetBL(uint8_t Value) {
	if (Value > 100)
		Value = 100;

}
//------------------------------------------------------------------------------------------------//
void ST7789_DisplayPower(uint8_t On) {
	if (On)
		ST7789_SendCmd(ST7789_Cmd_DISPON);
	else
		ST7789_SendCmd(ST7789_Cmd_DISPOFF);
}
//------------------------------------------------------------------------------------------------//
void ST7789_DrawRectangle(int16_t x1, int16_t y1, int16_t x2, int16_t y2,
		uint16_t color) {
	ST7789_DrawLine(x1, y1, x1, y2, color);
	ST7789_DrawLine(x2, y1, x2, y2, color);
	ST7789_DrawLine(x1, y1, x2, y1, color);
	ST7789_DrawLine(x1, y2, x2, y2, color);
}
//------------------------------------------------------------------------------------------------//
void ST7789_DrawRectangleFilled(int16_t x1, int16_t y1, int16_t x2, int16_t y2,
		uint16_t fillcolor) {
	if (x1 > x2)
		SwapInt16Values(&x1, &x2);
	if (y1 > y2)
		SwapInt16Values(&y1, &y2);
	ST7789_FillRect(x1, y1, x2 - x1, y2 - y1, fillcolor);
}
//------------------------------------------------------------------------------------------------//
void ST7789_DrawLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2,
		uint16_t color) {
	// ������������ �����
	if (x1 == x2) {
		// ������������ ����� ������� �������
		if (y1 > y2)
			ST7789_FillRect(x1, y2, 1, y1 - y2 + 1, color);
		else
			ST7789_FillRect(x1, y1, 1, y2 - y1 + 1, color);
		return;
	}

	// �������������� �����
	if (y1 == y2) {
		// ������������ ����� ������� �������
		if (x1 > x2)
			ST7789_FillRect(x2, y1, x1 - x2 + 1, 1, color);
		else
			ST7789_FillRect(x1, y1, x2 - x1 + 1, 1, color);
		return;
	}

	ST7789_DrawLine_Slow(x1, y1, x2, y2, color);
}
//------------------------------------------------------------------------------------------------//
void SwapInt16Values(int16_t *pValue1, int16_t *pValue2) {
	int16_t TempValue = *pValue1;
	*pValue1 = *pValue2;
	*pValue2 = TempValue;
}
//------------------------------------------------------------------------------------------------//
void ST7789_DrawLine_Slow(int16_t x1, int16_t y1, int16_t x2, int16_t y2,
		uint16_t color) {
	const int16_t deltaX = abs(x2 - x1);
	const int16_t deltaY = abs(y2 - y1);
	const int16_t signX = x1 < x2 ? 1 : -1;
	const int16_t signY = y1 < y2 ? 1 : -1;

	int16_t error = deltaX - deltaY;

	ST7789_DrawPixel(x2, y2, color);

	while (x1 != x2 || y1 != y2) {
		ST7789_DrawPixel(x1, y1, color);
		const int16_t error2 = error * 2;

		if (error2 > -deltaY) {
			error -= deltaY;
			x1 += signX;
		}
		if (error2 < deltaX) {
			error += deltaX;
			y1 += signY;
		}
	}
}
//------------------------------------------------------------------------------------------------//
void ST7789_DrawPixel(int16_t x, int16_t y, uint16_t color) {
	if ((x < 0) || (x >= ST7789_Width) || (y < 0) || (y >= ST7789_Height))
		return;

	ST7789_SetWindow(x, y, x, y);
	ST7789_RamWrite(&color, 1);
}
//------------------------------------------------------------------------------------------------//
void ST7789_DrawCircleFilled(int16_t x0, int16_t y0, int16_t radius,
		uint16_t fillcolor) {
	int x = 0;
	int y = radius;
	int delta = 1 - 2 * radius;
	int error = 0;

	while (y >= 0) {
		ST7789_DrawLine(x0 + x, y0 - y, x0 + x, y0 + y, fillcolor);
		ST7789_DrawLine(x0 - x, y0 - y, x0 - x, y0 + y, fillcolor);
		error = 2 * (delta + y) - 1;

		if (delta < 0 && error <= 0) {
			++x;
			delta += 2 * x + 1;
			continue;
		}

		error = 2 * (delta - x) - 1;

		if (delta > 0 && error > 0) {
			--y;
			delta += 1 - 2 * y;
			continue;
		}

		++x;
		delta += 2 * (x - y);
		--y;
	}
}
//------------------------------------------------------------------------------------------------//
void ST7789_DrawCircle(int16_t x0, int16_t y0, int16_t radius, uint16_t color) {
	int x = 0;
	int y = radius;
	int delta = 1 - 2 * radius;
	int error = 0;

	while (y >= 0) {
		ST7789_DrawPixel(x0 + x, y0 + y, color);
		ST7789_DrawPixel(x0 + x, y0 - y, color);
		ST7789_DrawPixel(x0 - x, y0 + y, color);
		ST7789_DrawPixel(x0 - x, y0 - y, color);
		error = 2 * (delta + y) - 1;

		if (delta < 0 && error <= 0) {
			++x;
			delta += 2 * x + 1;
			continue;
		}

		error = 2 * (delta - x) - 1;

		if (delta > 0 && error > 0) {
			--y;
			delta += 1 - 2 * y;
			continue;
		}

		++x;
		delta += 2 * (x - y);
		--y;
	}
}
//------------------------------------------------------------------------------------------------//
void ST7789_DrawBMP(uint16_t x, uint16_t y, const uint8_t *pArr) {
	uint8_t *pBmp;
	uint8_t *start;
	uint8_t *end;
	uint32_t offset = 0, size = 0;
	int32_t height = 0, width = 0;
	uint16_t colordepth = 0;
	uint16_t color = 0;

	pBmp = (uint8_t*) pArr;
	/* Read bitmap size */
	size = *(volatile uint16_t*) (pBmp + 2);
	size |= (*(volatile uint16_t*) (pBmp + 4)) << 16;
	/* Get bitmap data address offset */
	offset = *(volatile uint16_t*) (pBmp + 10);
	offset |= (*(volatile uint16_t*) (pBmp + 12)) << 16;
	/* Read bitmap width */
	width = *(uint16_t*) (pBmp + 18);
	width |= (*(uint16_t*) (pBmp + 20)) << 16;
	/* Read bitmap height */
	height = *(uint16_t*) (pBmp + 22);
	height |= (*(uint16_t*) (pBmp + 24)) << 16;
	/* Read color depth */
	colordepth = *(uint16_t*) (pBmp + 28);

	start = (uint8_t*) pBmp + offset;
	end = (uint8_t*) pBmp + size;

	ST7789_SetWindow(x, y, x + width - 1, y + abs(height) - 1);
	HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET);
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
				//color = LCD_Color565(*(pBmp), *(pBmp + 1), *(pBmp + 2));
				LCD_WriteMultiple((uint8_t*) &color, 2);
				pBmp += 3;
			}
		} else {
			pBmp = end - 1;
			while (pBmp >= start) {
				//color = LCD_Color565(*(pBmp), *(pBmp - 1), *(pBmp - 2));
				LCD_WriteMultiple((uint8_t*) &color, 2);
				pBmp -= 3;
			}
		}
	}
	ST7789_SetWindow(0, 0, width - 1, height - 1);
}
//------------------------------------------------------------------------------------------------//
inline void LCD_WriteMultiple(uint8_t *pData, uint32_t size) {

	/* Several data should be sent in a raw */
	/* Direct SPI accesses for optimization */
	for (uint32_t counter = size; counter != 0; counter -= 2) {
		while (((SPI5->SR) & SPI_FLAG_TXE) != SPI_FLAG_TXE) {
		}
		/* Need to invert bytes for LCD*/
		*((__IO uint8_t*) &(SPI5->DR)) = *(pData + 1);
		while (((SPI5->SR) & SPI_FLAG_TXE) != SPI_FLAG_TXE) {
		}
		*((__IO uint8_t*) &(SPI5->DR)) = *pData;
		pData += 2;
	}

	/* очікую готовності SPI після передачі даних по SPI */
	while (((SPI5->SR) & SPI_FLAG_BSY) != RESET) {
	}
}
//------------------------------------------------------------------------------------------------//
inline uint16_t LCD_Color565(uint8_t r, uint8_t g, uint8_t b) {
	return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

//---------------Функції роботи з текстом---------------------------------------------------------//
void ST7789_DrawChar_5x8(uint16_t x, uint16_t y, uint16_t TextColor,
		uint16_t BgColor, uint8_t TransparentBg, unsigned char c) {
	if ((x >= 240) || (y >= 240) || ((x + 4) < 0) || ((y + 7) < 0))
		return;
	if (c < 128)
		c = c - 32;
	if (c >= 144 && c <= 175)
		c = c - 48;
	if (c >= 128 && c <= 143)
		c = c + 16;
	if (c >= 176 && c <= 191)
		c = c - 48;
	if (c > 191)
		return;
	for (uint8_t i = 0; i < 6; i++) {
		uint8_t line;
		if (i == 5)
			line = 0x00;
		else
			line = font[(c * 5) + i];
		for (uint8_t j = 0; j < 8; j++) {
			if (line & 0x01)
				ST7789_DrawPixel(x + i, y + j, TextColor);
			else if (!TransparentBg)
				ST7789_DrawPixel(x + i, y + j, BgColor);
			line >>= 1;
		}
	}
}
//------------------------------------------------------------------------------------------------//
void ST7789_DrawChar_7x11(uint16_t x, uint16_t y, uint16_t TextColor,
		uint16_t BgColor, uint8_t TransparentBg, unsigned char c) {
	uint8_t i, j;
	uint8_t buffer[11];

	if ((x >= 240) || (y >= 240) || ((x + 4) < 0) || ((y + 7) < 0))
		return;

	// Copy selected simbol to buffer
	memcpy(buffer, &font7x11[(c - 32) * 11], 11);
	for (j = 0; j < 11; j++) {
		for (i = 0; i < 7; i++) {
			if ((buffer[j] & (1 << i)) == 0) {
				if (!TransparentBg)
					ST7789_DrawPixel(x + i, y + j, BgColor);
			} else
				ST7789_DrawPixel(x + i, y + j, TextColor);
		}
	}
}
//------------------------------------------------------------------------------------------------//
void ST7789_print_5x8(uint16_t x, uint16_t y, uint16_t TextColor,
		uint16_t BgColor, uint8_t TransparentBg, char *str) {
	unsigned char type = *str;
	if (type >= 128)
		x = x - 3;
	while (*str) {
		ST7789_DrawChar_5x8(x, y, TextColor, BgColor, TransparentBg, *str++);
		unsigned char type = *str;
		if (type >= 128)
			x = x + 3;
		else
			x = x + 6;
	}
}
//------------------------------------------------------------------------------------------------//
void ST7789_print_7x11(uint16_t x, uint16_t y, uint16_t TextColor,
		uint16_t BgColor, uint8_t TransparentBg, char *str) {
	unsigned char type = *str;
	if (type >= 128)
		x = x - 3;
	while (*str) {
		ST7789_DrawChar_7x11(x, y, TextColor, BgColor, TransparentBg, *str++);
		unsigned char type = *str;
		if (type >= 128)
			x = x + 8;
		else
			x = x + 8;
	}
}
//------------------------------------------------------------------------------------------------//

#ifdef USE_DMA
//-Функції для Виводу графічної інфи через DMA--------------------------------------//

void vFinaLCD_ClearScreenDMA(uint8_t x, uint8_t y, uint8_t width,
		uint8_t height) {

	uint32_t bitmap_size = width * height * 2;

	/*Якщо розмір массиву менше розміру максимального буфера, то впакуємо передачу в одну транзакцію DMA*/
	if (bitmap_size < 65535) {

		ST7789_SetWindow(x, y, x + width - 1, y + height - 1);
		for (uint16_t i = 0; i <= bitmap_size; i++) {

			frame_buffer[i] = 0x00;
		}

		DC_DATA();
		HAL_SPI_Transmit_DMA(&hspi5, (uint8_t*) frame_buffer, bitmap_size);
		while (SPI5->SR & SPI_SR_BSY) {
		}
	}
	/*Якщо розмір массиву більше розміру максимального буфера, то впакуємо передачу в дві транзакції DMA*/
	else {
		uint32_t HALF_BUFFER_SIZE = 0;

		HALF_BUFFER_SIZE = bitmap_size / 2; // розділяю массив bitmap на 2 рівні частинии

		//Виставляю кадрове вікно для першого пів-кадра!!!
		ST7789_SetWindow(x, y, x + width - 1, y + (height / 2) - 1);

		DC_DATA();
		HAL_SPI_Transmit_DMA(&hspi5, (uint8_t*) lcd_clear_frame_buffer,
				HALF_BUFFER_SIZE);
		// Обов'язково чекаю флаг готовності SPI
		while (SPI5->SR & SPI_SR_BSY) {
		}

		// Виставляю кадрове вікно для другого пів-кадра!!!

		ST7789_SetWindow(x, y + (height / 2), x + width - 1, y + height - 1);

		DC_DATA();
		HAL_SPI_Transmit_DMA(&hspi5, (uint8_t*) lcd_clear_frame_buffer,
				HALF_BUFFER_SIZE);
		// Обов'язково чекаю флаг готовності SPI
		while (SPI5->SR & SPI_SR_BSY) {
		}
	}
}

//---------------Вивід бітмап масиву 240*240 без заголовку через DMA------------------------------//
void vFinaLCD_DrawImage240_240_FromFlashDMA(const uint8_t *bmp) {
	uint32_t i, HALF_BUFFER_SIZE;

	HALF_BUFFER_SIZE = 57600; //Пачка даних байт, що передається в дма за 1 транзакцію макс. 65535

	ST7789_ColumnSet(0, 239);
	ST7789_RowSet(0, 119);
	ST7789_SendCmd(ST7789_Cmd_RAMWR);

	for (i = 0; i <= 57600; i++) {

		frame_buffer[i] = bmp[i + 1];
	}

	DC_DATA();
	HAL_SPI_Transmit_DMA(&hspi5, (uint8_t*) frame_buffer, HALF_BUFFER_SIZE);

	while (SPI5->SR & SPI_SR_BSY) {

	}

	ST7789_ColumnSet(0, 239);
	ST7789_RowSet(120, 239);

	ST7789_SendCmd(ST7789_Cmd_RAMWR);

	for (i = 0; i <= 57600; i++) {

		frame_buffer[i] = bmp[i + 1 + HALF_BUFFER_SIZE];
	}
	DC_DATA();
	HAL_SPI_Transmit_DMA(&hspi5, (uint8_t*) frame_buffer, HALF_BUFFER_SIZE);

}

//---------------Вивід бітмап масиву по координатах без заголовку через DMA-----------------------//
void vFinaLCD_DrawImageFromFlashDMA(uint8_t x, uint8_t y, uint8_t width,
		uint8_t height, const uint8_t *bmp) {
	uint32_t i, HALF_BUFFER_SIZE;
	uint32_t MAX_DMA_BUFFER_SIZE = 65535;
	uint32_t bitmap_size = width * height * 2;

	/*Якщо розмір массиву менше розміру максимального буфера, то впакуємо передачу в одну транзакцію DMA*/
	if (bitmap_size < MAX_DMA_BUFFER_SIZE) {

		ST7789_SetWindow(x, y, x + width - 1, y + height - 1);
		for (i = 0; i <= bitmap_size; i++) {

			frame_buffer[i] = bmp[i];
		}

		DC_DATA();
		HAL_SPI_Transmit_DMA(&hspi5, (uint8_t*) frame_buffer, bitmap_size);
		while (SPI5->SR & SPI_SR_BSY) {
		}
	}
	/*Якщо розмір массиву більше розміру максимального буфера, то впакуємо передачу в дві транзакції DMA*/
	else {

		HALF_BUFFER_SIZE = bitmap_size / 2; // розділяю массив bitmap на 2 рівні частинии

		//Виставляю кадрове вікно для першого пів-кадра!!!
		ST7789_SetWindow(x, y, x + width - 1, y + (height / 2) - 1);
		// Заповнюю фрейм-буфер половиною об'єму bmp
		for (i = 0; i <= HALF_BUFFER_SIZE; i++) {

			frame_buffer[i] = bmp[i];
		}

		DC_DATA();
		HAL_SPI_Transmit_DMA(&hspi5, (uint8_t*) frame_buffer, HALF_BUFFER_SIZE);
		// Обов'язково чекаю флаг готовності SPI
		while (SPI5->SR & SPI_SR_BSY) {
		}

		// Виставляю кадрове вікно для другого пів-кадра!!!

		ST7789_SetWindow(x, y + (height / 2), x + width - 1, y + height - 1);
		// Заповнюю фрейм-буфер половиною об'єму bmp
		for (i = 0; i <= HALF_BUFFER_SIZE; i++) {

			frame_buffer[i] = bmp[i + HALF_BUFFER_SIZE];
		}
		DC_DATA();
		HAL_SPI_Transmit_DMA(&hspi5, (uint8_t*) frame_buffer, HALF_BUFFER_SIZE);
		// Обов'язково чекаю флаг готовності SPI
		while (SPI5->SR & SPI_SR_BSY) {
		}
	}

}

#endif

