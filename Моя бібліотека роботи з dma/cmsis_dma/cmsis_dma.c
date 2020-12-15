/*
 * cmsis_dma.c
 *
 *  Created on: 15 дек. 2020 г.
 *      Author: viktor.starovit
 */

#include "cmsis_dma.h"

#ifdef STM32F7
void DMA_Init(DMA_Stream_TypeDef *Channel, uint32_t Src, uint32_t Dst,
		uint32_t Size, uint32_t Conf)
{

	uint32_t tmp = 0;
	DMA_Disable(Channel);
	while((Channel->CR & DMA_SxCR_EN) != 0);
	/* Обнуляю все */
	tmp &= ((uint32_t)~(DMA_SxCR_CHSEL | DMA_SxCR_MBURST | DMA_SxCR_PBURST | \
	                      DMA_SxCR_PL    | DMA_SxCR_MSIZE  | DMA_SxCR_PSIZE  | \
	                      DMA_SxCR_MINC  | DMA_SxCR_PINC   | DMA_SxCR_CIRC   | \
	                      DMA_SxCR_DIR   | DMA_SxCR_CT     | DMA_SxCR_DBM));
	Channel->CR = tmp;
	tmp |= Conf;		   // Закатуємо на результат біти налаштувань
	Channel->CR &= (uint32_t)(~DMA_SxCR_DBM);
	Channel->NDTR = (uint16_t)Size;   // Number of data items to transfer (0 up to 65535)
	Channel->PAR =  Src;  // Адреса периферії
	Channel->M0AR = Dst;   // Адреса у пам'яті
	Channel->CR = (uint32_t)tmp;	   // Записую налаштування у пам'ять
}

void DMA_Enable(DMA_Stream_TypeDef *Channel)
{
	Channel->CR |= DMA_SxCR_EN; // Enable DMA
}

void DMA_Disable(DMA_Stream_TypeDef *Channel)
{
	Channel->CR &= ~(uint32_t) (1<<0); // Enable DMA
}

void DMA_DeInit(DMA_Stream_TypeDef* Channel)
{
    Channel->CR &= ~(1<<0);
    Channel->CR = 0;
    Channel->NDTR = 0;
    Channel->M0AR = 0;
    Channel->M1AR = 0;
}
#endif


void DMA_Init(DMA_Channel_TypeDef *Channel, uint32_t Src, uint32_t Dst,
		uint32_t Size, uint16_t Conf)
{

	uint32_t tmp = 0;

	    tmp = Channel->CCR;		// Копируем биты настройки
	    tmp &= CCR_CLEAR_Mask;	// и стираем все кроме битов EN. А он и так будет 0
	    tmp |= Conf;			// Закатываем на результат наши биты настроек.

	    Channel->CNDTR = Size;	// Заполняем все нужные поля. Размер передчи
	    Channel->CPAR = Src;	// Адрес периферии
	    Channel->CMAR = Dst;		// Адрес в памяти
	    Channel->CCR = tmp;		// Записываем настройки в память.
}

void DMA_Enable(DMA_Channel_TypeDef *Channel)
{
	Channel->CCR |= DMA_CCR_EN; // Enable DMA
}

void DMA_Disable(DMA_Channel_TypeDef *Channel)
{
	Channel->CCR &= ~(uint32_t) (1<<0); // Enable DMA
}

void DMA_DeInit(DMA_Channel_TypeDef* Channel)
{
    Channel->CCR &= (uint16_t)(~DMA_CCR_EN);
    Channel->CCR = 0;
    Channel->CNDTR = 0;
    Channel->CPAR = 0;
    Channel->CMAR = 0;



}
