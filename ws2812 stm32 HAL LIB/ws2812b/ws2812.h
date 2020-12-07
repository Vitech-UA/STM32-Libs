#ifndef WS2812_H_
#define WS2812_H_
//--------------------------------------------------
#include "stm32f7xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#define WS2812_PWM_TIMER &htim4
#define WS2812_PWM_CHANEL TIM_CHANNEL_3


//--------------------------------------------------
#define DELAY_LEN 48
#define BRIGHT 255
#define LED_COUNT 4
#define ARRAY_LEN DELAY_LEN + LED_COUNT*24
#define HIGH 15
#define LOW 8

#define HSV_RED 360
#define HSV_BLUE 240
#define HSV_GREEN 120

//--------------------------------------------------
#define BitIsSet(reg, bit) ((reg & (1<<bit)) != 0)
//--------------------------------------------------
typedef struct 
{
    uint16_t H;
    uint8_t S;
    uint8_t V;
} HSV_t;
//--------------------------------------------------
typedef struct 
{
    uint8_t R;
    uint8_t G;
    uint8_t B;
} RGB_t;
//--------------------------------------------------


	void HSVtoRGB(void);
	void RgbBufToDma(void);
	void Ws2812_Init(void);
	void pixel_rgb_to_buf_dma(uint8_t Rpixel , uint8_t Gpixel, uint8_t Bpixel, uint16_t posX);
	void setValue(void);
	void update(void);
	void Ws2812_SetLedColorHSV(uint16_t H, uint8_t S, uint8_t V, uint8_t LedIndex);




//--------------------------------------------------
#endif /* WS2812_H_ */
