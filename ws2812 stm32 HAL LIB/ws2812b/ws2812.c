#include "ws2812.h"
#include "math.h"

/*********************************************************************************************************/

extern TIM_HandleTypeDef htim4;
/*********************************************************************************************************/

uint16_t BUF_DMA [ARRAY_LEN] = {0};
uint8_t rgb_temp[15][3];
RGB_t rgb_temp2[LED_COUNT];
HSV_t back_buf [LED_COUNT] = {0};
HSV_t back_buf_temp;
uint16_t DMA_BUF_TEMP[24];

/*********************************************************************************************************/
void Ws2812_Init(void)
{
  int i;
  for(i=DELAY_LEN;i<ARRAY_LEN;i++) BUF_DMA[i] = LOW;
}
/*********************************************************************************************************/
void pixel_rgb_to_buf_dma(uint8_t Rpixel , uint8_t Gpixel, uint8_t Bpixel, uint16_t posX)
{
  volatile uint16_t i;
  for(i=0;i<8;i++)
  {
    if (BitIsSet(Rpixel,(7-i)) == 1)
    {
      BUF_DMA[DELAY_LEN+posX*24+i+8] = HIGH;
    }else
    {
      BUF_DMA[DELAY_LEN+posX*24+i+8] = LOW;
    }
    if (BitIsSet(Gpixel,(7-i)) == 1)
    {
      BUF_DMA[DELAY_LEN+posX*24+i+0] = HIGH;
    }else
    {
      BUF_DMA[DELAY_LEN+posX*24+i+0] = LOW;
    }
    if (BitIsSet(Bpixel,(7-i)) == 1)
    {
      BUF_DMA[DELAY_LEN+posX*24+i+16] = HIGH;
    }else
    {
      BUF_DMA[DELAY_LEN+posX*24+i+16] = LOW;
    }
  }
}
/*********************************************************************************************************/
void HSVtoRGB(void)
{
  uint8_t color[3];
  volatile uint16_t j;
  uint32_t base_V;
	for(j=0;j<LED_COUNT;j++)
  {
		if (back_buf[j].S == 0) 
    {
      color[0] = back_buf[j].V;
      color[1] = back_buf[j].V;
      color[2] = back_buf[j].V;
    }
		else
    {
      base_V = ((255 - back_buf[j].S) * back_buf[j].V) >> 8;
			switch (back_buf[j].H / 60)
      {
        case 0:
					color[0] = back_buf[j].V;
          color[1] = (((back_buf[j].V - base_V) * back_buf[j].H) / 60) + base_V;
          color[2] = base_V;
          break;
        case 1:
					color[0] = (((back_buf[j].V - base_V) * (60 - (back_buf[j].H % 60))) / 60) + base_V;
          color[1] = back_buf[j].V;
          color[2] = base_V;
          break;
        case 2:
          color[0] = base_V;
          color[1] = back_buf[j].V;
          color[2] = (((back_buf[j].V - base_V) * (back_buf[j].H % 60)) / 60) + base_V;
          break;
        case 3:
          color[0] = base_V;
          color[1] = (((back_buf[j].V - base_V) * (60 - (back_buf[j].H % 60))) / 60) + base_V;
          color[2] = back_buf[j].V;
          break;
        case 4:
          color[0] = (((back_buf[j].V - base_V) * (back_buf[j].H % 60)) / 60) + base_V;
          color[1] = base_V;
          color[2] = back_buf[j].V;
          break;
        case 5:
          color[0] = back_buf[j].V;
          color[1] = base_V;
          color[2] = (((back_buf[j].V - base_V) * (60 - (back_buf[j].H % 60))) / 60) + base_V;
          break;
      }
    }
    rgb_temp2[j].R = color[0];
    rgb_temp2[j].G = color[1];
    rgb_temp2[j].B = color[2];
  }
}
/*********************************************************************************************************/
void RgbBufToDma(void)
{
  volatile uint16_t j;
  uint8_t i;
  for(j=0;j<LED_COUNT;j++)
  {
    for(i=0;i<8;i++)
    {
      if (BitIsSet(rgb_temp2[j].R,(7-i)) == 1)
      {
        BUF_DMA[DELAY_LEN+j*24+i+8] = HIGH;
      }else
      {
        BUF_DMA[DELAY_LEN+j*24+i+8] = LOW;
      }
      if (BitIsSet(rgb_temp2[j].G,(7-i)) == 1)
      {
        BUF_DMA[DELAY_LEN+j*24+i+0] = HIGH;
      }else
      {
        BUF_DMA[DELAY_LEN+j*24+i+0] = LOW;
      }
      if (BitIsSet(rgb_temp2[j].B,(7-i)) == 1)
      {
        BUF_DMA[DELAY_LEN+j*24+i+16] = HIGH;
      }else
      {
        BUF_DMA[DELAY_LEN+j*24+i+16] = LOW;
      }
    }
  }
}
/*********************************************************************************************************/
void setValue(void)
{
  uint8_t n=0;
  for(n=0;n<20;n++)
  {
    pixel_rgb_to_buf_dma( rgb_temp[0][0], rgb_temp[0][1], rgb_temp[0][2], n*15);
    pixel_rgb_to_buf_dma( rgb_temp[1][0], rgb_temp[1][1], rgb_temp[1][2], n*15+1);
    pixel_rgb_to_buf_dma( rgb_temp[2][0], rgb_temp[2][1], rgb_temp[2][2], n*15+2);
    pixel_rgb_to_buf_dma( rgb_temp[3][0], rgb_temp[3][1], rgb_temp[3][2], n*15+3);
    pixel_rgb_to_buf_dma( rgb_temp[4][0], rgb_temp[4][1], rgb_temp[4][2], n*15+4);
    pixel_rgb_to_buf_dma( rgb_temp[5][0], rgb_temp[5][1], rgb_temp[5][2], n*15+5);
    pixel_rgb_to_buf_dma( rgb_temp[6][0], rgb_temp[6][1], rgb_temp[6][2], n*15+6);
    pixel_rgb_to_buf_dma( rgb_temp[7][0], rgb_temp[7][1], rgb_temp[7][2], n*15+7);
    pixel_rgb_to_buf_dma( rgb_temp[8][0], rgb_temp[8][1], rgb_temp[8][2], n*15+8);
    pixel_rgb_to_buf_dma( rgb_temp[9][0], rgb_temp[9][1], rgb_temp[9][2], n*15+9);
    pixel_rgb_to_buf_dma(rgb_temp[10][0],rgb_temp[10][1],rgb_temp[10][2],n*15+10);
    pixel_rgb_to_buf_dma(rgb_temp[11][0],rgb_temp[11][1],rgb_temp[11][2],n*15+11);
    pixel_rgb_to_buf_dma(rgb_temp[12][0],rgb_temp[12][1],rgb_temp[12][2],n*15+12);
    pixel_rgb_to_buf_dma(rgb_temp[13][0],rgb_temp[13][1],rgb_temp[13][2],n*15+13);
    pixel_rgb_to_buf_dma(rgb_temp[14][0],rgb_temp[14][1],rgb_temp[14][2],n*15+14);
  }
}
void Ws2812_SetLedColorHSV(uint16_t H, uint8_t S, uint8_t V, uint8_t LedIndex)
{

	    back_buf->H = H;
	    back_buf->S = S;
	    back_buf->V = V;

	    HSVtoRGB();
	    pixel_rgb_to_buf_dma(rgb_temp2->R, rgb_temp2->G, rgb_temp2->B, LedIndex);
	    update();
	    HAL_Delay(20);
}
/*********************************************************************************************************/
void update(void)
{
  HAL_TIM_PWM_Start_DMA(WS2812_PWM_TIMER,WS2812_PWM_CHANEL,(uint32_t*)&BUF_DMA,ARRAY_LEN);
}
/*********************************************************************************************************/

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	HAL_TIM_PWM_Stop_DMA(WS2812_PWM_TIMER, WS2812_PWM_CHANEL);
}
