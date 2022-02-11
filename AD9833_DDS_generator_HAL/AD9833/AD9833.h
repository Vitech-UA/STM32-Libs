/*
 * ad9833.h
 *
 *  Created on: Feb 10, 2022
 *      Author: Vitech-UA
 */

#ifndef AD9833_H_
#define AD9833_H_

#include "main.h"
#include "stdint.h"
#include <math.h>
#define ENABLE(x,y)              ((x) |= (1<<(y)))
#define DISABLE(x,y)             ((x) &=~ (1<<(y)))
#define TOGGLE(x,y)              ((x) ^= (1<<(y)))
#define CHECKBIT(x,y)            ((x) & (1<<(y)))
#define BIT(x)                   (1 << (x))

//-------------------------------
#define WRITE_TO_FREQ1_REG	 0x8000
#define WRITE_TO_FREQ0_REG	 0x4000
#define WRITE_TO_PHASE1_REG	 0xE000
#define WRITE_TO_PHASE0_REG	 0xC000
//-------------------------------
#define B28	   		         13 /* 28-BIT/14-BIT WORD, D13 */
#define HLB  		 		 12 /* HIGH/LOW BYTE, D12 */
//-------------------------------
#define USE_FREQ	 		 11 /* D11 */
#define USE_PHASE	 		 10 /* D10 */
//-------------------------------
#define RESET	 		 	 8  /* D8 */
#define STOP_MCLK 		 	 7  /* SLEEP1, D7 */
#define STOP_DAC	 		 6  /* SLEEP12, D6 */
#define DISCONNECT_DAC		 5  /* OPBITEN, D5 */
#define DAC_MSB				 3  /* DIV2, D3 */
#define BYPASS_SINROM		 1  /* MODE, D1 */
//-------------------------------
#define FMCLK				 25000000 /*Частота кварцевого резонатора AD9833 Hz */
#define USE_FREQ0_REG		 0
#define USE_FREQ1_REG		 1
#define USE_PHASE0_REG		 0
#define USE_PHASE1_REG		 1
//-------------------------------

#define VMAX				 3300

typedef enum
{
	SINUS,
	TRIANGLE,
	SQUARE,

} ad9833_waveform_t;

typedef enum{
	POWER_OFF,
	POWER_ON,

}ad9833_power_t;

void ad9833_write_16bit(uint16_t data);
void ad9833_set_wave_form(ad9833_waveform_t waveform);
void ad9833_set_wave_param(ad9833_waveform_t waveform, float frequency, float phase);
void ad9833_set_power(ad9833_power_t state);


#endif /* AD9833_H_ */
