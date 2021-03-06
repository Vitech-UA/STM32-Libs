/*
 * adc.h
 *
 *  Created on: 22 янв. 2021 г.
 *      Author: viktor.starovit
 */

#ifndef ADC_H_
#define ADC_H_


#include "stm32f051x8.h"

/* Temperature sensor calibration value address */
#define TEMP110_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7C2))
#define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7B8))
#define VDD_CALIB ((uint16_t) (330))
#define VDD_APPLI ((uint16_t) (300))

#define ADC_IN0 0x01
#define ADC_IN1 0x02
#define ADC_IN2 0x04
#define ADC_IN3 0x08
#define ADC_IN4 0x10
#define ADC_IN5 0x20
#define ADC_IN6 0x40
#define ADC_IN7 0x80
#define ADC_IN8 0x100
#define ADC_IN9 0x200
#define ADC_IN10 0x400
#define ADC_IN11 0x800
#define ADC_IN12 0x1000
#define ADC_IN13 0x2000
#define ADC_IN14 0x4000
#define ADC_IN15 0x8000

typedef enum ADC_IN {

	IN0 = 0,
	IN1,
	IN2,
	IN3,
	IN4,
	IN5,
	IN6,
	IN7,
	IN8,
	IN9,
	IN10,
	IN11,
	IN12,
	IN13,
	IN14,
	IN15,
	IN16

} ADC_IN_t;

// Можливі джерела тактування ADC
typedef enum ADC_CLOCK_SOURCE {
	ADC_ADCCLK = 0,   // Тут буде джитер
	ADC_PCLK_DivBy2, // Відсутній джитер з цим джерелом тактування !!!
	ADC_PCLK_DivBy4, // Відсутній джитер з цим джерелом тактування !!!
} ADC_CLOCK_SOURCE_t;

class Adc {

public:
	Adc(ADC_IN_t channel);
	uint16_t GetValue(void); // Запускає перетворення і повертає значення АЦП
	uint16_t GetMcuTemperature(void);
private:
	void SetClockSource(ADC_CLOCK_SOURCE_t ADC_CLK_MODE);
	void Calibrate(void);
	void AddToScan(ADC_IN_t channel);
	void ConfigGpio(ADC_IN_t channel);

	ADC_IN_t CurrentItemChannel;
	ADC_TypeDef *ADC_ITEM;

};

#endif /* ADC_H_ */
