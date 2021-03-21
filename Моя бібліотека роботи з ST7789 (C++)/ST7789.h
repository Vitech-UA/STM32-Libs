/*
 * ST7789.h
 *
 *  Created on: Mar 21, 2021
 *      Author: Embed Viktor
 */

#ifndef ST7789_H_
#define ST7789_H_

#include "SPI.h"

class ST7789: public SPI {
public:
	ST7789(SPI_TypeDef *Port, SPI_DataSize_t size, GPIO_TypeDef *BlinkPort,
			uint16_t BlinkPin);
	void BrightnessEnable(void);
	void BrightnessDisable(void);
private:
	GPIO_TypeDef *ItemBlinkPort;
	uint16_t ItemBlinkPin;
	Gpio Blink;
};

#endif /* ST7789_H_ */
