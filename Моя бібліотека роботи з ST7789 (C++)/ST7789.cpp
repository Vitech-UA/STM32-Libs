/*
 * ST7789.cpp
 *
 *  Created on: Mar 21, 2021
 *      Author: Embed Viktor
 */

#include "ST7789.h"

ST7789::ST7789(SPI_TypeDef *Port, SPI_DataSize_t size, GPIO_TypeDef *BlinkPort,
		uint16_t BlinkPin) :
		SPI(Port, size) {
	this->ItemBlinkPin = BlinkPin;
	this->ItemBlinkPort = BlinkPort;
	this->Blink = Gpio(this->ItemBlinkPort, this->ItemBlinkPin);
	this->Blink.SetAsGenerapPurporseOutput(OUTPUT_PP);

}



void ST7789::BrightnessEnable(void) {
	this->Blink.Set();
}
void ST7789::BrightnessDisable(void) {
	this->Blink.Reset();

}
