/*
 * SPI.cpp
 *
 *  Created on: 14 февр. 2021 г.
 *      Author: Embed Viktor
 */

#include "SPI.h"


SPI::SPI(SPI_TypeDef *Port, SPI_mode_t Mode) {

	this->MISO_PORT = GPIOA;
	this->MISO_PIN = 6;

	this->MOSI_PORT = GPIOA;
	this->MISO_PIN = 7;

	this->SCK_PORT = GPIOA;
	this->SCK_PIN = 5;

}

void SPI::InitGpio() {

}

