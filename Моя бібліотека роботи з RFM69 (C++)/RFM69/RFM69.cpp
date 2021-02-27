/*
 * RFM69.cpp
 *
 *  Created on: 25 февр. 2021 г.
 *      Author: Embed Viktor
 */

#include <RFM69.h>

RFM69::RFM69(SPI_TypeDef *Port, GPIO_TypeDef *SS_Port, uint16_t SS_Pin,
		GPIO_TypeDef *Interrupt_Port, uint16_t Interrupt_Pin) :
		SPI(Port) {

	this->SPI_ITEM = Port;
	this->nCS_PORT = SS_Port;
	this->nCS_PIN = SS_Pin;

	this->Interrupt_PORT = Interrupt_Port;
	this->Interrupt_PIN = Interrupt_Pin;

}

void RFM69::WriteReg(uint8_t addr, uint8_t val) {
	this->nCS_Low();
	this->SPI_ITEM->DR = (uint16_t) ((addr << 8) | val);
	while ((this->SPI_ITEM->SR & SPI_SR_BSY) == SPI_SR_BSY)
		;
	this->nCS_High();
}
