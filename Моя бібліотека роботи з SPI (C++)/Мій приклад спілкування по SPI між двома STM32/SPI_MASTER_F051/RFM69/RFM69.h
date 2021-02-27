/*
 * RFM69.h
 *
 *  Created on: 25 февр. 2021 г.
 *      Author: Embed Viktor
 */

#ifndef RFM69_H_
#define RFM69_H_

#include "SPI.h"

class RFM69 : public SPI {

public:
	RFM69(SPI_TypeDef *Port, GPIO_TypeDef *SS_Port, uint16_t SS_Pin,
			GPIO_TypeDef *Interrupt_Port, uint16_t Interrupt_Pin);
	void WriteReg(uint8_t addr, uint8_t val);

private:
	SPI_TypeDef *SPI_ITEM;
	GPIO_TypeDef *nCS_PORT;
	uint16_t nCS_PIN;
	GPIO_TypeDef *Interrupt_PORT;
	uint16_t Interrupt_PIN;
};
#endif /* RFM69_H_ */
