/*
 * SPI.cpp
 *
 *  Created on: 14 февр. 2021 г.
 *      Author: Embed Viktor
 */

#include <SPI.h>

SPI::SPI(SPI_TypeDef *Port, SPI_mode_t Mode) {
	// TODO Допиляти цей конструктор
	this->SPI_Item = Port;

	switch (Mode) {
	case Full_Duplex_Master:

		break;
	case Full_Duplex_Slave:

		break;
	case Half_Duplex_Master:

		break;
	case Half_Duplex_Slave:

		break;
	case Transmit_Only_Master:

		break;
	case Transmit_Only_Slave:

		break;
	case Receive_Only_Master:

		break;
	case Receive_Only_Slave:

		break;
	default:
		break;
	}

}

