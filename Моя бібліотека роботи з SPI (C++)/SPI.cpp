/*
 * SPI.cpp
 *
 *  Created on: 14 февр. 2021 г.
 *      Author: Embed Viktor
 */

#include "SPI.h"

extern "C" void SPI1_IRQHandler();

SPI::SPI(SPI_TypeDef *Port) {
	this->SPI_ITEM = Port;
	this->InitGpio();
	this->EnableClk();
	this->SetClockPrsc(fPCLK_DIV_By_2);
	this->EnableSoftwareSlaveManagment();
	this->EnableMotorollaMode();
	this->Config();
	this->SetFrameSize(DataSize_16B);
	//this->SetFrameSize(DataSize_8B);
	this->SetClockPhase(CPHA0);
	this->SetClockPolarity(CPOL0);
	this->SetMsbLsbFirst(MSB_First);
	this->Enable();
}
void SPI::Config() {
	//TODO: Повиносити звідси строки в окремі ф-ї елементи

	//this->SPI_ITEM->CR1 |= SPI_CR1_BIDIMODE; // 0: 2-проводной режим роботи з однонаправленою передачею по лініях даних
	// 1: 1-проводной режим роботи з двунаправленою передачею по лініях даних

	//this->SPI_ITEM->CR1 |= SPI_CR1_BIDIOE;   // 1: Лише прийом (працює лише при BIDIMODE = 1)
	// 0: Лише передача (працює лише при BIDIMODE = 1)

	this->SPI_ITEM->CR1 |= SPI_CR1_MSTR;       // 1: Master configuration
											   // 0: Slave configuration

	this->SPI_ITEM->CR2 &= ~SPI_CR2_DS;       // Clear bitfield
}

void SPI::InitGpio(void) {
	// Визначення GPIO
	this->MISO_PORT = GPIOA;
	this->MISO_PIN = 6;

	this->MOSI_PORT = GPIOB;
	this->MOSI_PIN = 5;

	this->SCK_PORT = GPIOA;
	this->SCK_PIN = 5;

	this->nSC_PORT = GPIOC;
	this->nSC_PIN = 4;

	// Ініціалізація GPIO
	Gpio MISO = Gpio(this->MISO_PORT, this->MISO_PIN);
	MISO.SetAsAF(AF0, OUTPUT_PP);

	Gpio MOSI = Gpio(this->MOSI_PORT, this->MOSI_PIN);
	MOSI.SetAsAF(AF0, OUTPUT_PP);

	Gpio SCK = Gpio(this->SCK_PORT, this->SCK_PIN);
	SCK.SetAsAF(AF0, OUTPUT_PP);

	Gpio nCS = Gpio(this->nSC_PORT, this->nSC_PIN);
	nCS.SetAsGenerapPurporseOutput(OUTPUT_PP);
}

void SPI::EnableClk(void) {
	if (this->SPI_ITEM == SPI1) {
		RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	}
	if (this->SPI_ITEM == SPI2) {
		RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
	}
}

void SPI::SetFrameSize(SPI_DataSize_t Size) {
	if (Size == DataSize_8B) {
		this->SPI_ITEM->CR2 |= 0x07 << SPI_CR2_DS_Pos; // 8 Bit frame
	}
	if (Size == DataSize_16B) {
		this->SPI_ITEM->CR2 |= (0x0F << SPI_CR2_DS_Pos); // 16 Bit frame
	}
}

void SPI::SetClockPrsc(SetClockPrsc_t Prescaler) {
	this->SPI_ITEM->CR1 |= (Prescaler << SPI_CR1_BR_Pos);
}

void SPI::EnableSoftwareSlaveManagment(void) {
	this->SPI_ITEM->CR1 |= SPI_CR1_SSM;  // 1: Software slave management enabled
	this->SPI_ITEM->CR1 |= SPI_CR1_SSI;  // 1: Internal slave select
}

void SPI::DisableSoftwareSlaveManagment(void) {
	this->SPI_ITEM->CR1 &= ~SPI_CR1_SSM; // 1: Software slave management disabled
}

void SPI::EnableMotorollaMode(void) {
	this->SPI_ITEM->CR2 &= ~SPI_CR2_FRF;      // Motorolla mode
}

void SPI::nCS_Low(void) {
	this->nSC_PORT->BSRR |= ((1 << this->nSC_PIN) << 16U); // BIT RESET
}

void SPI::nCS_High(void) {
	this->nSC_PORT->BSRR |= (1 << this->nSC_PIN); // BIT SET
}

void SPI::Enable(void) {
	this->SPI_ITEM->CR1 |= SPI_CR1_SPE;
}

void SPI::Disable(void) {
	this->SPI_ITEM->CR1 &= ~SPI_CR1_SPE;
}

void SPI::SetClockPolarity(ClockPol_t cpol) {

	if (cpol = CPOL1) {
		this->SPI_ITEM->CR1 |= SPI_CR1_CPOL;    // Polarity clc signal CPOL = 1;
	}
	if (cpol = CPOL0) {
		this->SPI_ITEM->CR1 &= ~SPI_CR1_CPOL;   // Polarity clc signal CPOL = 0;
	}

}

void SPI::SetClockPhase(ClockPhase_t cpha) {
	if (cpha = CPHA1) {
		this->SPI_ITEM->CR1 |= SPI_CR1_CPHA;    // Phase clc signal    CPHA = 0;
	}
	if (cpha = CPHA0) {
		this->SPI_ITEM->CR1 &= ~SPI_CR1_CPHA;   // Phase clc signal    CPHA = 0;
	}

}

void SPI::SetMsbLsbFirst(MSB_LSB_First_t msb_lsb_first) {
	if (msb_lsb_first == MSB_First) {
		this->SPI_ITEM->CR1 &= ~SPI_CR1_LSBFIRST;
	}
	if (msb_lsb_first == LSB_First) {
		this->SPI_ITEM->CR1 |= SPI_CR1_LSBFIRST;
	}
}

void SPI::WriteReg(uint8_t rg, uint8_t dt) {

	this->nCS_Low();
	this->SPI_ITEM->DR = (uint16_t) ((rg << 8) | dt);
	while ((this->SPI_ITEM->SR & SPI_SR_BSY) == SPI_SR_BSY)
		;
	this->nCS_High();
}

void SPI::TransmitBlocking(uint8_t *buffer, uint16_t n) {
	if ((this->SPI_ITEM->SR & SPI_SR_TXE) == 0) {
		this->SPI_ITEM->DR = *buffer++;
	}

}
void SPI::ReceiveBlocking(uint16_t *buffer, uint16_t n) {
	while (!(this->SPI_ITEM->SR & SPI_SR_RXNE)) {
	}

	*buffer++ = this->SPI_ITEM->DR;
}

uint16_t SPI::Receive(void) {
	//SPI1->DR = 0; //запускаем обмен

	//Ждем, пока не появится новое значение
	//в буфере приемника
	while (!(SPI1->SR & SPI_SR_RXNE))
		;

	//возвращаем значение буфера приемника
	return SPI1->DR;
}

void SPI::EnableIRQ(void) {
	this->SPI_ITEM->CR2 |= SPI_CR2_RXNEIE; // Tx buffer empty interrupt enable
	//this->SPI_ITEM->CR2 |= SPI_CR2_TXEIE;  // RX buffer not empty interrupt enable
	//this->SPI_ITEM->CR2 |= SPI_CR2_ERRIE;  // Error interrupt enable
	NVIC_EnableIRQ(SPI1_IRQn);

}

uint16_t SPI::TransmitReceive16B(uint16_t TxData) {
	if ((this->SPI_ITEM->SR & SPI_SR_TXE) == 0) {
	} // Очікую спустошення передавального буфера.
	this->SPI_ITEM->DR = (uint16_t) TxData;
	if((this->SPI_ITEM->SR & SPI_SR_RXNE) != 0) {
	}// Очікую заповнення приймального буфера.
	return this->SPI_ITEM->DR;
}

uint8_t SPI::TransmitReceive8B(uint8_t TxData) {
	if ((this->SPI_ITEM->SR & SPI_SR_TXE) == 0) {
	} // Очікую спустошення передавального буфера.
	this->SPI_ITEM->DR = (uint8_t) TxData;
	if((this->SPI_ITEM->SR & SPI_SR_RXNE) != 0) {
	}// Очікую заповнення приймального буфера.
	return this->SPI_ITEM->DR;
}
