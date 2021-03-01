/*
 * SPI.h
 *
 *  Created on: 14 февр. 2021 г.
 *      Author: Embed Viktor
 */

#ifndef SPI_H_
#define SPI_H_

#include "Gpio.h"

#define STM32_SERIES 0

#if(STM32_SERIES == 0)
/*SPI1 ремапи*/
#define SPI1_MOSI
#define SPI1_MOSI

#define SPI1_MISO
#define SPI1_MISO

#define SPI1_SCK
#define SPI1_SCK

/*SPI2 ремапи*/
#define SPI2_MOSI
#define SPI2_MOSI

#define SPI2_MISO
#define SPI2_MISO

#define SPI2_SCK
#define SPI2_SCK

#endif
typedef enum SPI_mode {
	Full_Duplex_Master = 0,
	Full_Duplex_Slave,
	Half_Duplex_Master,
	Half_Duplex_Slave,
	Receive_Only_Master,
	Receive_Only_Slave,
	Transmit_Only_Master,
	Transmit_Only_Slave
} SPI_mode_t;

typedef enum SPI_DataSize {
	DataSize_8B = 0, DataSize_16B
} SPI_DataSize_t;

typedef enum MSB_LSB_First {
	LSB_First = 0, MSB_First
} MSB_LSB_First_t;

typedef enum SetClockPrsc {

	fPCLK_DIV_By_2 = 0,
	fPCLK_DIV_By_4,
	fPCLK_DIV_By_8,
	fPCLK_DIV_By_16,
	fPCLK_DIV_By_32,
	fPCLK_DIV_By_64,
	fPCLK_DIV_By_128,
	fPCLK_DIV_By_256

} SetClockPrsc_t;

typedef enum ClockPol {
	CPOL1 = 0, CPOL0
} ClockPol_t;

typedef enum ClockPhase {
	CPHA1 = 0, CPHA0
} ClockPhase_t;

class SPI {
public:
	SPI(SPI_TypeDef *Port, SPI_DataSize_t size);
	void nCS_Low();
	void nCS_High();
	void ReceiveBlocking(uint16_t *buffer, uint16_t n);
	uint16_t Receive(void);
	void EnableIRQ(void);
	void TransmitBlocking(uint8_t buffer);
private:

	void Config();
	void InitGpio(void);
	void EnableClk(void);
	void SetFrameSize(SPI_DataSize_t);
	void SetClockPrsc(SetClockPrsc_t);
	void EnableSoftwareSlaveManagment(void);
	void DisableSoftwareSlaveManagment(void);
	void EnableMotorollaMode(void);
	void Enable(void);
	void Disable(void);
	void SetClockPolarity(ClockPol_t);
	void SetClockPhase(ClockPhase_t);
	void SetMsbLsbFirst(MSB_LSB_First_t);

protected:
	uint16_t TransmitReceive16B(uint16_t TxData);
	uint8_t TransmitReceive8B(uint8_t TxData);
	void WriteReg(uint8_t rg, uint8_t dt);
	SPI_TypeDef *SPI_ITEM;
	GPIO_TypeDef *MOSI_PORT;
	uint16_t MOSI_PIN;
	GPIO_TypeDef *MISO_PORT;
	uint16_t MISO_PIN;
	GPIO_TypeDef *SCK_PORT;
	uint16_t SCK_PIN;
	GPIO_TypeDef *nSC_PORT;
	uint16_t nSC_PIN;
	SPI_DataSize_t _dataSize;
};

#endif /* SPI_H_ */
