/*
 * RFM69.h
 *
 *  Created on: 25 февр. 2021 г.
 *      Author: Embed Viktor
 */

#ifndef RFM69_H_
#define RFM69_H_

#include "SPI.h"
#include "RFM69_Registers.h"


#define REGISTER_DETAIL
#define RF69_FSTEP  61.03515625
#define RFM69_MAX_PAYLOAD 64 //

typedef enum {
	RFM69_MODE_SLEEP = 0, RFM69_MODE_STANDBY, RFM69_MODE_FS, // Синтезатор частоти ввімкнено.
	RFM69_MODE_TX,
	RFM69_MODE_RX
} RFM69Mode;

typedef enum {
	RFM69_DATA_MODE_PACKET = 0,
	RFM69_DATA_MODE_CONTINUOUS_WITH_SYNC = 2,
	RFM69_DATA_MODE_CONTINUOUS_WITHOUT_SYNC = 3,

} RFM69DataMode;


class RFM69: public SPI {

public:
	RFM69(SPI_TypeDef *spi, GPIO_TypeDef *csGPIO, uint16_t csPin,
			bool highPowerDevice, SPI_DataSize_t);
	void WriteReg(uint8_t addr, uint8_t val);
	void reset();
	bool init();
	void setFrequency(unsigned int frequency);
	void setFrequencyDerivation(unsigned int frequency);
	void setBitRate(unsigned int bitrate);
	RFM69Mode setMode(RFM69Mode mode);
	void setPowerLevel(uint8_t power);
	int setPowerDBm(int8_t dBm);
	void setHighPowerSettings(bool enable);
	void setCustomConfig(const uint8_t config[][2], unsigned int length);
	int send(const void *data, unsigned int datalength);
	int receive(char *data, unsigned int dataLength);
	void sleep();
	int getRSSI(void);
	void setOOKMode(bool enable);
	void setDataMode(RFM69DataMode dataMode = RFM69_DATA_MODE_PACKET);
	void setAutoreadRSSI(bool enable);
	void setCSMA(bool enable);
	void continuousBit(bool bit);
	void dumpRegisters();
	void setPASettings(uint8_t forcePA = 0);
	bool setAESEncryption(const void *aesKey, unsigned int keyLength);
	void chipSelect(void);
	void chipUnselect(void);
	uint8_t readRegister(uint8_t reg);
	void writeRegister(uint8_t reg, uint8_t value);
	void SetResetPin(GPIO_TypeDef *RESET_PORT, uint16_t RESET_PIN);
	uint8_t ReadTemperature(uint8_t calFactor);
	uint32_t getFrequency();
	void setAddress(uint16_t addr);
	void setNetwork(uint8_t networkID);
	void readAllRegs();
private:

	void transmit(uint8_t rx);
	void clearFIFO(void);
	void waitForModeReady();
	void waitForPacketSent();
	int readRSSI();
	bool channelFree();
	int _receive(char *data, unsigned int datalength);

	SPI_TypeDef *_spi;

	GPIO_TypeDef *_csGPIO;
	uint16_t _csPin;

	GPIO_TypeDef *_resetGPIO;
	uint16_t _resetPin;

	GPIO_TypeDef *_dataGPIO;
	uint16_t _dataPin;

	bool _init;
	RFM69Mode _mode;
	bool _highPowerDevice;
	uint8_t _powerLevel;
	int _rssi;
	uint16_t _address;
	bool _autoReadRSSI;
	bool _ookEnabled;
	RFM69DataMode _dataMode;
	bool _highPowerSettings;
	bool _csmaEnabled;
	char _rxBuffer[RFM69_MAX_PAYLOAD];
	unsigned int _rxBufferLength;

};
#endif /* RFM69_H_ */
