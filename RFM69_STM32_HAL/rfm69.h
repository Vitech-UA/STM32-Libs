/*
 * rfm69.h
 *
 *  Created on: Aug 2, 2021
 *      Author: Vitech-UA
 */

#ifndef INC_RFM69_H_
#define INC_RFM69_H_

#include "main.h"
#include "rfm69_registers.h"
#include "stdbool.h"

#define SPI1_DR_8bit          (*(__IO uint8_t *)((uint32_t)&(SPI1->DR)))
#define SPI2_DR_8bit          (*(__IO uint8_t *)((uint32_t)&(SPI2->DR)))

#define RFM69_SELECT_GPIO RFM_NSEL_GPIO_Port
#define RFM69_SELECT_PIN RFM_NSEL_Pin

#define RFM69_RESET_GPIO RFM_RESET_GPIO_Port
#define RFM69_RESET_PIN RFM_RESET_Pin

#define RFM69_SPI_PORT hspi1

// available frequency bands
#define RF69_315MHZ            31 // non trivial values to avoid misconfiguration
#define RF69_433MHZ            43
#define RF69_868MHZ            86
#define RF69_915MHZ            91

#define RF69_MAX_DATA_LEN       61

typedef struct  {
	uint8_t size;
	uint8_t targetId, senderId, ctlByte;
	uint8_t data[RF69_MAX_DATA_LEN];
	int16_t signalStrength;
}Payload;


void rfm69_select(void);
void rfm69_release(void);
void rfm69_up_reset_pin(void);
void rfm69_down_reset_pin(void);
uint8_t spi_transfer(uint8_t data);
uint8_t read_reg(uint8_t reg);
void writeReg(uint8_t reg, uint8_t value);
bool rfm69_init(uint8_t freqBand, uint8_t nodeID, uint8_t networkID);
void setMode(uint8_t newMode, bool waitForReady);
void setAddress(uint8_t addr);
uint32_t getFrequency(void);
uint8_t readTemperature (uint8_t calFactor);
void setHighPowerRegs(bool onOff);
void setPowerLevel(uint8_t powerLevel);
int16_t readRSSI(bool forceTrigger);
bool read_data(Payload *data);
bool waitForResponce(Payload *data , uint32_t timeout);
bool setAESEncryption(const void* aesKey, unsigned int keyLength);
void receiveBegin();
void setFrequency(uint32_t freqHz);
uint32_t send(uint8_t toAddress, uint8_t *buffer, uint16_t bufferSize,bool requestACK, bool sendACK);
#endif /* INC_RFM69_H_ */
