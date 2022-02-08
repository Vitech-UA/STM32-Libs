/*
 * rfm69.c
 *
 *  Created on: Aug 2, 2021
 *      Author: Vitech-UA
 */

#include "rfm69.h"
#include "string.h"

extern SPI_HandleTypeDef hspi1;

#define rfm_spi hspi1
#define RFM69_XO               32000000    ///< Internal clock frequency [Hz]
#define RFM69_FSTEP            61.03515625 ///< Step width of synthesizer [Hz]
#define CSMA_LIMIT              -90 // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_MODE_SLEEP         0 // XTAL OFF
#define RF69_MODE_STANDBY       1 // XTAL ON
#define RF69_MODE_SYNTH         2 // PLL ON
#define RF69_MODE_RX            3 // RX MODE
#define RF69_MODE_TX            4 // TX MODE
#define COURSE_TEMP_COEF    -90 // puts the temperature reading in the ballpark, user can fine tune the returned value

#define RF69_TX_LIMIT_MS   1000
// TWS: define CTLbyte bits
#define RFM69_CTL_SENDACK   0x80
#define RFM69_CTL_REQACK    0x40
#define RF69_CSMA_LIMIT_MS 1000
#define RF69_BROADCAST_ADDR 255

volatile uint8_t _mode;        // current transceiver state
volatile bool _inISR;
volatile uint8_t PAYLOADLEN;

volatile uint8_t DATALEN;
volatile uint8_t SENDERID;
volatile uint8_t TARGETID;
volatile uint8_t PAYLOADLEN;
volatile uint8_t ACK_REQUESTED;
volatile uint8_t ACK_RECEIVED;
volatile uint8_t ctlByte;
volatile int16_t RSSI; // most accurate RSSI during reception (closest to the reception)
volatile bool _inISR;
volatile bool _haveData;

uint8_t _powerLevel;
uint8_t _address;
uint8_t _interruptPin;
uint8_t _interruptNum;

bool _isRFM69HW = false;
bool _promiscuousMode;
uint8_t frame[256];

void rfm69_select(void) {

	RFM69_SELECT_GPIO->BSRR |= RFM69_SELECT_PIN << 16U; // RESET

}
void rfm69_release(void) {

	RFM69_SELECT_GPIO->BSRR |= RFM69_SELECT_PIN; // SET
}
void rfm69_up_reset_pin(void) {
	RFM69_RESET_GPIO->BSRR |= RFM69_RESET_PIN; //SET
}
void rfm69_down_reset_pin(void) {
	RFM69_RESET_GPIO->BSRR |= RFM69_RESET_PIN << 16U; //SET
}
uint8_t read_reg(uint8_t reg) {

	uint8_t regval = 0;
	uint8_t zero_byte = 0;
	uint8_t read_data = reg & 0x7F;

	rfm69_select();
	HAL_SPI_Transmit(&rfm_spi, &read_data, 1, 100);
	HAL_SPI_TransmitReceive(&rfm_spi, (uint8_t*) &zero_byte, (uint8_t*) &regval,
			1, 100);
	rfm69_release();

	return regval;

}
void writeReg(uint8_t reg, uint8_t value) {
	rfm69_select();
	uint8_t write_data = reg | 0x80;
	HAL_SPI_Transmit(&rfm_spi, (uint8_t*) &write_data, 1, 100);
	HAL_SPI_Transmit(&rfm_spi, (uint8_t*) &value, 1, 100);
	rfm69_release();

}
bool rfm69_init(uint8_t freqBand, uint8_t nodeID, uint8_t networkID) {
	const uint8_t CONFIG[][2] =
			{
			/* 0x01 */{ REG_OPMODE, RF_OPMODE_SEQUENCER_ON
					| RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
					/* 0x02 */{ REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET
							| RF_DATAMODUL_MODULATIONTYPE_FSK
							| RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
					/* 0x03 */{ REG_BITRATEMSB, RF_BITRATEMSB_1200 }, // default: 4.8 KBPS
					/* 0x04 */{ REG_BITRATELSB, RF_BITRATELSB_1200 },
					/* 0x05 */{ REG_FDEVMSB, RF_FDEVMSB_50000 }, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
					/* 0x06 */{ REG_FDEVLSB, RF_FDEVLSB_50000 },

					/* 0x07 */{ REG_FRFMSB, (uint8_t) (
							freqBand == RF69_315MHZ ?
									RF_FRFMSB_315 :
									(freqBand == RF69_433MHZ ?
											RF_FRFMSB_433 :
											(freqBand == RF69_868MHZ ?
													RF_FRFMSB_868 :
													RF_FRFMSB_915))) },
					/* 0x08 */{ REG_FRFMID, (uint8_t) (
							freqBand == RF69_315MHZ ?
									RF_FRFMID_315 :
									(freqBand == RF69_433MHZ ?
											RF_FRFMID_433 :
											(freqBand == RF69_868MHZ ?
													RF_FRFMID_868 :
													RF_FRFMID_915))) },
					/* 0x09 */{ REG_FRFLSB, (uint8_t) (
							freqBand == RF69_315MHZ ?
									RF_FRFLSB_315 :
									(freqBand == RF69_433MHZ ?
											RF_FRFLSB_433 :
											(freqBand == RF69_868MHZ ?
													RF_FRFLSB_868 :
													RF_FRFLSB_915))) },

					// looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
					// +17dBm and +20dBm are possible on RFM69HW
					// +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
					// +17dBm formula: Pout = -14 + OutputPower (with PA1 and PA2)**
					// +20dBm formula: Pout = -11 + OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
					{ REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF| RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111 },
					/* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)

					// RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4KHz)
					/* 0x19 */{ REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16
							| RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw)
					//for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
					/* 0x25 */{ REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using
					/* 0x26 */{ REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
					/* 0x28 */{ REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
					/* 0x29 */{ REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
					///* 0x2D */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE } // default 3 preamble bytes 0xAAAAAA
					/* 0x2E */{ REG_SYNCCONFIG, RF_SYNC_ON
							| RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2
							| RF_SYNC_TOL_0 },
					/* 0x2F */{ REG_SYNCVALUE1, 0x2D }, // attempt to make this compatible with sync1 byte of RFM12B lib
					/* 0x30 */{ REG_SYNCVALUE2, networkID }, // NETWORK ID
					/* 0x37 */{ REG_PACKETCONFIG1,
					RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF
							| RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON
							| RF_PACKET1_ADRSFILTERING_OFF },
					/* 0x38 */{ REG_PAYLOADLENGTH, RF69_MAX_DATA_LEN + 5 }, // in variable length mode: the max frame size, not used in TX
					///* 0x39 */ { REG_NODEADRS, nodeID }, // turned off because we're not using address filtering
					/* 0x3C */{ REG_FIFOTHRESH,
					RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty
					/* 0x3D */{ REG_PACKETCONFIG2,
					RF_PACKET2_RXRESTARTDELAY_2BITS
							| RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
					//for BR-19200: /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
					/* 0x6F */{ REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
					{ 255, 0 } };

	uint32_t start = HAL_GetTick();
	uint32_t timeout = 50;
	do {
		writeReg(REG_SYNCVALUE1, 0xAA);
	} while (read_reg(REG_SYNCVALUE1) != 0xaa && HAL_GetTick() - start < timeout);
	start = HAL_GetTick();
	do {
		writeReg(REG_SYNCVALUE1, 0x55);
	} while (read_reg(REG_SYNCVALUE1) != 0x55 && HAL_GetTick() - start < timeout);

	for (uint8_t i = 0; CONFIG[i][0] != 255; i++) {
		writeReg(CONFIG[i][0], CONFIG[i][1]);
	}

	setMode(RF69_MODE_STANDBY, false);
	start = HAL_GetTick();
	while (((read_reg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00)
			&& HAL_GetTick() - start < timeout)
		; // wait for ModeReady
	if (HAL_GetTick() - start >= timeout) {
		return false;
	}
    setHighPowerRegs(true);
    setPowerLevel(13);
	setAddress(nodeID);
	return true;
}
uint32_t getFrequency(void) {
	return RFM69_FSTEP
			* (((uint32_t) read_reg(REG_FRFMSB) << 16)
					+ ((uint16_t) read_reg(REG_FRFMID) << 8)
					+ read_reg(REG_FRFLSB));
}
void setFrequency(uint32_t freqHz) {
	uint8_t oldMode = _mode;
	if (oldMode == RF69_MODE_TX) {
		setMode(RF69_MODE_RX, false);
	}
	freqHz /= RFM69_FSTEP; // divide down by FSTEP to get FRF
	writeReg(REG_FRFMSB, freqHz >> 16);
	writeReg(REG_FRFMID, freqHz >> 8);
	writeReg(REG_FRFLSB, freqHz);
	if (oldMode == RF69_MODE_RX) {
		setMode(RF69_MODE_SYNTH, false);
	}
	setMode(oldMode, false);
}
void setMode(uint8_t newMode, bool waitForReady) {
	if (newMode == _mode)
		return;

	switch (newMode) {
	case RF69_MODE_TX:
		writeReg(REG_OPMODE,
				(read_reg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
		if (_isRFM69HW)
			setHighPowerRegs(true);
		break;
	case RF69_MODE_RX:
		writeReg(REG_OPMODE, (read_reg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
		if (_isRFM69HW)
			setHighPowerRegs(false);
		break;
	case RF69_MODE_SYNTH:
		writeReg(REG_OPMODE,
				(read_reg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
		break;
	case RF69_MODE_STANDBY:
		writeReg(REG_OPMODE, (read_reg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
		break;
	case RF69_MODE_SLEEP:
		writeReg(REG_OPMODE, (read_reg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
		break;
	default:
		return;
	}

	if (waitForReady) {
		while ((read_reg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00)
			; // wait for ModeReady
	}

	_mode = newMode;
}
void setAddress(uint8_t addr) {
	_address = addr;
	writeReg(REG_NODEADRS, _address);
}
void setNetwork(uint8_t networkID) {
	writeReg(REG_SYNCVALUE2, networkID);
}
void setPowerLevel(uint8_t powerLevel) {
	_powerLevel = (powerLevel > 31 ? 31 : _powerLevel);
	if (_isRFM69HW)
		_powerLevel /= 2;
	writeReg(REG_PALEVEL, (read_reg(REG_PALEVEL) & 0xE0) | _powerLevel);
}
uint32_t send(uint8_t toAddress, uint8_t *buffer, uint16_t bufferSize,
bool requestACK, bool sendACK) {
	setMode(RF69_MODE_STANDBY, /*waitForReady=*/true); // turn off receiver to prevent reception while filling fifo
	writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"

	// control byte
	if (bufferSize > RF69_MAX_DATA_LEN) {
		bufferSize = RF69_MAX_DATA_LEN;
	}

	uint8_t CTLbyte = 0x00;
	if (sendACK) {
		CTLbyte = RFM69_CTL_SENDACK;
	} else if (requestACK) {
		CTLbyte = RFM69_CTL_REQACK;
	}

	uint8_t cmd[6] = { (uint8_t) (REG_FIFO | 0x80), (uint8_t) (bufferSize + 3),
			toAddress, _address, CTLbyte, 0 };

	// write to FIFO
	rfm69_select();
	HAL_SPI_Transmit(&RFM69_SPI_PORT, (uint8_t*) &cmd, 5, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&RFM69_SPI_PORT, buffer, bufferSize, HAL_MAX_DELAY);
	rfm69_release();

	// no need to wait for transmit mode to be ready since its handled by the radio
	setMode(RF69_MODE_TX, true);

	uint32_t txStart = HAL_GetTick();
	while (HAL_GPIO_ReadPin(DIO0_GPIO_Port, DIO0_Pin) == GPIO_PIN_RESET
			&& HAL_GetTick() - txStart < RF69_TX_LIMIT_MS)
		; // Очікую підняття DIO0 що сигналізуватиме про завершення передачі
	setMode(RF69_MODE_STANDBY, /*waitForReady=*/true);
	receiveBegin();

	return (HAL_GetTick() - txStart);
}

bool readData(Payload *data) {
	if (_mode == RF69_MODE_RX
			&& (read_reg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)) {

		data->targetId = data->senderId = data->ctlByte = 0xFF;

		data->signalStrength = readRSSI(false);
		memset(data->data, 0, RF69_MAX_DATA_LEN);

		// Читаю кадр
		setMode(RF69_MODE_STANDBY, /*waitForReady=*/true);
		uint8_t zero_byte = 0;
		uint8_t read_data = REG_FIFO & 0x7F;

		rfm69_select();
		HAL_SPI_Transmit(&rfm_spi, &read_data, 1, 100);
		HAL_SPI_TransmitReceive(&rfm_spi, (uint8_t*) &zero_byte,
				(uint8_t*) &data->size, 1, 100);

		HAL_StatusTypeDef errorCode = HAL_SPI_Receive(&RFM69_SPI_PORT, *&frame,
				data->size, HAL_MAX_DELAY);

		rfm69_release();
		setMode(RF69_MODE_RX, true);

		//Парсим кадр

		if (errorCode == HAL_OK) {
			data->targetId = frame[0];
			data->senderId = frame[1];
			data->ctlByte = frame[2];
			for (int8_t i = 3; i < data->size; i++) {
				data->data[i - 3] = frame[i];
			}
			writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // set DIO0 to "PAYLOADREADY" in receive mode
			setMode(RF69_MODE_RX, false);
			return true;
		}
	}
	return false;
}
bool waitForResponce(Payload *data, uint32_t timeout) {
	uint32_t start = HAL_GetTick();
	while (timeout == __UINT32_MAX__ || HAL_GetTick() - start < timeout) {
		if (HAL_GPIO_ReadPin(DIO0_GPIO_Port, DIO0_Pin) == GPIO_PIN_RESET) {
			continue;
		}

		if (readData(data)) {
			return true;
		}
	}
	return false;
}
void receiveBegin() {
	if (read_reg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY) {
		writeReg(REG_PACKETCONFIG2,
				(read_reg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
	}
	writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // set DIO0 to "PAYLOADREADY" in receive mode
	setMode(RF69_MODE_RX, false);
}
int16_t readRSSI(bool forceTrigger) {
	int16_t rssi = 0;
	if (forceTrigger) {
		// RSSI trigger not needed if DAGC is in continuous mode
		writeReg(REG_RSSICONFIG, RF_RSSI_START);
		while ((read_reg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00)
			; // wait for RSSI_Ready
	}
	rssi = -read_reg(REG_RSSIVALUE);
	rssi >>= 1;
	return rssi;
}
void setHighPower(bool onOff) {
	_isRFM69HW = onOff;
	writeReg(REG_OCP, _isRFM69HW ? RF_OCP_OFF : RF_OCP_ON);
	if (_isRFM69HW) // turning ON
	{
		writeReg(REG_PALEVEL,
				(read_reg(REG_PALEVEL) & 0x1F) | RF_PALEVEL_PA1_ON
						| RF_PALEVEL_PA2_ON); // enable P1 & P2 amplifier stages
	} else {
		writeReg(REG_PALEVEL,
				RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF
						| _powerLevel); // enable P0 only
	}
}

bool setAESEncryption(const void *aesKey, unsigned int keyLength) {
	bool enable = false;

	if ((0 != aesKey) && (16 == keyLength))
		enable = true;

	setMode(RF69_MODE_STANDBY, false);

	uint8_t set_AES_cmd[] = { 0x3E | 0x80 };
	if (true == enable) {

		rfm69_select();
		HAL_SPI_Transmit(&RFM69_SPI_PORT, (uint8_t*) &set_AES_cmd[0], 1, 100);

		for (unsigned int i = 0; i < keyLength; i++) {
			HAL_SPI_Transmit(&RFM69_SPI_PORT, &aesKey[i], 1, 100);
		}

		rfm69_release();
	}

	writeReg(0x3D, (read_reg(0x3D) & 0xFE) | (enable ? 1 : 0));

	return enable;
}

void setHighPowerRegs(bool onOff) {
	writeReg(REG_TESTPA1, onOff ? 0x5D : 0x55);
	writeReg(REG_TESTPA2, onOff ? 0x7C : 0x70);
}
uint8_t readTemperature(uint8_t calFactor) // returns centigrade
{
	setMode(RF69_MODE_STANDBY, /*waitForReady=*/true);
	writeReg(REG_TEMP1, RF_TEMP1_MEAS_START);
	while ((read_reg(REG_TEMP1) & RF_TEMP1_MEAS_RUNNING))
		;
	return ~read_reg(REG_TEMP2) + COURSE_TEMP_COEF + calFactor; // 'complement' corrects the slope, rising temp = rising val
}

void rcCalibration() {
	writeReg(REG_OSC1, RF_OSC1_RCCAL_START);
	while ((read_reg(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00)
		;
}
