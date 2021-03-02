/*
 * RFM69.cpp
 *
 *  Created on: 25 февр. 2021 г.
 *      Author: Embed Viktor
 */

#include <RFM69.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Gpio.h"
#include "main.h"
#include "uart.h"

extern Uart Debug;

#define TIMEOUT_MODE_READY    100 ///< Maximum amount of time until mode switch [ms]
#define TIMEOUT_PACKET_SENT   100 ///< Maximum amount of time until packet must be sent [ms]
#define TIMEOUT_CSMA_READY    500 ///< Maximum CSMA wait time for channel free detection [ms]
#define CSMA_RSSI_THRESHOLD   -85 ///< If RSSI value is smaller than this, consider channel as free [dBm]

static const uint8_t rfm69_base_config[][2] =
{
{ 0x01, 0x04 }, // RegOpMode: Standby Mode
		{ 0x02, 0x00 }, // RegDataModul: Packet mode, FSK, no shaping
		{ 0x03, 0x0C }, // RegBitrateMsb: 10 kbps
		{ 0x04, 0x80 }, // RegBitrateLsb
		{ 0x05, 0x01 }, // RegFdevMsb: 20 kHz
		{ 0x06, 0x48 }, // RegFdevLsb
		{ 0x07, 0xD9 }, // RegFrfMsb: 868,15 MHz
		{ 0x08, 0x09 }, // RegFrfMid
		{ 0x09, 0x9A }, // RegFrfLsb
		{ 0x18, 0x88 }, // RegLNA: 200 Ohm impedance, gain set by AGC loop
		{ 0x19, 0x4C }, // RegRxBw: 25 kHz
		{ 0x2C, 0x00 }, // RegPreambleMsb: 3 bytes preamble
		{ 0x2D, 0x03 }, // RegPreambleLsb
		{ 0x2E, 0x88 }, // RegSyncConfig: Enable sync word, 2 bytes sync word
		{ 0x2F, 0x41 }, // RegSyncValue1: 0x4148
		{ 0x30, 0x48 }, // RegSyncValue2
		{ 0x37, 0xD0 }, // RegPacketConfig1: Variable length, CRC on, whitening
		{ 0x38, 0x40 }, // RegPayloadLength: 64 bytes max payload
		{ 0x3C, 0x8F }, // RegFifoThresh: TxStart on FifoNotEmpty, 15 bytes FifoLevel
		{ 0x58, 0x1B }, // RegTestLna: Normal sensitivity mode
		{ 0x6F, 0x30 }, // RegTestDagc: Improved margin, use if AfcLowBetaOn=0 (default)
		};

// Clock constants. DO NOT CHANGE THESE!
#define RFM69_XO               32000000    ///< Internal clock frequency [Hz]
#define RFM69_FSTEP            61.03515625 ///< Step width of synthesizer [Hz]

RFM69::RFM69(SPI_TypeDef *spi, GPIO_TypeDef *csGPIO, uint16_t csPin,
		bool highPowerDevice, SPI_DataSize_t size) :
		SPI(spi, size)
{
	_spi = spi;
	_csGPIO = csGPIO;
	_csPin = csPin;
	_resetGPIO = 0;
	_resetPin = 0;
	_init = false;
	_mode = RFM69_MODE_STANDBY;
	_highPowerDevice = highPowerDevice;
	_powerLevel = 0;
	_rssi = -127;
	_ookEnabled = false;
	_autoReadRSSI = false;
	_dataMode = RFM69_DATA_MODE_PACKET;
	_dataGPIO = 0;
	_dataPin = 0;
	_highPowerSettings = false;
	_csmaEnabled = false;
	_rxBufferLength = 0;

}

void chipSelect(void)
{

}
void chipUnselect(void)
{

}
int RFM69::getRSSI()
{
	return _rssi;
}
void RFM69::setAutoreadRSSI(bool enable)
{
	_autoReadRSSI = enable;
}
void RFM69::setCSMA(bool enable)
{
	_csmaEnabled = enable;
}

void RFM69::reset()
{
	if (_resetGPIO == 0)
		return;

	_init = false;

	// generate reset impulse
	this->_resetGPIO->BSRR |= (1 << this->_resetPin); // Set
	delay_ms(1);
	this->_resetGPIO->BSRR |= ((1 << this->_resetPin) << 16U); // Reset

	// wait until module is ready
	delay_ms(10);

	_mode = RFM69_MODE_STANDBY;
}

bool RFM69::init()
{
	// set base configuration
	setCustomConfig(rfm69_base_config, sizeof(rfm69_base_config) / 2);

	// set PA and OCP settings according to RF module (normal/high power)
	setPASettings();

	// clear FIFO and flags
	clearFIFO();

	_init = true;

	return _init;
}

void RFM69::setFrequency(unsigned int frequency)
{
	// switch to standby if TX/RX was active
	if (RFM69_MODE_RX == _mode || RFM69_MODE_TX == _mode)
		setMode(RFM69_MODE_STANDBY);

	// calculate register value
	frequency /= RFM69_FSTEP;

	// set new frequency
	writeRegister(0x07, frequency >> 16);
	writeRegister(0x08, frequency >> 8);
	writeRegister(0x09, frequency);
}

void RFM69::setFrequencyDerivation(unsigned int frequency)
{
	// switch to standby if TX/RX was active
	if (RFM69_MODE_RX == _mode || RFM69_MODE_TX == _mode)
		setMode(RFM69_MODE_STANDBY);

	// calculate register value
	frequency /= RFM69_FSTEP;

	// set new frequency
	writeRegister(0x05, frequency >> 8);
	writeRegister(0x06, frequency);
}

void RFM69::setBitRate(unsigned int bitrate)
{
	// switch to standby if TX/RX was active
	if (RFM69_MODE_RX == _mode || RFM69_MODE_TX == _mode)
		setMode(RFM69_MODE_STANDBY);

	// calculate register value
	bitrate = RFM69_XO / bitrate;

	// set new bitrate
	writeRegister(0x03, bitrate >> 8);
	writeRegister(0x04, bitrate);
}

uint8_t RFM69::readRegister(uint8_t reg)
{

	uint8_t regval = 0;

	this->nCS_Low();
	this->transfer(reg & 0x7F);
	regval = this->transfer(0);
	this->nCS_High();

	return regval;

}

void RFM69::writeRegister(uint8_t reg, uint8_t value)
{
	uint8_t i;

	this->nCS_Low();
	this->transfer(reg | 0x80);
	this->transfer(value);
	this->nCS_High();

}

void RFM69::transmit(uint8_t rx)
{

	this->nCS_Low();
	this->transfer(rx);
	this->nCS_High();

}

void RFM69::chipSelect()
{

	this->nCS_Low();
}

void RFM69::chipUnselect()
{

	this->nCS_High();
}

RFM69Mode RFM69::setMode(RFM69Mode mode)
{
	if ((mode == _mode) || (mode > RFM69_MODE_RX))
		return _mode;

	// set new mode
	writeRegister(0x01, mode << 2);

	// set special registers if this is a high power device (RFM69HW)
	if (true == _highPowerDevice)
	{
		switch (mode)
		{
		case RFM69_MODE_RX:
			// normal RX mode
			if (true == _highPowerSettings)
				setHighPowerSettings(false);
			break;

		case RFM69_MODE_TX:
			// +20dBm operation on PA_BOOST
			if (true == _highPowerSettings)
				setHighPowerSettings(true);
			break;

		default:
			break;
		}
	}

	_mode = mode;

	return _mode;
}

void RFM69::setPASettings(uint8_t forcePA)
{
	// disable OCP for high power devices, enable otherwise
	writeRegister(0x13, 0x0A | (_highPowerDevice ? 0x00 : 0x10));

	if (0 == forcePA)
	{
		if (true == _highPowerDevice)
		{
			// enable PA1 only
			writeRegister(0x11, (readRegister(0x11) & 0x1F) | 0x40);
		}
		else
		{
			// enable PA0 only
			writeRegister(0x11, (readRegister(0x11) & 0x1F) | 0x80);
		}
	}
	else
	{
		// PA settings forced
		uint8_t pa = 0;

		if (forcePA & 0x01)
			pa |= 0x80;

		if (forcePA & 0x02)
			pa |= 0x40;

		if (forcePA & 0x04)
			pa |= 0x20;

		// check if high power settings are forced
		_highPowerSettings = (forcePA & 0x08) ? true : false;
		setHighPowerSettings(_highPowerSettings);

		writeRegister(0x11, (readRegister(0x11) & 0x1F) | pa);
	}
}

void RFM69::setPowerLevel(uint8_t power)
{
	if (power > 31)
		power = 31;

	writeRegister(0x11, (readRegister(0x11) & 0xE0) | power);

	_powerLevel = power;
}

void RFM69::setHighPowerSettings(bool enable)
{
	// enabling only works if this is a high power device
	if (true == enable && false == _highPowerDevice)
		enable = false;

	writeRegister(0x5A, enable ? 0x5D : 0x55);
	writeRegister(0x5C, enable ? 0x7C : 0x70);
}

void RFM69::setCustomConfig(const uint8_t config[][2], unsigned int length)
{
	for (unsigned int i = 0; i < length; i++)
	{
		writeRegister(config[i][0], config[i][1]);
	}
}

void RFM69::clearFIFO()
{
	// clear flags and FIFO
	writeRegister(0x28, 0x10);
}

void RFM69::waitForModeReady()
{
	uint32_t timeEntry = mstimer_get();
	while (((readRegister(0x27) & 0x80) == 0)
			&& ((mstimer_get() - timeEntry) < TIMEOUT_MODE_READY))
		;
}

void RFM69::sleep()
{
	setMode(RFM69_MODE_SLEEP);
}

int RFM69::receive(char *data, unsigned int dataLength)
{
	// check if there is a packet in the internal buffer and copy it
	if (_rxBufferLength > 0)
	{
		// copy only until dataLength, even if packet in local buffer is actually larger
		memcpy(data, _rxBuffer, dataLength);

		unsigned int bytesRead = _rxBufferLength;

		// empty local buffer
		_rxBufferLength = 0;

		return bytesRead;
	}
	else
	{
		// regular receive
		return _receive(data, dataLength);
	}
}

int RFM69::_receive(char *data, unsigned int dataLength)
{
	// go to RX mode if not already in this mode
	if (RFM69_MODE_RX != _mode)
	{
		setMode(RFM69_MODE_RX);
		waitForModeReady();
	}

	// check for flag PayloadReady
	if (readRegister(0x28) & 0x04)
	{
		// go to standby before reading data
		setMode(RFM69_MODE_STANDBY);

		// get FIFO content
		unsigned int bytesRead = 0;

		// read until FIFO is empty or buffer length exceeded
		while ((readRegister(0x28) & 0x40) && (bytesRead < dataLength))
		{
			// read next byte
			data[bytesRead] = readRegister(0x00);
			bytesRead++;
		}

		// automatically read RSSI if requested
		if (true == _autoReadRSSI)
		{
			readRSSI();
		}

		// go back to RX mode
		setMode(RFM69_MODE_RX);
		// todo: wait needed?
		//		waitForModeReady();

		// todo: Видалити
		int i;
		i = bytesRead;

		return bytesRead;
	}

	else

		return 0;
}

bool RFM69::setAESEncryption(const void *aesKey, unsigned int keyLength)
{
	bool enable = false;

	// check if encryption shall be enabled or disabled
	if ((0 != aesKey) && (16 == keyLength))
		enable = true;

	// switch to standby
	setMode(RFM69_MODE_STANDBY);

	if (true == enable)
	{
		// transfer AES key to AES key register

		this->nCS_Low();
		// address first AES MSB register
		this->transmit(0x3E | 0x80);
		// TODO  _spi->transfer(0x3E | 0x80);
		//this->TransmitReceive8B(0x3E | 0x80);

		// transfer key (0x3E..0x4D)
		for (unsigned int i = 0; i < keyLength; i++)
			// TODO   _spi->transfer(((uint8_t*)aesKey)[i]);
			this->transmit(((uint8_t*) aesKey)[i]);
		this->nCS_Low();

	}

	// set/reset AesOn Bit in packet config
	writeRegister(0x3D, (readRegister(0x3D) & 0xFE) | (enable ? 1 : 0));

	return enable;
}

void RFM69::waitForPacketSent()
{
	uint32_t timeEntry = mstimer_get();
	while (((readRegister(0x28) & 0x08) == 0)
			&& ((mstimer_get() - timeEntry) < TIMEOUT_PACKET_SENT))
		;
}

void RFM69::continuousBit(bool bit)
{
	// only allow this in continuous mode and if data pin was specified
	if ((RFM69_DATA_MODE_PACKET == _dataMode) || (0 == _dataGPIO))
		return;

	// send low or high bit
	if (false == bit)
	{
	}
	// TODO GPIO_ResetBits(_dataGPIO, _dataPin);
	else
	{
	}
	// TODO GPIO_SetBits(_dataGPIO, _dataPin);
}

int RFM69::readRSSI()
{
	_rssi = -readRegister(0x24) / 2;

	return _rssi;
}

void RFM69::dumpRegisters(void)
{

	uint8_t RxBuffer[100] =
	{ };
	uint8_t RxByte;
	for (int i = 1; i <= 113; i++)
	{
		RxBuffer[i] = readRegister(i);
		Debug.SendByte(RxBuffer[i]);
		Debug.SendByte('\n');
	}
}

void RFM69::setOOKMode(bool enable)
{
	// switch to standby if TX/RX was active
	if (RFM69_MODE_RX == _mode || RFM69_MODE_TX == _mode)
		setMode(RFM69_MODE_STANDBY);

	if (false == enable)
	{
		// FSK
		writeRegister(0x02, (readRegister(0x02) & 0xE7));
	}
	else
	{
		// OOK
		writeRegister(0x02, (readRegister(0x02) & 0xE7) | 0x08);
	}

	_ookEnabled = enable;
}

void RFM69::setDataMode(RFM69DataMode dataMode)
{
	// switch to standby if TX/RX was active
	if (RFM69_MODE_RX == _mode || RFM69_MODE_TX == _mode)
		setMode(RFM69_MODE_STANDBY);

	switch (dataMode)
	{
	case RFM69_DATA_MODE_PACKET:
		writeRegister(0x02, (readRegister(0x02) & 0x1F));
		break;

	case RFM69_DATA_MODE_CONTINUOUS_WITH_SYNC:
		writeRegister(0x02, (readRegister(0x02) & 0x1F) | 0x40);
		writeRegister(0x25, 0x04); // Dio2Mapping = 01 (Data)
		continuousBit(false);
		break;

	case RFM69_DATA_MODE_CONTINUOUS_WITHOUT_SYNC:
		writeRegister(0x02, (readRegister(0x02) & 0x1F) | 0x60);
		writeRegister(0x25, 0x04); // Dio2Mapping = 01 (Data)
		continuousBit(false);
		break;

	default:
		return;
	}

	_dataMode = dataMode;
}

int RFM69::setPowerDBm(int8_t dBm)
{
	/* Output power of module is from -18 dBm to +13 dBm
	 * in "low" power devices, -2 dBm to +20 dBm in high power devices */
	if (dBm < -18 || dBm > 20)
		return -1;

	if (false == _highPowerDevice && dBm > 13)
		return -1;

	if (true == _highPowerDevice && dBm < -2)
		return -1;

	uint8_t powerLevel = 0;

	if (false == _highPowerDevice)
	{
		// only PA0 can be used
		powerLevel = dBm + 18;

		// enable PA0 only
		writeRegister(0x11, 0x80 | powerLevel);
	}
	else
	{
		if (dBm >= -2 && dBm <= 13)
		{
			// use PA1 on pin PA_BOOST
			powerLevel = dBm + 18;

			// enable PA1 only
			writeRegister(0x11, 0x40 | powerLevel);

			// disable high power settings
			_highPowerSettings = false;
			setHighPowerSettings(_highPowerSettings);
		}
		else if (dBm > 13 && dBm <= 17)
		{
			// use PA1 and PA2 combined on pin PA_BOOST
			powerLevel = dBm + 14;

			// enable PA1+PA2
			writeRegister(0x11, 0x60 | powerLevel);

			// disable high power settings
			_highPowerSettings = false;
			setHighPowerSettings(_highPowerSettings);
		}
		else
		{
			// output power from 18 dBm to 20 dBm, use PA1+PA2 with high power settings
			powerLevel = dBm + 11;

			// enable PA1+PA2
			writeRegister(0x11, 0x60 | powerLevel);

			// enable high power settings
			_highPowerSettings = true;
			setHighPowerSettings(_highPowerSettings);
		}
	}

	return 0;
}

bool RFM69::channelFree()
{
	if (readRSSI() < CSMA_RSSI_THRESHOLD)
	{
		return true;
	}
	else
	{
		return false;
	}
}

int RFM69::send(const void *data, unsigned int dataLength)
{
// switch to standby and wait for mode ready, if not in sleep mode
	if (RFM69_MODE_SLEEP != _mode)
	{
		setMode(RFM69_MODE_STANDBY);
		waitForModeReady();
	}

	// clear FIFO to remove old data and clear flags
	clearFIFO();

	// limit max payload
	if (dataLength > RFM69_MAX_PAYLOAD)
		dataLength = RFM69_MAX_PAYLOAD;

	// payload must be available
	if (0 == dataLength)
		return 0;

	/* Wait for a free channel, if CSMA/CA algorithm is enabled.
	 * This takes around 1,4 ms to finish if channel is free */
	if (true == _csmaEnabled)
	{
		// Restart RX
		writeRegister(0x3D, (readRegister(0x3D) & 0xFB) | 0x20);

		// switch to RX mode
		setMode(RFM69_MODE_RX);

		// wait until RSSI sampling is done; otherwise, 0xFF (-127 dBm) is read

		// RSSI sampling phase takes ~960 µs after switch from standby to RX
		uint32_t timeEntry = mstimer_get();
		while (((readRegister(0x23) & 0x02) == 0)
				&& ((mstimer_get() - timeEntry) < 10))
			;

		while ((false == channelFree())
				&& ((mstimer_get() - timeEntry) < TIMEOUT_CSMA_READY))
		{
			// wait for a random time before checking again
			delay_ms(rand() % 10);

			/* try to receive packets while waiting for a free channel
			 * and put them into a temporary buffer */
			int bytesRead;
			if ((bytesRead = _receive(_rxBuffer, RFM69_MAX_PAYLOAD)) > 0)
			{
				_rxBufferLength = bytesRead;

				// module is in RX mode again

				// Restart RX and wait until RSSI sampling is done
				writeRegister(0x3D, (readRegister(0x3D) & 0xFB) | 0x20);
				uint32_t timeEntry = mstimer_get();
				while (((readRegister(0x23) & 0x02) == 0)
						&& ((mstimer_get() - timeEntry) < 10))
					;
			}
		}

		setMode(RFM69_MODE_STANDBY);
	}

	// transfer packet to FIFO
	chipSelect();

	// address FIFO
	this->transfer(0x00 | 0x80);

	// send length byte
	this->transfer(dataLength);

	// send payload
	for (unsigned int i = 0; i < dataLength; i++)
		this->transfer(((uint8_t*) data)[i]);

	chipUnselect();

	// start radio transmission
	setMode(RFM69_MODE_TX);

	// wait for packet sent
	waitForPacketSent();

	// go to standby
	setMode(RFM69_MODE_STANDBY);

	return dataLength;
}

void RFM69::SetResetPin(GPIO_TypeDef *RESET_PORT, uint16_t RESET_PIN)
{
	this->_resetGPIO = RESET_PORT;
	this->_resetPin = RESET_PIN;
	Gpio ResetPin = Gpio(this->_resetGPIO, this->_resetPin);
	ResetPin.SetAsGenerapPurporseOutput(OUTPUT_PP);

}

uint8_t RFM69::ReadTemperature(uint8_t calFactor)
{

	this->setMode(RFM69_MODE_STANDBY);
	this->writeRegister(REG_TEMP1, RF_TEMP1_MEAS_START);
	while ((this->readRegister(REG_TEMP1) & RF_TEMP1_MEAS_RUNNING))
		;
	return ~this->readRegister(REG_TEMP2) + calFactor; // 'complement' corrects the slope, rising temp = rising val

}

uint32_t RFM69::getFrequency()
{
  return RF69_FSTEP * (((uint32_t) this->readRegister(REG_FRFMSB) << 16) + ((uint16_t) this->readRegister(REG_FRFMID) << 8) + this->readRegister(REG_FRFLSB));
}

void RFM69::setAddress(uint16_t addr)
{
  _address = addr;
  this->writeRegister(REG_NODEADRS, _address); //unused in packet mode
}

//set this node's network id
void RFM69::setNetwork(uint8_t networkID)
{
	this->writeRegister(REG_SYNCVALUE2, networkID);
}
