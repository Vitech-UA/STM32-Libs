#include "PZEM_004_v3.h"
#include "stdbool.h"
#include <inttypes.h>

#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(uint32_t *)(addr))

float voltage;
float current;
float power;
float energy;
float frequeny;
float pf;
uint16_t alarms;
uint8_t _addr;   // Адреса пристрою
uint64_t _lastRead; // час з моменту останнього оновлення

float PZEM004Tv30_voltage() {
	if (!PZEM004Tv30_updateValues()) // Update vales if necessary
		return NAN; // Update did not work, return NAN

	return voltage;
}

float PZEM004Tv30_current() {
	if (!PZEM004Tv30_updateValues()) // Update vales if necessary
		return NAN; // Update did not work, return NAN

	return current;
}

float PZEM004Tv30_power() {
	if (!PZEM004Tv30_updateValues()) // Update vales if necessary
		return NAN; // Update did not work, return NAN

	return power;
}

float PZEM004Tv30_energy() {
	if (!PZEM004Tv30_updateValues()) // Update vales if necessary
		return NAN; // Update did not work, return NAN

	return energy;
}

float PZEM004Tv30_frequency() {
	if (!PZEM004Tv30_updateValues()) // Update vales if necessary
		return NAN; // Update did not work, return NAN

	return frequeny;
}

float PZEM004Tv30_pf() {
	if (!PZEM004Tv30_updateValues()) // Update vales if necessary
		return NAN; // Update did not work, return NAN

	return pf;
}
static const uint16_t crcTable[] = { 0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301,
		0X03C0, 0X0280, 0XC241, 0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1,
		0XC481, 0X0440, 0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81,
		0X0E40, 0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
		0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40, 0X1E00,
		0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41, 0X1400, 0XD4C1,
		0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641, 0XD201, 0X12C0, 0X1380,
		0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040, 0XF001, 0X30C0, 0X3180, 0XF141,
		0X3300, 0XF3C1, 0XF281, 0X3240, 0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501,
		0X35C0, 0X3480, 0XF441, 0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0,
		0X3E80, 0XFE41, 0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881,
		0X3840, 0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
		0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40, 0XE401,
		0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640, 0X2200, 0XE2C1,
		0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041, 0XA001, 0X60C0, 0X6180,
		0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240, 0X6600, 0XA6C1, 0XA781, 0X6740,
		0XA501, 0X65C0, 0X6480, 0XA441, 0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01,
		0X6FC0, 0X6E80, 0XAE41, 0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1,
		0XA881, 0X6840, 0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80,
		0XBA41, 0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
		0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640, 0X7200,
		0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041, 0X5000, 0X90C1,
		0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241, 0X9601, 0X56C0, 0X5780,
		0X9741, 0X5500, 0X95C1, 0X9481, 0X5440, 0X9C01, 0X5CC0, 0X5D80, 0X9D41,
		0X5F00, 0X9FC1, 0X9E81, 0X5E40, 0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901,
		0X59C0, 0X5880, 0X9841, 0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1,
		0X8A81, 0X4A40, 0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80,
		0X8C41, 0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
		0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };

bool PZEM004Tv30_sendCmd8(uint8_t cmd, uint16_t rAddr, uint16_t val, bool check,
		uint16_t slave_addr) {
	uint8_t sendBuffer[8]; // Буфер відправки
	uint8_t respBuffer[8]; // Буфер прийнятої відповіді

	if ((slave_addr == 0xFFFF) || (slave_addr < 0x01) || (slave_addr > 0xF7)) {
		slave_addr = _addr;
	}

	sendBuffer[0] = slave_addr;              // Set slave address
	sendBuffer[1] = cmd;                     // Set command

	sendBuffer[2] = (rAddr >> 8) & 0xFF;    // Set high byte of register address
	sendBuffer[3] = (rAddr) & 0xFF;          // Set low byte =//=

	sendBuffer[4] = (val >> 8) & 0xFF;       // Set high byte of register value
	sendBuffer[5] = (val) & 0xFF;            // Set low byte =//=

	PZEM004Tv30_setCRC(sendBuffer, 8);                   // Set CRC of frame

	//_serial->write(sendBuffer, 8); // send frame
	HAL_UART_Transmit(PZEM_UART, sendBuffer, 8, HAL_MAX_DELAY);

	if (check) {
		if (!PZEM004Tv30_recieve(respBuffer, 8)) { // if check enabled, read the response
			return false;
		}

		// Check if response is same as send
		for (uint8_t i = 0; i < 8; i++) {
			if (sendBuffer[i] != respBuffer[i])
				return false;
		}
	}
	return true;

}

bool PZEM004Tv30_updateValues() {
	//static uint8_t buffer[] = {0x00, CMD_RIR, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00};
	static uint8_t response[25];

	// If we read before the update time limit, do not update
	if (_lastRead + UPDATE_TIME > HAL_GetTick()) {
		return true;
	}

	// Read 10 registers starting at 0x00 (no check)
	PZEM004Tv30_sendCmd8(CMD_RIR, 0x00, 0x0A, false, 0x01);

	if (PZEM004Tv30_recieve(response, 25) != 25) { // Something went wrong
		return false;
	}

	// Update the current values
	voltage = ((uint32_t) response[3] << 8 | // Raw voltage in 0.1V
			(uint32_t) response[4]) / 10.0;

	current = ((uint32_t) response[5] << 8
			| // Raw current in 0.001A
			(uint32_t) response[6] | (uint32_t) response[7] << 24
			| (uint32_t) response[8] << 16) / 1000.0;

	power = ((uint32_t) response[9] << 8
			| // Raw power in 0.1W
			(uint32_t) response[10] | (uint32_t) response[11] << 24
			| (uint32_t) response[12] << 16) / 10.0;

	energy = ((uint32_t) response[13] << 8
			| // Raw Energy in 1Wh
			(uint32_t) response[14] | (uint32_t) response[15] << 24
			| (uint32_t) response[16] << 16) / 1000.0;

	frequeny = ((uint32_t) response[17] << 8 | // Raw Frequency in 0.1Hz
			(uint32_t) response[18]) / 10.0;

	pf = ((uint32_t) response[19] << 8 | // Raw pf in 0.01
			(uint32_t) response[20]) / 100.0;

	alarms = ((uint32_t) response[21] << 8 | // Raw alarm value
			(uint32_t) response[22]);

	// Record current time as _lastRead
	_lastRead = HAL_GetTick();

	return true;
}

void PZEM004Tv30_setCRC(uint8_t *buf, uint16_t len) {
	if (len <= 2) // Sanity check
		return;

	uint16_t crc = PZEM004Tv30_CRC16(buf, len - 2); // CRC of data

	// Write high and low byte to last two positions
	buf[len - 2] = crc & 0xFF; // Low byte first
	buf[len - 1] = (crc >> 8) & 0xFF; // High byte second
}

void PZEM004Tv30_init(uint8_t addr) {
	if (addr < 0x01 || addr > 0xF8) // Sanity check of address
		addr = PZEM_DEFAULT_ADDR;
	_addr = addr;

	// Set initial lastRed time so that we read right away
	_lastRead = 0;
	_lastRead -= UPDATE_TIME;
}

bool PZEM004Tv30_setAddress(uint8_t addr) {
	if (addr < 0x01 || addr > 0xF7) // sanity check
		return false;

	// Write the new address to the address register
	if (!PZEM004Tv30_sendCmd8(CMD_WSR, WREG_ADDR, addr, true, addr))
		return false;

	_addr = addr; // If successful, update the current slave address

	return true;
}

bool PZEM004Tv30_checkCRC(const uint8_t *buf, uint16_t len) {
	if (len <= 2) // Sanity check
		return false;

	uint16_t crc = PZEM004Tv30_CRC16(buf, len - 2); // Compute CRC of data
	return ((uint16_t) buf[len - 2] | (uint16_t) buf[len - 1] << 8) == crc;
}

uint16_t PZEM004Tv30_CRC16(const uint8_t *data, uint16_t len) {
	uint8_t nTemp; // CRC table index
	uint16_t crc = 0xFFFF; // Default value

	while (len--) {
		nTemp = *data++ ^ crc;
		crc >>= 8;
		crc ^= (uint16_t) pgm_read_word(&crcTable[nTemp]);
	}
	return crc;
}

uint16_t PZEM004Tv30_recieve(uint8_t *resp, uint16_t len) {

	unsigned long startTime = HAL_GetTick(); // Start time for Timeout
	uint8_t index = 0; // Bytes we have read
	while ((index < len) && (HAL_GetTick() - startTime < READ_TIMEOUT)) {
		if (HAL_UART_GetState(PZEM_UART) == HAL_UART_STATE_READY) {
			uint8_t c = 0;
			HAL_UART_Receive(PZEM_UART, &c, 1, 100);

			resp[index++] = c;
		}
		// yield();	// do background netw tasks while blocked for IO (prevents ESP watchdog trigger)
	}

	// Check CRC with the number of bytes read
	if (!PZEM004Tv30_checkCRC(resp, index)) {
		return 0;
	}

	return index;
}

bool PZEM004Tv30_setPowerAlarm(uint16_t watts)
{
    if (watts > 25000){ // Sanitych check
        watts = 25000;
    }

    // Write the watts threshold to the Alarm register
    if(!PZEM004Tv30_sendCmd8(CMD_WSR, WREG_ALARM_THR, watts, true, 0x01))
        return false;

    return true;
}


bool PZEM004Tv30_getPowerAlarm()
{
	if(!PZEM004Tv30_updateValues()) // Update vales if necessary
	        return NAN; // Update did not work, return NAN

	    return alarms != 0x0000;
}
bool PZEM004Tv30_resetEnergy(){
    uint8_t buffer[] = {0x00, CMD_REST, 0x00, 0x00};
    uint8_t reply[5];
    buffer[0] = _addr;

    PZEM004Tv30_setCRC(buffer, 4);

    HAL_UART_Transmit(PZEM_UART, buffer, 4, HAL_MAX_DELAY);

    uint16_t length = recieve(reply, 5);

    if(length == 0 || length == 5){
        return false;
    }

    return true;
}
