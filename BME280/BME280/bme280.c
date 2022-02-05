/*
 * bme280.c
 *
 *  Created on: 10 ����. 2019 �.
 *      Author: Embed Viktor
 */

#include "bme280.h"

extern I2C_HandleTypeDef hi2c1;

BME280_CalibData CalibData;
int32_t temper_int;
/**************************************************************************************************************************/
void Error() {

	LED_OFF;

}
/**************************************************************************************************************************/
uint8_t BME280_ReadStatus() {

	//clear unuset bits

	uint8_t res = BME280_ReadReg(BME280_REGISTER_STATUS) & 0x09;

	return res;
}

/**************************************************************************************************************************/

void BME280_ReadCoefficients(void)

{
	BME280_ReadReg_U16(BME280_REGISTER_DIG_T1, &CalibData.dig_T1);
	BME280_ReadReg_S16(BME280_REGISTER_DIG_T2, &CalibData.dig_T2);
	BME280_ReadReg_S16(BME280_REGISTER_DIG_T3, &CalibData.dig_T3);
	BME280_ReadReg_U16(BME280_REGISTER_DIG_P1, &CalibData.dig_P1);
	BME280_ReadReg_S16(BME280_REGISTER_DIG_P2, &CalibData.dig_P2);
	BME280_ReadReg_S16(BME280_REGISTER_DIG_P3, &CalibData.dig_P3);
	BME280_ReadReg_S16(BME280_REGISTER_DIG_P4, &CalibData.dig_P4);
	BME280_ReadReg_S16(BME280_REGISTER_DIG_P5, &CalibData.dig_P5);
	BME280_ReadReg_S16(BME280_REGISTER_DIG_P6, &CalibData.dig_P6);
	BME280_ReadReg_S16(BME280_REGISTER_DIG_P7, &CalibData.dig_P7);
	BME280_ReadReg_S16(BME280_REGISTER_DIG_P8, &CalibData.dig_P8);
	BME280_ReadReg_S16(BME280_REGISTER_DIG_P9, &CalibData.dig_P9);
	CalibData.dig_H1 = BME280_ReadReg(BME280_REGISTER_DIG_H1);
	BME280_ReadReg_S16(BME280_REGISTER_DIG_H2, &CalibData.dig_H2);
	CalibData.dig_H3 = BME280_ReadReg(BME280_REGISTER_DIG_H3);
	CalibData.dig_H4 = (BME280_ReadReg(BME280_REGISTER_DIG_H4) << 4)
			| (BME280_ReadReg(BME280_REGISTER_DIG_H4 + 1) & 0xF);
	CalibData.dig_H5 = (BME280_ReadReg(BME280_REGISTER_DIG_H5 + 1) << 4)
			| (BME280_ReadReg(BME280_REGISTER_DIG_H5) >> 4);
	CalibData.dig_H6 = (int8_t) BME280_ReadReg(BME280_REGISTER_DIG_H6);

}

/**************************************************************************************************************************/

void BME280_SetStandby(uint8_t tsb) {
	uint8_t reg;
	reg = BME280_ReadReg(BME280_REG_CONFIG) & ~BME280_STBY_MSK;
	reg |= tsb & BME280_STBY_MSK;
	BME280_WriteReg(BME280_REG_CONFIG, reg);
}

/**************************************************************************************************************************/

void BME280_Init() {

	uint8_t id = 0;

	id = BME280_ReadReg(BME280_REG_ID);
	if (id != BME280_ID) {
		Error();
		return;
	}

	else {

	}

	BME280_WriteReg(BME280_REG_SOFTRESET, BME280_SOFTRESET_VALUE);

	while (BME280_ReadStatus() & BME280_STATUS_IM_UPDATE)
		;

	BME280_ReadCoefficients();

	BME280_SetStandby(BME280_STBY_1000);

	BME280_SetFilter(BME280_FILTER_4);

	BME280_SetOversamplingTemper(BME280_OSRS_T_x4);

	BME280_SetOversamplingPressure(BME280_OSRS_P_x2);

	BME280_SetOversamplingHum(BME280_OSRS_H_x1);

	BME280_SetMode(BME280_MODE_NORMAL);
}
/**************************************************************************************************************************/
static void I2Cx_WriteData(uint16_t Addr, uint8_t Reg, uint8_t Value) {
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Write(BME280_I2C_PORT, Addr, (uint16_t) Reg,
	I2C_MEMADD_SIZE_8BIT, &Value, 1, 0x10000);
	if (status != HAL_OK)
		Error();

}
/**************************************************************************************************************************/
static uint8_t I2Cx_ReadData(uint16_t Addr, uint8_t Reg) {
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t value = 0;
	status = HAL_I2C_Mem_Read(BME280_I2C_PORT, Addr, Reg, I2C_MEMADD_SIZE_8BIT,
			&value, 1, 0x10000);
	if (status != HAL_OK)
		Error();
	return value;

}
/**************************************************************************************************************************/
static void I2Cx_ReadData16(uint16_t Addr, uint8_t Reg, uint16_t *Value) {
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Mem_Read(BME280_I2C_PORT, Addr, Reg, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) Value, 2, 0x10000);

	if (status != HAL_OK)
		Error();
}

/**************************************************************************************************************************/

static void I2Cx_ReadData24(uint16_t Addr, uint8_t Reg, uint32_t *Value) {
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Mem_Read(BME280_I2C_PORT, Addr, Reg, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) Value, 3, 0x10000);

	if (status != HAL_OK)
		Error();
}
/**************************************************************************************************************************/
void BME280_WriteReg(uint8_t Reg, uint8_t Value) {
	I2Cx_WriteData(BME280_ADDRESS, Reg, Value);
}

uint8_t BME280_ReadReg(uint8_t Reg) {
	uint8_t res = I2Cx_ReadData(BME280_ADDRESS, Reg);
	return res;
}

/**************************************************************************************************************************/

void BME280_ReadReg_U16(uint8_t Reg, uint16_t *Value) {
	I2Cx_ReadData16(BME280_ADDRESS, Reg, Value);
}
/**************************************************************************************************************************/
void BME280_ReadReg_S16(uint8_t Reg, int16_t *Value) {
	I2Cx_ReadData16(BME280_ADDRESS, Reg, (uint16_t*) Value);
}
/**************************************************************************************************************************/
void BME280_ReadReg_BE_S16(uint8_t Reg, int16_t *Value) {

	I2Cx_ReadData16(BME280_ADDRESS, Reg, (uint16_t*) Value);

	*(uint16_t*) Value = be16toword(*(uint16_t* ) Value);

}
/**************************************************************************************************************************/
void BME280_ReadReg_U24(uint8_t Reg, uint32_t *Value) {

	I2Cx_ReadData24(BME280_ADDRESS, Reg, Value);

	*(uint32_t*) Value &= 0x00FFFFFF;

}
/**************************************************************************************************************************/
void BME280_SetFilter(uint8_t filter) {
	uint8_t reg;
	reg = BME280_ReadReg(BME280_REG_CONFIG) & ~BME280_FILTER_MSK;
	reg |= filter & BME280_FILTER_MSK;
	BME280_WriteReg(BME280_REG_CONFIG, reg);
}

/**************************************************************************************************************************/
void BME280_SetOversamplingTemper(uint8_t osrs) {
	uint8_t reg;
	reg = BME280_ReadReg(BME280_REG_CTRL_MEAS) & ~BME280_OSRS_T_MSK;
	reg |= osrs & BME280_OSRS_T_MSK;
	BME280_WriteReg(BME280_REG_CTRL_MEAS, reg);
}

/**************************************************************************************************************************/

void BME280_SetOversamplingPressure(uint8_t osrs) {
	uint8_t reg;
	reg = BME280_ReadReg(BME280_REG_CTRL_MEAS) & ~BME280_OSRS_P_MSK;
	reg |= osrs & BME280_OSRS_P_MSK;
	BME280_WriteReg(BME280_REG_CTRL_MEAS, reg);
}

/**************************************************************************************************************************/
void BME280_SetOversamplingHum(uint8_t osrs) {
	uint8_t reg;
	reg = BME280_ReadReg(BME280_REG_CTRL_HUM) & ~BME280_OSRS_H_MSK;
	reg |= osrs & BME280_OSRS_H_MSK;
	BME280_WriteReg(BME280_REG_CTRL_HUM, reg);
	reg = BME280_ReadReg(BME280_REG_CTRL_MEAS);
	BME280_WriteReg(BME280_REG_CTRL_MEAS, reg);
}

/**************************************************************************************************************************/
void BME280_SetMode(uint8_t mode) {
	uint8_t reg;
	reg = BME280_ReadReg(BME280_REG_CTRL_MEAS) & ~BME280_MODE_MSK;
	reg |= mode & BME280_MODE_MSK;
	BME280_WriteReg(BME280_REG_CTRL_MEAS, reg);

}
/**************************************************************************************************************************/
float BME280_get_temperature() {
	int32_t val1, val2;
	float temper_float = 0.0f;
	uint32_t temper_raw;
	BME280_ReadReg_BE_U24(BME280_REGISTER_TEMPDATA, &temper_raw);
	temper_raw >>= 4;

	val1 = ((((temper_raw >> 3) - ((int32_t) CalibData.dig_T1 << 1)))
			* ((int32_t) CalibData.dig_T2)) >> 11;
	val2 = (((((temper_raw >> 4) - ((int32_t) CalibData.dig_T1))
			* ((temper_raw >> 4) - ((int32_t) CalibData.dig_T1))) >> 12)
			* ((int32_t) CalibData.dig_T3)) >> 14;

	temper_int = val1 + val2;

	temper_float = ((temper_int * 5 + 128) >> 8);
	temper_float /= 100.0f;
	return temper_float;
}

/**************************************************************************************************************************/

float BME280_get_pressure() {
	float press_float = 0.0f;
	uint32_t press_raw, pres_int;
	int64_t val1, val2, p;
	BME280_get_temperature(); // must be done first to get t_fine
	BME280_ReadReg_BE_U24(BME280_REGISTER_PRESSUREDATA, &press_raw);
	press_raw >>= 4;
	val1 = ((int64_t) temper_int) - 128000;
	val2 = val1 * val1 * (int64_t) CalibData.dig_P6;
	val2 = val2 + ((val1 * (int64_t) CalibData.dig_P5) << 17);
	val2 = val2 + ((int64_t) CalibData.dig_P4 << 35);
	val1 = ((val1 * val1 * (int64_t) CalibData.dig_P3) >> 8)
			+ ((val1 * (int64_t) CalibData.dig_P2) << 12);
	val1 = (((((int64_t) 1) << 47) + val1)) * ((int64_t) CalibData.dig_P1)
			>> 33;
	if (val1 == 0) {
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576 - press_raw;
	p = (((p << 31) - val2) * 3125) / val1;
	val1 = (((int64_t) CalibData.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	val2 = (((int64_t) CalibData.dig_P8) * p) >> 19;
	p = ((p + val1 + val2) >> 8) + ((int64_t) CalibData.dig_P7 << 4);
	pres_int = ((p >> 8) * 1000) + (((p & 0xff) * 390625) / 100000);
	press_float = pres_int / 100.0f;
	return press_float;

}

/**************************************************************************************************************************/

float BME280_get_humidity() {
	float hum_float = 0.0f;
	int16_t hum_raw;
	int32_t hum_raw_sign, v_x1_u32r;
	BME280_get_temperature();
	BME280_ReadReg_BE_S16(BME280_REGISTER_HUMIDDATA, &hum_raw);
	hum_raw_sign = ((int32_t) hum_raw) & 0x0000FFFF;

	v_x1_u32r = (temper_int - ((int32_t) 76800));
	v_x1_u32r = (((((hum_raw_sign << 14) - (((int32_t) CalibData.dig_H4) << 20)
			- (((int32_t) CalibData.dig_H5) * v_x1_u32r)) + ((int32_t) 16384))
			>> 15)
			* (((((((v_x1_u32r * ((int32_t) CalibData.dig_H6)) >> 10)
					* (((v_x1_u32r * ((int32_t) CalibData.dig_H3)) >> 11)
							+ ((int32_t) 32768))) >> 10) + ((int32_t) 2097152))
					* ((int32_t) CalibData.dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r
			- (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7)
					* ((int32_t) CalibData.dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
	v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
	hum_float = (v_x1_u32r >> 12);
	hum_float /= 1024.0f;
	return hum_float;
}

/**************************************************************************************************************************/

float BME280_ReadAltitude(float seaLevel) {
	float att = 0.0f;
	float atm = BME280_get_pressure();

	att = 44330.0 * (1.0 - pow(atm / seaLevel, 0.1903));
	return att;
}

/**************************************************************************************************************************/
void BME280_ReadReg_BE_U24(uint8_t Reg, uint32_t *Value) {
	I2Cx_ReadData24(BME280_ADDRESS, Reg, Value);
	*(uint32_t*) Value = be24toword(*(uint32_t *) Value) & 0x00FFFFFF;
}
