/*
 * MT48LC4M32B2.c
 *
 *  Created on: May 20, 2020
 *      Author: viktor.starovit
 */

#include "MT48LC4M32B2.h"

FMC_SDRAM_CommandTypeDef command;
HAL_StatusTypeDef hal_status;

void MT48LC4M32B2_Init(SDRAM_HandleTypeDef *sdram) {
	__IO uint32_t tmrmd = 0;

	command.CommandMode = FMC_SDRAM_CMD_CLK_ENABLE;
	command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
	command.AutoRefreshNumber = 1;
	command.ModeRegisterDefinition = 0;
	hal_status = HAL_SDRAM_SendCommand(sdram, &command, SDRAM_TIMEOUT);
	HAL_Delay(1);

	command.CommandMode = FMC_SDRAM_CMD_PALL;
	command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
	command.AutoRefreshNumber = 1;
	command.ModeRegisterDefinition = 0;
	hal_status = HAL_SDRAM_SendCommand(sdram, &command, SDRAM_TIMEOUT);

	command.CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
	command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
	command.AutoRefreshNumber = 8;
	command.ModeRegisterDefinition = 0;
	hal_status = HAL_SDRAM_SendCommand(sdram, &command, SDRAM_TIMEOUT);

	tmrmd = (uint32_t) SDRAM_MODEREG_BURST_LENGTH_1 |
	SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL |
	SDRAM_MODEREG_CAS_LATENCY_2 |
	SDRAM_MODEREG_OPERATING_MODE_STANDARD |
	SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;
	command.CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
	command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
	command.AutoRefreshNumber = 1;
	command.ModeRegisterDefinition = tmrmd;
	hal_status = HAL_SDRAM_SendCommand(sdram, &command, SDRAM_TIMEOUT);

	sdram->Instance->SDRTR |= ((uint32_t) ((1292) << 1));

	HAL_Delay(1);

}

void MT48LC4M32B2_WriteBuffer(uint32_t *Buffer, uint32_t WRITE_ADDR,
		uint32_t WRITE_LENGTH) {
	uint32_t uwIndex = 0;
	/* Функція запису буферу у пам'ять */
	for (uwIndex = 0; uwIndex < WRITE_LENGTH; uwIndex++) {
		*(__IO uint32_t*) (WRITE_ADDR + 4 * uwIndex) = Buffer[uwIndex];
	}
}

void MT48LC4M32B2_ReadToBuffer(uint32_t *dstBuffer, uint32_t Read_ADDR,
		uint32_t READ_LENGTH) {
	uint32_t uwIndex = 0;
	/* Функція запису буферу у пам'ять */
	for (uwIndex = 0; uwIndex < READ_LENGTH; uwIndex++) {
		dstBuffer[uwIndex] = *(__IO uint32_t*) (Read_ADDR + 4 * uwIndex);
	}
}
