/*
 * MT48LC4M32B2.h
 *
 *  Created on: May 20, 2020
 *      Author: viktor.starovit
 */

#ifndef MT48LC4M32B2_MT48LC4M32B2_H_
#define MT48LC4M32B2_MT48LC4M32B2_H_

#include <stm32f7xx_hal.h>
#include <stdint.h>
#include <string.h>


#define BUFFER_SIZE ((uint32_t)0x64)
#define WRITE_READ_ADDR ((uint32_t)0x0800)
#define SDRAM_BANK_ADDR ((uint32_t)0xC0000000)

#define SDRAM_TIMEOUT     ((uint32_t)0xFFFF)
#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)



void MT48LC4M32B2_Init(SDRAM_HandleTypeDef *sdram);
void MT48LC4M32B2_WriteBuffer(uint32_t* Buffer, uint32_t WRITE_ADDR, uint32_t WRITE_LENGTH);
void MT48LC4M32B2_ReadToBuffer(uint32_t *dstBuffer, uint32_t Read_ADDR,uint32_t READ_LENGTH);

#endif /* MT48LC4M32B2_MT48LC4M32B2_H_ */
