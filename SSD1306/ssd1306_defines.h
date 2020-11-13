/*
 * ssd1306_defines.h
 *
 *  Created on: 14 ���. 2018 �.
 *      Author: Andriy
 */

#ifndef SSD1306_DEFINES_H_
#define SSD1306_DEFINES_H_

#define STM32F1XX					// OR #define STM32F0XX OR #define STM32F4XX etc.
#define STM32_I2C_PORT		hi2c1 	// I2C port as defined in main generated by CubeMx
#define SSD1306_ADDRESS		0x3C	// I2C address display
#define SSD1306_128X64 				// OR #define SSD1306_128X32

#endif /* SSD1306_DEFINES_H_ */
