/*
 * SPI.h
 *
 *  Created on: 14 февр. 2021 г.
 *      Author: Embed Viktor
 */

#ifndef SPI_H_
#define SPI_H_

#include "Gpio.h"

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

#define STM32_SERIES 0
// #define STM32_SERIES 1
// #define STM32_SERIES 3
// #define STM32_SERIES 4

#if (STM32_SERIES == 0)
//#define SPI1_USE_HARDWARE_In_nCS
#define SPI1_USE_HARDWARE_Out_nCS

//#define SPI2_USE_HARDWARE_In_nCS
#define SPI2_USE_HARDWARE_Out_nCS

// SPI1 GPIO Definition //
#define SPI1_SCK_PORT_PA5
//#define SPI1_SCK_PORT_PB3
#define SPI1_MISO_PORT_PA6
//#define SPI1_MISO_PORT_PB4
#define SPI1_MOSI_PORT_PA7
//#define SPI1_MOSI_PORT_PB5

#ifdef SPI1_USE_HARDWARE_Out_nCS
#define SPI1_nSS_PORT_PA4
//#define SPI1_nSS_PORT_PA15
#endif

#ifdef SPI1_USE_HARDWARE_In_nCS
		#define SPI1_nSS_PORT_PA4
	  //#define SPI1_nSS_PORT_PA15
	#endif

// SPI2 GPIO Definition //
#define SPI2_SCK_PORT_PA5
//#define SPI2_SCK_PORT_PB3
#define SPI2_MISO_PORT_PA6
//#define SPI2_MISO_PORT_PB4
#define SPI2_MOSI_PORT_PA7
//#define SPI2_MOSI_PORT_PB5

#ifdef SPI2_USE_HARDWARE_Out_nCS
#define SPI2_nSS_PORT_PB12
#endif

#ifdef SPI2_USE_HARDWARE_In_nCS
		#define SPI2_nSS_PORT PB12
	#endif
#endif

#if (STM32_SERIES == 1)

#endif

#if (STM32_SERIES == 3)

#endif

#if (STM32_SERIES == 4)

#endif

#if (STM32_SERIES != 0 || STM32_SERIES != 1 || STM32_SERIES != 3 || STM32_SERIES != 4)
// #error Please uncoment STM32_SERIES in SPI.h
#endif

class SPI {
public:
	SPI(SPI_TypeDef *Port, SPI_mode_t Mode);

private:
	void InitGpio();
	SPI_TypeDef *SPI_Item;
	GPIO_TypeDef *MOSI_PORT;
	uint16_t MOSI_PIN;
	GPIO_TypeDef *MISO_PORT;
	uint16_t MISO_PIN;
	GPIO_TypeDef *SCK_PORT;
	uint16_t SCK_PIN;
	GPIO_TypeDef *nSC_PORT;
	uint16_t nSC_PIN;


};

#endif /* SPI_H_ */
