/*
 * SPI.h
 *
 *  Created on: 14 февр. 2021 г.
 *      Author: Embed Viktor
 */



#ifndef SPI_H_
#define SPI_H_

#define STM32_SERIES 0
// #define STM32_SERIES 1
// #define STM32_SERIES 3
// #define STM32_SERIES 4

//#define USE_HARDWARE_nCS // Розкоментувати для апаратного контролю піном nCS, закоментувати для програмного

#if (STM32_SERIES == 0)

#endif

#if (STM32_SERIES == 1)

#endif

#if (STM32_SERIES == 3)

#endif

#if (STM32_SERIES == 4)

#endif

class SPI {
public:
	SPI();
};

#endif /* SPI_H_ */
