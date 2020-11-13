#ifndef _45DBXXCONFIG_H
#define _45DBXXCONFIG_H

#define	_45DBXX_USE_FREERTOS		0
#define _45BDXX_USE_UART_DEBUG 1
#define MAX_BUFFER_SIZE 254

#define	_45DBXX_SPI						hspi1 /*SPI - на якому висить мікросхема*/
#define	_45DBXX_CS_GPIO					GPIOA /*Порт на якому знаходитьс ножка CS мікросхеми*/
#define	_45DBXX_CS_PIN					GPIO_PIN_4 /*Пін який відповідає за функцію чіп селект*/

#endif
