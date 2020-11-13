/*
 * JQ8400.c
 *
 *  Created on: 6 θών. 2019 γ.
 *      Author: viktor.starovit
 */

#include "JQ8400.h"


static uint8_t mp3Buffer[6];


void setVolume( unsigned char vol)
{
	mp3Buffer[0] = BYTE_START;
	mp3Buffer[1] = VOLUME_SETINGS;
	mp3Buffer[2] = 0x01;
	mp3Buffer[3] = vol;
	mp3Buffer[4] = (vol + 0xBE);

	write_nBytes(5);
}


void write_nBytes(uint8_t n)
{
	for (u_int8_t i = 0; i < n; i++) //
	{
		HAL_UART_Transmit(&HAL_UART, &mp3Buffer[i], sizeof(mp3Buffer[i]) , 100);
		HAL_Delay(40);
	}


}

void writeMySong(unsigned char track)
{
	mp3Buffer[0] = BYTE_START;
	mp3Buffer[1] = 0x07;
	mp3Buffer[2] = 0x02;
	mp3Buffer[3] = 0x00;
	mp3Buffer[4] = track;
	mp3Buffer[5] = (track+0xB3);
	write_nBytes(6);
}

