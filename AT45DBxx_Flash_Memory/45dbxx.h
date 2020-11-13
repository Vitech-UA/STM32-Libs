#ifndef _45DBXX_H
#define _45DBXX_H

#include <stdint.h>
#include <stdbool.h>
#include "stddef.h"
typedef struct
{
	uint8_t		FlashSize_MBit;	
	uint16_t	PageSize;
	uint16_t	Pages;
	uint8_t		Shift;
}AT45dbxx_t;


extern AT45dbxx_t	AT45dbxx;

typedef struct __flash_storage
{
    int pg_num;
    int pg_shifts;
    uint32_t block_sz;
    uint32_t erase_sz;
    uint32_t n_eraseblocks;
} flash_storage_t;

bool AT45dbxx_Init(void);
void AT45dbxx_EraseChip(void);
void AT45dbxx_ErasePage(uint16_t page);
void AT45dbxx_WritePage(uint8_t *Data, uint16_t len, uint16_t page);
void AT45dbxx_ReadPage(uint8_t* Data, uint16_t len, uint16_t page);
void AT45dbxx_Select_Chip();
void AT45dbxx_Release_Chip();
void AT45dbxx_WriteBMP();
void AT45dbxx_WriteBlock(uint32_t sblock, size_t nblocks,const uint8_t *buf);
void AT45dbxx_Read_Block(uint32_t sblock, size_t nblocks, uint8_t *buf);

#endif
