/*
 * AT24Cxx.c
 *
 *  Created on: 20 ����. 2018 �.
 *      Author: Andriy
 */
#include "AT24Cxx.h"

// ������������ ������ � 7 �� �� �� 8 ��
static unsigned eeprom_address = EEPROM_ADDRESS << 1;
// ����� ���������� �� �������� ������ �������.
static unsigned inpage_addr_mask = EEPROM_PAGESIZE - 1;

static HAL_StatusTypeDef AT24Cxx_WriteReadEEPROM(unsigned address, const void* src, unsigned len, bool write);
static unsigned size_to_page_end(unsigned addr);

HAL_StatusTypeDef AT24Cxx_IsConnected(void)
{
	return HAL_I2C_IsDeviceReady(&EEPROM_I2C, eeprom_address, 1, EEPROM_TIMEOUT);
}

HAL_StatusTypeDef AT24Cxx_ReadEEPROM(unsigned address, const void* src, unsigned len)
{
	return AT24Cxx_WriteReadEEPROM(address, src, len, false);
}

HAL_StatusTypeDef AT24Cxx_WriteEEPROM(unsigned address, const void* src, unsigned len)
{
	return AT24Cxx_WriteReadEEPROM(address, src, len, true);
}

static HAL_StatusTypeDef AT24Cxx_WriteReadEEPROM(unsigned address, const void* src, unsigned len, bool write)
{
	uint8_t *pdata = (uint8_t*) src;

	HAL_StatusTypeDef result = HAL_OK;

    // ����� ������ �� ���� ���� �����, ��� �� ��������� ����� �������
    unsigned max_portion = size_to_page_end(address);
    unsigned portion;

    while (len != 0 && result == HAL_OK)
    {
        portion = len;              // ���, �� ���������� -- � ���� ������

        if (portion > max_portion)
        {
        	portion = max_portion;  // ������ ��������, ��������
        }

        // �������� �� �������, ��� ����� �����������
        if(write)
		{
			result = HAL_I2C_Mem_Write(&EEPROM_I2C,
									eeprom_address,
									address,
									I2C_MEMADD_SIZE_16BIT,
									pdata,
									portion,
									EEPROM_TIMEOUT);
		}
        else
        {
        	result = HAL_I2C_Mem_Read(&EEPROM_I2C,
        	        				eeprom_address,
        							address,
        							I2C_MEMADD_SIZE_16BIT,
        							pdata,
        							portion,
        							EEPROM_TIMEOUT);
        }

        // � ��������, �� ��� �� �������.
        len     -= portion;
        address += portion;
        pdata   += portion;

        // ��������, ���� ������ ����, ���� �� ������� �������
        max_portion = EEPROM_PAGESIZE;

        if(write)
        {
        	HAL_Delay(EEPROM_WRITE);
        }
        else
        {
        	HAL_Delay(EEPROM_WRITE / 2);
        }
    }

    return result;
}

// ���������� ������ �� ���� �������
static unsigned size_to_page_end(unsigned addr)
{
    return (~addr & inpage_addr_mask) + 1;
}
