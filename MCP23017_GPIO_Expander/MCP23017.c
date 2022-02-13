/*
 * MCP21017.c
 *
 *  Created on: 12 февр. 2022 г.
 *      Author: Vitech-UA
 */

#include "MCP23017.h"

extern I2C_HandleTypeDef hi2c1;

void mcp23017_init(MCP23017_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c,
		uint16_t addr)
{
	/*Ініціалізація структури розширювача gpio*/
	hdev->hi2c = hi2c;
	hdev->addr = addr << 1;
}

void mcp23017_read_reg(MCP23017_HandleTypeDef *hdev, uint16_t reg,
		uint8_t *data)
{
	HAL_I2C_Mem_Read(hdev->hi2c, hdev->addr, reg, 1, data, 1, I2C_TIMEOUT);
}

void mcp23017_write_reg(MCP23017_HandleTypeDef *hdev, uint16_t reg,
		uint8_t *data)
{
	HAL_I2C_Mem_Write(hdev->hi2c, hdev->addr, reg, 1, &data[0], 1, I2C_TIMEOUT);
}

void mcp23013_set_pin_dir(MCP23017_HandleTypeDef *hdev, uint8_t gpio_port,
		uint8_t gpio_pin, gpio_dir_t mode)
{
	uint8_t write_data[1] =
	{ 0 };

	if (mode == GPIO_INPUT)
	{
		write_data[0] |= gpio_pin;
	}
	if (mode == GPIO_OUTPUT)
	{
		write_data[0] &= ~gpio_pin;
	}

	if (gpio_port == MCP23017_PORTA)
	{
		mcp23017_write_reg(hdev, REGISTER_IODIRA, &write_data[0]);

	}
	if (gpio_port == MCP23017_PORTB)
	{
		mcp23017_write_reg(hdev, REGISTER_IODIRB, &write_data[0]);

	}
}

void mcp23013_set_pin_state(MCP23017_HandleTypeDef *hdev, uint8_t gpio_port,
		uint8_t gpio_pin, gpio_state_t state)
{
	uint8_t write_data[1] =
	{ 0 };

	if (state == GPIO_RESET)
	{
		write_data[0] &= ~gpio_pin;
	}
	if (state == GPIO_SET)
	{
		write_data[0] |= gpio_pin;
	}

	if (gpio_port == MCP23017_PORTA)
	{
		mcp23017_write_reg(hdev, REGISTER_GPIOA, &write_data[0]);
	}
	if (gpio_port == MCP23017_PORTB)
	{
		mcp23017_write_reg(hdev, REGISTER_GPIOB, &write_data[0]);
	}
}

bool mcp23013_get_pin_state(MCP23017_HandleTypeDef *hdev, uint8_t gpio_port,
		uint8_t gpio_pin)
{
	uint8_t write_data[1] =
	{ 0 };
	if (gpio_port == MCP23017_PORTB)
	{
		mcp23017_read_reg(hdev, REGISTER_GPIOB, &write_data[0]);
	}
	if (gpio_port == MCP23017_PORTA)
	{
		mcp23017_read_reg(hdev, REGISTER_GPIOA, &write_data[0]);
	}

	if (write_data[0] & (1 << gpio_pin))
	{
		return true;
	}
	else
		return false;

}
