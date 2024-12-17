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
	uint8_t current_data;
	mcp23017_read_reg(hdev, REGISTER_GPIOA | gpio_port, &current_data);

	if (state == GPIO_RESET)
	{
		current_data &= ~gpio_pin;
	}
	else if (state == GPIO_SET)
	{
		current_data |= gpio_pin;
	}

	mcp23017_write_reg(hdev, REGISTER_GPIOA | gpio_port, &current_data);
}

bool mcp23013_get_pin_state(MCP23017_HandleTypeDef *hdev, uint8_t gpio_port,
		uint8_t gpio_pin)
{
	uint8_t write_data[1] =
	{ 0 };

	mcp23017_read_reg(hdev, REGISTER_GPIOA | gpio_port, &write_data[0]);

	if (write_data[0] & (1 << gpio_pin))
	{
		return true;
	}
	else
		return false;

}

void mcp23013_set_pin_interrupt(MCP23017_HandleTypeDef *hdev, uint8_t gpio_port,
		uint8_t gpio_pin)
{

	uint8_t write_data[1] =
	{ 0 };

	/* Default value 0
	 (Переривання спрацює тоді, коли значення на піні буде протилежним
	 значенню у регістрі DEFVAL. В даному випадку тоді, коли на піні буде лог 0 ) */
	write_data[0] |= gpio_pin;
	mcp23017_write_reg(hdev, REGISTER_DEFVALB, &write_data[0]);

	// Pin value is compared against the associated bit in the DEFVAL register.
	write_data[0] |= gpio_pin;
	mcp23017_write_reg(hdev, REGISTER_INTCONB, &write_data[0]);

	// Enables GPIO input pin for interrupt-on-change event.
	write_data[0] |= gpio_pin;
	mcp23017_write_reg(hdev, REGISTER_GPINTENB, &write_data[0]);

}

void mcp23017_INTpin_config(MCP23017_HandleTypeDef *hdev, uint8_t gpio_port,
		int_out_t INT_pin_type)
{
	uint8_t write_data[1] =
	{ 0 };

	write_data[0] |= (INT_pin_type << 2);
	if (gpio_port == MCP23017_PORTA)
	{
		mcp23017_write_reg(hdev, REGISTER_IOCONA, &write_data[0]);
	}
	if (gpio_port == MCP23017_PORTB)
	{
		mcp23017_write_reg(hdev, REGISTER_IOCONB, &write_data[0]);
	}

}

void mcp23013_set_pullup_pulldown(MCP23017_HandleTypeDef *hdev,
		uint8_t gpio_port, uint8_t gpio_pin, gpio_pullup_down_t mode)
{

	uint8_t write_data[1] =
	{ 0 };
	if (mode == PIN_PULLUP_ENABLED)
	{
		write_data[0] |= gpio_pin;
	}
	if (mode == PIN_PULLUP_DISABLED)
	{
		write_data[0] &= ~gpio_pin;
	}
	if (gpio_port == MCP23017_PORTA)
	{
		mcp23017_write_reg(hdev, REGISTER_GPPUA, &write_data[0]);
	}
	if (gpio_port == MCP23017_PORTB)
	{
		mcp23017_write_reg(hdev, REGISTER_GPPUB, &write_data[0]);
	}

}
