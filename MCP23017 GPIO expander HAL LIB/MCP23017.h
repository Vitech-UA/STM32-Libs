#include "main.h"
#include "stdbool.h"
// Ports
#define MCP23017_PORTA			0x12
#define MCP23017_PORTB			0x13
// Address (A0-A2)
#define MCP23017_ADDRESS_20		0x20
#define MCP23017_ADDRESS_21		0x21
#define MCP23017_ADDRESS_22		0x22
#define MCP23017_ADDRESS_23		0x23
#define MCP23017_ADDRESS_24		0x24
#define MCP23017_ADDRESS_25		0x25
#define MCP23017_ADDRESS_26		0x26
#define MCP23017_ADDRESS_27		0x27

// I/O Pins
#define MCP23017_GPIO0	0x01
#define MCP23017_GPIO1	0x02
#define MCP23017_GPIO2	0x04
#define MCP23017_GPIO3	0x08
#define MCP23017_GPIO4	0x10
#define MCP23017_GPIO5	0x20
#define MCP23017_GPIO6	0x40
#define MCP23017_GPIO7	0x80

// I/O Direction
// Default state: MCP23017_IODIR_ALL_INPUT
#define MCP23017_IODIR_ALL_OUTPUT	0x00
#define MCP23017_IODIR_ALL_INPUT	0xFF
#define MCP23017_IODIR_IO0_INPUT	0x01
#define MCP23017_IODIR_IO1_INPUT	0x02
#define MCP23017_IODIR_IO2_INPUT	0x04
#define MCP23017_IODIR_IO3_INPUT	0x08
#define MCP23017_IODIR_IO4_INPUT	0x10
#define MCP23017_IODIR_IO5_INPUT	0x20
#define MCP23017_IODIR_IO6_INPUT	0x40
#define MCP23017_IODIR_IO7_INPUT	0x80

// Input Polarity
// Default state: MCP23017_IPOL_ALL_NORMAL
#define MCP23017_IPOL_ALL_NORMAL	0x00
#define MCP23017_IPOL_ALL_INVERTED	0xFF
#define MCP23017_IPOL_IO0_INVERTED	0x01
#define MCP23017_IPOL_IO1_INVERTED	0x02
#define MCP23017_IPOL_IO2_INVERTED	0x04
#define MCP23017_IPOL_IO3_INVERTED	0x08
#define MCP23017_IPOL_IO4_INVERTED	0x10
#define MCP23017_IPOL_IO5_INVERTED	0x20
#define MCP23017_IPOL_IO6_INVERTED	0x40
#define MCP23017_IPOL_IO7_INVERTED	0x80

// Pull-Up Resistor
// Default state: MCP23017_GPPU_ALL_DISABLED
#define MCP23017_GPPU_ALL_DISABLED	0x00
#define MCP23017_GPPU_ALL_ENABLED	0xFF
#define MCP23017_GPPU_IO0_ENABLED	0x01
#define MCP23017_GPPU_IO1_ENABLED	0x02
#define MCP23017_GPPU_IO2_ENABLED	0x04
#define MCP23017_GPPU_IO3_ENABLED	0x08
#define MCP23017_GPPU_IO4_ENABLED	0x10
#define MCP23017_GPPU_IO5_ENABLED	0x20
#define MCP23017_GPPU_IO6_ENABLED	0x40
#define MCP23017_GPPU_IO7_ENABLED	0x80

// Registers
#define REGISTER_IODIRA		0x00
#define REGISTER_IODIRB		0x01
#define REGISTER_IPOLA		0x02
#define REGISTER_IPOLB		0x03
#define REGISTER_GPINTENA	0x04
#define REGISTER_GPINTENB	0x05
#define REGISTER_DEFVALA	0x06
#define REGISTER_DEFVALB	0x07
#define REGISTER_INTCONA	0x08
#define REGISTER_INTCONB	0x09
#define REGISTER_IOCONA		0x0A
#define REGISTER_IOCONB		0x0A
#define REGISTER_GPPUA		0x0C
#define REGISTER_GPPUB		0x0D
#define REGISTER_INTFA		0x0E
#define REGISTER_INTFB		0x0F
#define REGISTER_INTCAPA	0x10
#define REGISTER_INTCAPB	0x11
#define REGISTER_GPIOA		0x12
#define REGISTER_GPIOB		0x13
#define REGISTER_OLATA		0x14
#define REGISTER_OLATB		0x15

#define I2C_TIMEOUT		10
// Control settings
#define IOCON_BANK      0x80 // Banked registers for Port A and B
#define IOCON_BYTE_MODE 0x20 // Disables sequential operation, Address Ptr does not increment
//   If Disabled and Bank = 0, operations toggle between Port A and B registers
//   If Disabled and Bank = 1, operations do not increment registeraddress
#define IOCON_HAEN      0x08 // Hardware address enable

#define INTERRUPT_POLARITY_BIT 0x02
#define INTERRUPT_MIRROR_BIT   0x40

#define PORT_DIR_OUT   0x00
#define PORT_DIR_IN    0xFF

#define MCP2317_ADDR 0x4E

typedef struct
{
	I2C_HandleTypeDef *hi2c;
	uint16_t addr;
} MCP23017_HandleTypeDef;

typedef enum
{
	GPIO_OUTPUT, GPIO_INPUT
} gpio_dir_t;

typedef enum
{
	PIN_PULLUP_DISABLED, PIN_PULLUP_ENABLED
} gpio_pullup_down_t;

typedef enum
{
	GPIO_RESET, GPIO_SET
} gpio_state_t;

typedef enum
{
	INT_OUT_PP, INT_OUT_OD,
} int_out_t;


void mcp23017_init(MCP23017_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c,
		uint16_t addr);
void mcp23017_read_reg(MCP23017_HandleTypeDef *hdev, uint16_t reg,
		uint8_t *data);
void mcp23017_write_reg(MCP23017_HandleTypeDef *hdev, uint16_t reg,
		uint8_t *data);
void mcp23013_set_pin_state(MCP23017_HandleTypeDef *hdev, uint8_t gpio_port,
		uint8_t gpio_pin, gpio_state_t state);
void mcp23013_set_pin_dir(MCP23017_HandleTypeDef *hdev, uint8_t gpio_port,
		uint8_t gpio_pin, gpio_dir_t mode);
bool mcp23013_get_pin_state(MCP23017_HandleTypeDef *hdev, uint8_t gpio_port,
		uint8_t gpio_pin);
void mcp23013_set_pin_interrupt(MCP23017_HandleTypeDef *hdev, uint8_t gpio_port,
		uint8_t gpio_pin);
void mcp23017_INTpin_config(MCP23017_HandleTypeDef *hdev, uint8_t gpio_port,
		int_out_t INT_pin_type);
void mcp23013_set_pullup_pulldown(MCP23017_HandleTypeDef *hdev, uint8_t gpio_port,
		uint8_t gpio_pin, gpio_pullup_down_t mode);
