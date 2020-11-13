#include "main.h"
#include "stm32f0xx_hal.h"
#include "stdbool.h"
#include "stm32f0xx_hal_flash.h"
#include "ssd1306.h"

extern I2C_HandleTypeDef  hi2c1;

#define UART_BUFFER_SIZE 60

//Адреси регістрів

#define STATUS_REG 0x00
#define MEAS_MODE_REG 0x01
#define ALG_RESULT_DATA 0x02
#define ENV_DATA 0x05
#define NTC_REG 0x06
#define THRESHOLDS 0x10
#define BASELINE 0x11
#define HW_ID_REG 0x20
#define ERROR_ID_REG 0xe0
#define APP_START_REG 0xF4
#define SW_RESET 0xff

#define CCS_811_ADDRESS 0x5a /*Дефолтна адреса чипа*/
#define CCS_811_ADDRESS_SHIFTED 0xb4 /*Дефолтна адреса чипа << 1*/
#define GPIO_WAKE 0x5
#define DRIVE_MODE_IDLE 0x0
#define DRIVE_MODE_1SEC 1
#define DRIVE_MODE_10SEC 2
#define DRIVE_MODE_60SEC 3
#define INTERRUPT_DRIVEN 0x8
#define THRESHOLDS_ENABLED 0x4

#define CCS811_ADDWR 0xb4
#define CSS811_STATUS 0x00
#define CSS811_MEAS_MODE 0x01
#define CSS811_ALG_RESULT_DATA 0x02
#define CSS811_RAW_DATA 0x03
#define CSS811_ENV_DATA 0x05
#define CSS811_NTC 0x06
#define CSS811_THRESHOLDS 0x10
#define CSS811_BASELINE 0x11
#define CSS811_HW_ID 0x20
#define CSS811_HW_VERSION 0x21
#define CSS811_FW_BOOT_VERSION 0x23
#define CSS811_FW_APP_VERSION 0x24
#define CSS811_ERROR_ID 0xe0
#define CSS811_APP_START 0xf4
#define CSS811_SW_RESET 0xff
#define u8 u_int8_t
#define u16 u_int16_t

#define CCS811_ERRSTAT_ERROR               0x0001 // There is an error, the ERROR_ID register (0xE0) contains the error source
#define CCS811_ERRSTAT_I2CFAIL             0x0002 // Bit flag added by software: I2C transaction error
#define CCS811_ERRSTAT_DATA_READY          0x0008 // A new data sample is ready in ALG_RESULT_DATA
#define CCS811_ERRSTAT_APP_VALID           0x0010 // Valid application firmware loaded
#define CCS811_ERRSTAT_FW_MODE             0x0080 // Firmware is in application mode (not boot mode)
#define CCS811_ERRSTAT_WRITE_REG_INVALID   0x0100 // The CCS811 received an I²C write request addressed to this station but with invalid register address ID
#define CCS811_ERRSTAT_READ_REG_INVALID    0x0200 // The CCS811 received an I²C read request to a mailbox ID that is invalid
#define CCS811_ERRSTAT_MEASMODE_INVALID    0x0400 // The CCS811 received an I²C request to write an unsupported mode to MEAS_MODE
#define CCS811_ERRSTAT_MAX_RESISTANCE      0x0800 // The sensor resistance measurement has reached or exceeded the maximum range
#define CCS811_ERRSTAT_HEATER_FAULT        0x1000 // The heater current in the CCS811 is not in range
#define CCS811_ERRSTAT_HEATER_SUPPLY       0x2000 // The heater voltage is not being applied correctly

#define CCS811_ERRSTAT_OK                  (CCS811_ERRSTAT_DATA_READY | CCS811_ERRSTAT_APP_VALID | CCS811_ERRSTAT_FW_MODE)

// Таймінги
#define CCS811_WAIT_AFTER_RESET_US     2000 // The CCS811 needs a wait after reset
#define CCS811_WAIT_AFTER_APPSTART_US  1000 // The CCS811 needs a wait after app start
#define CCS811_WAIT_AFTER_WAKE_US        50 // The CCS811 needs a wait after WAKE signal
#define CCS811_WAIT_AFTER_APPERASE_MS   500 // The CCS811 needs a wait after app erase (300ms from spec not enough)
#define CCS811_WAIT_AFTER_APPVERIFY_MS   70 // The CCS811 needs a wait after app verify
#define CCS811_WAIT_AFTER_APPDATA_MS     50 // The CCS811 needs a wait after writing app data

#define CCS811_FW_BOOT_VERSION  0x23 // 2 bytes

u8 CCS811_Read_Register(u8 addr);
void setEnvironmentalData(float relativeHumidity, float temperature);
void CCS811_Get_Result();
void CCS811_write(u8 address, u8 regisster, u8 *tx_data_ptr, u8 length);
void CCS811_read(u_int8_t address, u_int8_t *rx_data_ptr, u_int8_t length);
void CCS811_Init();
void CCS811_Start_Application();
void CCS811_Write_Register(u8 addr, u8 val);
void CCS811_Reset();
void CCS811_WakeUp();
void CCS811_WakeDown();
void CCS811_GET_HwID();
void CCS811_CHECK_HW_Version();
void CCS811_CHECK_APP_VALID();
void CCS811_CHECK_APP_Version();
void CCS811_Enable_Application_MODE();
void CCS811_CHECK_I2C_ADDRES();
void CCS811_GET_Bootloader_Version();
void CCS811_CHECK_BOOT_MODE();
void CCS811_CHECK_READY_DATA();
void CCS811_Read_Data();
u8 CCS811_GET_StatusRegister();
void CCS811_EnableMaesurement(u8 mode);
void CCS811_DisableMaesurement();
void CCS811_GetSensorResistance();
void CCS811_Enable_TreshholdInterrupt();
