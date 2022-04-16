/**
    @author Stanislav Lakhtin
    @date   11.07.2016
    @brief  Реализация протокола 1wire на базе библиотеки libopencm3 для микроконтроллера STM32F103
            Возможно, библиотека будет корректно работать и на других uK (требуется проверка).
            Общая идея заключается в использовании аппаратного USART uK для иммитации работы 1wire.
            Подключение устройств осуществляется на выбранный USART к TX пину, который должен быть подтянут к линии питания сопротивлением 4.7К.
            Реализация библиотеки осуществляет замыкание RX на TX внутри uK, оставляя ножку RX доступной для использования в других задачах.
 */

#ifndef STM32_DS18X20_ONEWIRE_H
#define STM32_DS18X20_ONEWIRE_H
#include <stdint.h>
#define ONEWIRE_NOBODY 0xF0 //команда поиска ROM
#define ONEWIRE_SEARCH 0xF0 //команда поиска ROM
#define ONEWIRE_SKIP_ROM 0xCC  //команда пропуска ROM
#define ONEWIRE_READ_ROM 0x33  
#define ONEWIRE_MATCH_ROM 0x55 //команда совпадение ROM позволяет мастеру обращаться к конкретному  ведомому устройству
#define ONEWIRE_CONVERT_TEMPERATURE 0x44
#define ONEWIRE_READ_SCRATCHPAD 0xBE    //команда для чтения памяти датчика
#define ONEWIRE_WRITE_SCRATCHPAD 0x4E   //команда запись в память дтачика
#define ONEWIRE_COPY_SCRATCHPAD 0x48
#define ONEWIRE_RECALL_E2 0xB8

#ifndef MAXDEVICES_ON_THE_BUS
#define MAXDEVICES_ON_THE_BUS 2  // maximum planned number of devices on the bus
#endif

#define DS18B20 0x28  //код семейсва датчика 
#define DS18S20 0x10  //код семейсва датчика 

#define WIRE_0    0x00 // 0x00 --default
#define WIRE_1    0xff //ответ
#define OW_READ   0xff

typedef struct {
  int8_t inCelsus;
  uint8_t frac;
} Temperature; //

typedef struct {
  uint8_t family;
  uint8_t code[6];
  uint8_t crc;
} RomCode; //

typedef struct {
  uint8_t crc;
  uint8_t reserved[3];
  uint8_t configuration;
  uint8_t tl;
  uint8_t th;
  uint8_t temp_msb;
  uint8_t temp_lsb;
} Scratchpad_DS18B20;//

typedef struct {
  uint8_t crc;
  uint8_t count_per;
  uint8_t count_remain;
  uint8_t reserved[2];
  uint8_t tl;
  uint8_t th;
  uint8_t temp_msb;
  uint8_t temp_lsb;
} Scratchpad_DS18S20;//

typedef struct {
  RomCode ids[MAXDEVICES_ON_THE_BUS];//для всех ромов наших датчиков
  int lastDiscrepancy;
  uint8_t lastROM[8];//последний считанный ROM для поиска всех ROM
} OneWire;

typedef struct {
	int device;
	char info[30];
}DEVInfo;
DEVInfo devInfo;

void usart_setup_(uint32_t baud);

uint16_t owResetCmd(void);

int owSearchCmd(OneWire *ow);

void owSkipRomCmd(OneWire *ow);

uint8_t owCRC8(RomCode *rom);

void owMatchRomCmd(RomCode *rom);

void owConvertTemperatureCmd(OneWire *ow, RomCode *rom);

uint8_t* owReadScratchpadCmd(OneWire *ow, RomCode *rom, uint8_t *data);

void owCopyScratchpadCmd(OneWire *ow, RomCode *rom);

void owRecallE2Cmd(OneWire *ow, RomCode *rom);

Temperature readTemperature(OneWire *ow, RomCode *rom, uint8_t reSense);

void owSend(uint16_t data);

void owSendByte(uint8_t data);

uint16_t owEchoRead(void);

void owReadHandler(void);

int get_ROMid (void);

void get_Temperature (void);

#endif //STM32_DS18X20_ONEWIRE_H
