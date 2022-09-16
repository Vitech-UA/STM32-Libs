
#include "stdint.h"
#include "../LCD_WH1602/i2c_lcd_cfg.h"




#define e_set()   WriteByteI2CLCD(portlcd|=0x04) // ��������� ����� E � 1
#define e_reset()   WriteByteI2CLCD(portlcd&=~0x04)  // ��������� ����� E � 0
#define rs_set()    WriteByteI2CLCD(portlcd|=0x01) // ��������� ����� RS � 1
#define rs_reset()   WriteByteI2CLCD(portlcd&=~0x01)  // ��������� ����� RS � 0
#define setled()    WriteByteI2CLCD(portlcd|=0x08) // ��������� ���������
#define setwrite()   WriteByteI2CLCD(portlcd&=~0x02)  // ��������� ������ � ������ �������

/*���������*/
void WriteByteI2CLCD(uint8_t bt);
void LCD_Clear();
void sendbyte(uint8_t c, uint8_t mode);
void sendhalfbyte(uint8_t c);
void LCD_SendChar(char ch);
void LCD_SetPos(uint8_t x, uint8_t y);
void LCD_ini();
void LCD_String(uint8_t* st);
