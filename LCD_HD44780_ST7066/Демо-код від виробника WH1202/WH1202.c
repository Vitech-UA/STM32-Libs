//          WINSTAR Display Co.,Ltd 
//=============================================================
//          WINSTAR Display Co.,Ltd
//    LCM     	 :WH 
//    Contraller :ST7066 
//    Author     :Brian lin 2009/04/21
//    history     :
//==============================================================
#include	<reg51.h>
#include 	<stdio.h>          // define I/O functions
#include 	<INTRINS.H>        // KEIL FUNCTION
#define  	Cword	0x0c //12
#define	one		0x80 // DD RAM Address 第一行之起始位置0x00
						// 所以設定DD RAM位址為0x80+0x00=0x80	
#define	two		0xc0 // DD RAM Address 第二行之起始位置0x40
						// 所以設定DD RAM位址為0x80+0x40=0xc0
#define	Data_BUS P1

sbit		busy    	=P1^7;
sbit		RS      	=P3^0;
sbit	 	RW      	=P3^7;
sbit		Enable  	=P3^4;

char bdata  flag;
sbit busy_f  = flag^0;

void CheckBusy();
void WriteIns(char);
void WriteCommand(char);
void WriteData(char);
void WriteString(char,char *);
void Initial_ks0066();
void delay(char);
void full_on(void);
void CGRAM(void);
									  
								//0123456789ABCDEF	  
unsigned char code MSG1[Cword]  ="    GSR     ";
unsigned char code MSG2[Cword]  =" Technology ";
unsigned char code MSG3[Cword]  ="  12 Word   ";
unsigned char code MSG4[Cword]  ="   2 Line   ";

//CG RAM只能產生8個字型,以下為自己畫的圖形,分別為 ↑和 ↓
unsigned char code CGRAM1[8] ={0x04,0x0E,0x15,0x04,0x04,0x04,0x04,0x04,};  // ↑
unsigned char code CGRAM2[8] ={0x04,0x04,0x04,0x04,0x04,0x15,0x0e,0x04,};  // ↓
unsigned char code CGRAM3[8] ={0x04,0x0E,0x15,0x1F,0x15,0x0E,0x04,0x00,};  //
unsigned char code CGRAM4[8] ={0x04,0x0A,0x11,0x11,0x11,0x11,0x0A,0x04,};  //
unsigned char code CGRAM5[8] ={0x02,0x06,0x0A,0x1F,0x1F,0x0A,0x06,0x02,};  //

void CheckBusy()
{
	Data_BUS = 0xff; //訊號由high變為low比較容易,所以全部設為high.
	RS = 0;
	RW = 1;  
	do
	{
	  Enable = 1;
	  busy_f = busy;
	  Enable = 0;
	 }while(busy_f);
}
//=================================
void WriteIns(char instruction)
{
	 RS = 0;
	 RW = 0;
	 Data_BUS = instruction;
	 Enable = 1;            //1us
	 _nop_();		//1us
	 Enable = 0;		//1us
}

void WriteCommand(char instruction)
{
	CheckBusy();
	RS = 0;
	RW = 0;
	Data_BUS = instruction;
	Enable = 1;            //1us
	_nop_();		//1us
	Enable = 0;		//1us
}
//=================================
//=================================
void WriteData(char data1)
{
	CheckBusy(); 
	RS = 1;
	RW = 0;
	Data_BUS = data1;
	Enable = 1;
	_nop_();
	Enable = 0; 
}
//=================================
//=================================
void WriteString(count,MSG)
char count;
char *MSG;
{
	char i;
	for(i = 0; i<count;i++)
		WriteData(MSG[i]);
}
//=================================
//=================================
void Initial_ks0066()
{
	WriteIns(0x38);
	WriteIns(0x38);
	WriteIns(0x38);
	WriteIns(0x38);
	WriteIns(0x08);
	WriteIns(0x01);
	WriteIns(0x0c);
	WriteIns(0x06);
}
//============================================
//==================================
void delay(char m)
{
	unsigned char i,j,k;
	 for(j = 0;j<m;j++)
 	{
	 	for(k = 0; k<200;k++)
	 	{
			for(i = 0; i<200;i++)
			{
			}
		}
 	}	
}
//==================================
void CGRAM()
{
	unsigned char i,j;
	WriteCommand(0x40);    //第一個圖形的CGRAM Address起始位址為000000(0x00)
					  	    //CGRAM位址設定為0x40+0x00=0x40
	for(i = 0;i<8;i++)
	{
	   	WriteData(CGRAM1[i]);
	}
	
	WriteCommand(0x48);	   //第二個圖形的CGRAM Address起始位址為001000(0x08)
					   		  //CGRAM位址設定為0x40+0x08=0x48
	for(j = 0;j<8;j++)
	{
	   	WriteData(CGRAM2[j]); //data write to CGRAM
	}

	WriteCommand(0x50);	   //第三個圖形的CGRAM Address起始位址為010000(0x10)
					   		  //CGRAM位址設定為0x40+0x10=0x50
	for(j = 0;j<8;j++)
	{
	   	WriteData(CGRAM3[j]); //data write to CGRAM
	}

	WriteCommand(0x58);	   //第四個圖形的CGRAM Address起始位址為011000(0x18)
					   		  //CGRAM位址設定為0x40+0x18=0x58
	for(j = 0;j<8;j++)
	{
	   	WriteData(CGRAM4[j]); //data write to CGRAM
	}

	WriteCommand(0x60);	   //第四個圖形的CGRAM Address起始位址為100000(0x20)
					   		  //CGRAM位址設定為0x40+0x20=0x60
	for(j = 0;j<8;j++)
	{
	   	WriteData(CGRAM5[j]); //data write to CGRAM
	}
}

void full_on()
{
	unsigned char i=0,j=0;

	WriteCommand(0x80);

	for (j=0;j<2;j++)
	{
		for (i=0;i<20;i++)
			WriteData(0xff);
	}
}

void clear()
{
	unsigned char i=0,j=0;

	WriteCommand(0x80);

	for (j=0;j<2;j++)
	{
		for (i=0;i<20;i++)
			WriteData(0x00);
	}
}


main()
{
	Initial_ks0066();

	while(1)
	{
		WriteIns(one);
		WriteString(Cword,MSG1);
		WriteIns(two);
		WriteString(Cword,MSG2);
		delay(20);
		WriteIns(one);
		WriteString(Cword,MSG3);
		WriteIns(two);
		WriteString(Cword,MSG4);
		delay(20);
	}
}


