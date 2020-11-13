#include "CCS811.h"

//u_int8_t i2c_buff[8];
#define CCS811I2C &hi2c1

u8 BurnIn_Time_Complete = 0;
u8 RunIn_Time_Complete = 0;
u8 Baseline_Time_Complete = 0;
u8 EBaseline_Time_Complete = 0;
//These are the air quality values obtained from the sensor
unsigned int tVOC = 0;
unsigned int CO2 = 0;
uint16_t adc_raw = 0;
u8 current_value = 0;
u8 dummyread = 0;
u8 appvalue = 0;
u8 errvalue = 0;
u8 mosetting = 0;
u8 dtvalue = 0;
uint32_t ELBaseline_period = 0;
uint32_t ALBaseline_period = 0;
u8 Mode_CCS811 = 1;
float relativeHumidity = 65.0, temperature = 25.0;

int _appversion;

bool wake_gpio_enabled = true;
////////////////*****///////////////////////////////////*****///////////////////////////////////*****///////////////////

void CCS811_write(u8 address, u8 regisster, u8 *tx_data_ptr, u8 length)
{
	        HAL_I2C_Mem_Write(CCS811I2C, CCS_811_ADDRESS, regisster,
			I2C_MEMADD_SIZE_8BIT, &tx_data_ptr, length, 100);

	while (HAL_I2C_GetState(CCS811I2C) != HAL_I2C_STATE_READY)
	{
	}

	while (HAL_I2C_GetState(CCS811I2C) != HAL_I2C_STATE_READY)
	{
	}

}
////////////////*****///////////////////////////////////*****///////////////////////////////////*****///////////////////
void CCS811_read(u_int8_t address, u_int8_t *rx_data_ptr, u_int8_t length)
{
	HAL_I2C_Mem_Read(CCS811I2C, CCS_811_ADDRESS, (u8) address,
	I2C_MEMADD_SIZE_8BIT, &rx_data_ptr, sizeof(u_int8_t), 100);
}
////////////////*****///////////////////////////////////*****///////////////////////////////////*****///////////////////
void CCS811_Init()
{
		CCS811_Reset();   			 /* Програмне скидання сенсора записуємо код CSS811_SW_RESET, 0x11,0xE5,0x72,0x8A у регістр 0xFF */
	    CCS811_GET_HwID();			 /*Зчитуємо Hardware ID, з регістру 0х20*/
	    CCS811_CHECK_HW_Version();   /*Зчитуємо апартну версію сенсора має повернути не менше 0xF0*/
	    CCS811_CHECK_APP_Version();  /*Зчитуємо програмну рег 0х24 версію сенсора має повернути не менше 0xF0*/
	    CCS811_Start_Application();  /*Переводимо сенсор з режиму boot в режим application
	                                  цим підготовлюємо його до перетворення даних*/

	    CCS811_EnableMaesurement(DRIVE_MODE_1SEC); /* Запускаємо режим підготовки даних 1 раз/секунду.
	                                                можливо 1 раз/10 секунд, 1 раз/60 секунд і Запитувати замір шляхом дригання
	                                                ноги INT сенсора CCS811  */

}
////////////////*****///////////////////////////////////*****///////////////////////////////////*****///////////////////
u8 CCS811_Read_Register(u8 addr)
{
			u8 dt;
             /*Функція читання даних з регістру CCS811, Важливо: У HAL для конекту з пристроєм по I2C використовується
              Реальна адреса пристрою, але зсунута вліво на 1, тобто реальна адреса чіпа на шині 0х5а, а працюємо ми з адресою
              0x5A << 1 = 0xb4 вона задефайняна, як CCS_811_ADDRESS_SHIFTED у файлі CCS811.h */
			 HAL_I2C_Mem_Read(CCS811I2C, CCS_811_ADDRESS_SHIFTED, ( u8 )addr,1, &dt, 1,300 );
			 while (HAL_I2C_GetState(CCS811I2C) != HAL_I2C_STATE_READY);


			return dt;
 }////////*****///////////////////////////////////*****///////////////////////////////////*****///////////////////
////////////////*****///////////////////////////////////*****///////////////////////////////////*****///////////////////
void CCS811_EnableMaesurement(u8 mode)
{
	//Mode 0 = Вимкнено, режим мінімального енергоспоживання.(Реалізовано в функції ССS811_DisableMeasurement)
	//Mode 1 = read every 1s
	//Mode 2 = every 10s
	//Mode 3 = every 60s
	//Mode 4 = Режим запитування даних у перериванні.

	if (mode > 4) mode = 4; //орекція помилки

	mosetting = CCS811_Read_Register(MEAS_MODE_REG); //Зчитуємо поточну конфігурацію.

	mosetting &= ~(7 << 4); //Очищаємо Drive Mode bits
	mosetting |= (mode << 4); //Маскуємо режим
    CCS811_WakeUp();
	CCS811_Write_Register(MEAS_MODE_REG, mosetting);
	CCS811_WakeDown(); /*Перезавантажуємо сенсор*/

	mosetting = CCS811_Read_Register(MEAS_MODE_REG); //Read what's currently there
	CCS811_WakeDown();
	u_int8_t LCD[UART_BUFFER_SIZE];
	sprintf(LCD, "Drive Mode: 0x%02x\r\n", mosetting);
	//HAL_UART_Transmit(&huart1, LCD, strlen(LCD), 100);
}
////////////////*****///////////////////////////////////*****///////////////////////////////////*****///////////////////
void CCS811_Write_Register(u8 addr, u8 val)
{
	/*Функція запису даних в регістри CCS811, Важливо: У HAL для конекту з пристроєм по I2C використовується
	 Реальна адреса пристрою, але зсунута вліво на 1, тобто реальна адреса чіпа на шині 0х5а, а працюємо ми з адресою
	 0x5A << 1 = 0xb4 вона задефайняна, як CCS_811_ADDRESS_SHIFTED у файлі CCS811.h */
	HAL_I2C_Mem_Write(CCS811I2C, CCS_811_ADDRESS_SHIFTED, (u8) addr,
			I2C_MEMADD_SIZE_8BIT, &val, 1, 300);
	/*Очікую поки шина повернеться у статус готовності*/
	while (HAL_I2C_GetState(CCS811I2C) != HAL_I2C_STATE_READY)
	{
	}

	while (HAL_I2C_IsDeviceReady(CCS811I2C, CCS_811_ADDRESS_SHIFTED, 10, 300)
			== HAL_TIMEOUT)
	{
	};
	/*Очікую поки шина повернеться у статус готовності*/
	while (HAL_I2C_GetState(CCS811I2C) != HAL_I2C_STATE_READY)
	{
	};

}
////////////////*****///////////////////////////////////*****///////////////////////////////////*****///////////////////
void setEnvironmentalData(float relativeHumidity, float temperature)
{
	/* Функція корегування показників з використанням вологості + температури */

	int rH = relativeHumidity * 1000; //42.348 becomes 42348
	int temp = temperature * 1000; //23.2 becomes 23200

	u8 envData[4];

	//Split value into 7-bit integer and 9-bit fractional
	envData[0] =
			((rH % 1000) / 100) > 7 ? (rH / 1000 + 1) << 1 : (rH / 1000) << 1;
	envData[1] = 0; //CCS811 only supports increments of 0.5 so bits 7-0 will always be zero
	if (((rH % 1000) / 100) > 2 && (((rH % 1000) / 100) < 8))
	{
		envData[0] |= 1; //Set 9th bit of fractional to indicate 0.5%
	}

	temp += 25000; //Add the 25C offset
	//Split value into 7-bit integer and 9-bit fractional
	envData[2] =
			((temp % 1000) / 100) > 7 ?
					(temp / 1000 + 1) << 1 : (temp / 1000) << 1;
	envData[3] = 0;
	if (((temp % 1000) / 100) > 2 && (((temp % 1000) / 100) < 8))
	{
		envData[2] |= 1;  //Set 9th bit of fractional to indicate 0.5C
	}

	u8 env[6];
	env[0] = CSS811_ENV_DATA;
	env[1] = envData[0];
	env[2] = envData[1];
	env[3] = envData[2];
	env[5] = envData[3];
	HAL_I2C_Mem_Write(CCS811I2C, CCS811_ADDWR, (u8) CSS811_ENV_DATA,
	I2C_MEMADD_SIZE_8BIT, envData, 4, 100);
	/*Очікую поки шина повернеться у статус готовності*/
	while (HAL_I2C_GetState(CCS811I2C) != HAL_I2C_STATE_READY)
	{
	}
	while (HAL_I2C_IsDeviceReady(CCS811I2C, CCS_811_ADDRESS_SHIFTED, 10, 300)
			== HAL_TIMEOUT)
		;
	/*Очікую поки шина повернеться у статус готовності*/
	while (HAL_I2C_GetState(CCS811I2C) != HAL_I2C_STATE_READY)
	{
	}
}
////////////////*****///////////////////////////////////*****///////////////////////////////////*****///////////////////
void CCS811_Reset()
{
	u8 rstCMD[5] = {CSS811_SW_RESET, 0x11,0xE5,0x72,0x8A};
	CCS811_WakeUp();
	HAL_I2C_Mem_Write( CCS811I2C, CCS811_ADDWR, CSS811_SW_RESET,I2C_MEMADD_SIZE_8BIT, rstCMD, 5, 300);
	/*Очікую поки шина повернеться у статус готовності*/
	while (HAL_I2C_GetState(CCS811I2C) != HAL_I2C_STATE_READY){};
	CCS811_WakeDown();
	/* Передаю в термінал, що ресет пройшов успішно */
	//HAL_UART_Transmit(&huart1, "Reset successfull\r\n",strlen("Reset successfull\r\n"), 100);


}
////////////////*****///////////////////////////////////*****///////////////////////////////////*****///////////////////
void CCS811_Start_Application()
{
	u8 lodata[1];
    lodata[0]= CSS811_APP_START;
    CCS811_WakeUp();

    /*Записуємо
     команду переходу з режиму boot  в режим Aplication, запускаємо внутрішній АЦП шляхом передачі пустих даних
     у сенсор*/

    HAL_I2C_Master_Transmit(CCS811I2C, CCS_811_ADDRESS_SHIFTED, lodata, 1, 100);
    CCS811_WakeDown();
}
////////////////*****///////////////////////////////////*****///////////////////////////////////*****///////////////////
void CCS811_WakeUp()
{
	//Низький логічний рівень на ножці nWake пробуджує модуль CCS811
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_Delay(CCS811_WAIT_AFTER_WAKE_US);
}
////////////////*****///////////////////////////////////*****///////////////////////////////////*****///////////////////
void CCS811_WakeDown()
{
	//Високий логічний рівень на ножці nWake вимикає модуль CCS811
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
}
////////////////*****///////////////////////////////////*****///////////////////////////////////*****///////////////////
void CCS811_GET_HwID()
{
	//Hardware ID
	CCS811_WakeUp();
	u8 hwID = CCS811_Read_Register(HW_ID_REG); //Hardware ID should be 0x81
	CCS811_WakeDown();
	if (hwID != 0x81)
	{
		u_int8_t LCD[UART_BUFFER_SIZE];
		sprintf(LCD, "HID: 0x%02x\r\n", hwID);
		//HAL_UART_Transmit(&huart1, LCD, strlen(LCD), 100);
		//HAL_UART_Transmit(&huart1, "Check CCS811 wiring\r\n",strlen("Check CCS811 wiring\r\n"), 100);
		while (1)
			;
	}

	u_int8_t LCD[UART_BUFFER_SIZE];
	sprintf(LCD, "HID: 0x%02x succesfull\r\n", hwID);
	//HAL_UART_Transmit(&huart1, LCD, strlen(LCD), 100);

}
////////////////*****///////////////////////////////////*****///////////////////////////////////*****///////////////////
void CCS811_CHECK_HW_Version()
{
	// Check that HW_VERSION is 0x1X
	u8 hw_version;
	CCS811_WakeUp();
	hw_version = CCS811_Read_Register(CSS811_HW_VERSION);
	CCS811_WakeDown();

	if ((hw_version & 0xF0) != 0x10)
	{

		//HAL_UART_Transmit(&huart1, "HW_Version Wrong\r\n",strlen("HW_Version Wrong\r\n"), 100);
		while (1)
			;
	}
	else
	{
		u_int8_t LCD[UART_BUFFER_SIZE];
		sprintf(LCD, "HW Version: 0x%02x succesfull\r\n", hw_version);
		//HAL_UART_Transmit(&huart1, LCD, strlen(LCD), 100);
	}

}
////////////////*****///////////////////////////////////*****///////////////////////////////////*****///////////////////
void CCS811_CHECK_APP_VALID()
{
	/*Перевіряю статус. Після повного скидання сенсора він має бути 0x10 - режим boot*/
	u8 status; // зміннa в яку запендючим статус CCS811

	CCS811_WakeUp();
	HAL_I2C_Mem_Read(CCS811I2C, CCS_811_ADDRESS_SHIFTED, (u8) STATUS_REG, I2C_MEMADD_SIZE_8BIT, &status, 1, 100);

	while (HAL_I2C_GetState(CCS811I2C) != HAL_I2C_STATE_READY){};
	CCS811_WakeDown();
	if (status != 0x10)
	{
		u_int8_t LCD[UART_BUFFER_SIZE];
		sprintf(LCD, "Status 0x%02x Not in boot mode, or no valid app:\r\n",
				status);
		//HAL_UART_Transmit(&huart1, LCD, strlen(LCD), 100);

	}

	else
	{
		u_int8_t LCD[UART_BUFFER_SIZE];
		sprintf(LCD, "Status 0x%02x Boot mode & valid app succesfull\r\n", status);
		//HAL_UART_Transmit(&huart1, LCD, strlen(LCD), 100);
	}
}
////////////////*****///////////////////////////////////*****///////////////////////////////////*****///////////////////
void CCS811_CHECK_APP_Version()
{
	u8 app_version[2];
	CCS811_WakeUp();
	HAL_I2C_Mem_Read(CCS811I2C, CCS_811_ADDRESS_SHIFTED, 0x24,
			I2C_MEMADD_SIZE_8BIT, &app_version, 2, 100);
	CCS811_WakeDown();
	while (HAL_I2C_GetState(CCS811I2C) != HAL_I2C_STATE_READY)
		;
	_appversion = app_version[0] * 256 + app_version[1];
	u_int8_t LCD[UART_BUFFER_SIZE];
	sprintf(LCD, "CCS811_APP_Version  %d APP_Version Succesfull\r\n",
			_appversion);
	//HAL_UART_Transmit(&huart1, LCD, strlen(LCD), 100);
}
////////////////*****///////////////////////////////////*****///////////////////////////////////*****///////////////////
void CCS811_CHECK_I2C_ADDRES()
{
	/*Функція перевіряє, чи відгукується мікросхема на свою адресу. Увага. Тут використовується не зсунута на 1 біт вліво
	  адреса не 0xb4, а 0x5a*/
	if (HAL_I2C_IsDeviceReady(CCS811I2C, CCS_811_ADDRESS << 1, 10, 100)
			== HAL_OK)
	{
		u_int8_t LCD[UART_BUFFER_SIZE];
		sprintf(LCD, "Ready: 0x%02x succesfull\r\n", CCS_811_ADDRESS);
		//HAL_UART_Transmit(&huart1, LCD, strlen(LCD), 100);

	}
	else
	{
		u8 LCD[UART_BUFFER_SIZE];
		sprintf(LCD, "CCS811 Not found ERROR\r\n");
		//HAL_UART_Transmit(&huart1, LCD, strlen(LCD), 100);
		while (1)
			{};
	}
}
////////////////*****///////////////////////////////////*****///////////////////////////////////*****///////////////////
void CCS811_GET_Bootloader_Version()
{
	u8 buf[2];
	CCS811_WakeUp();
	HAL_I2C_Mem_Read(CCS811I2C, CCS_811_ADDRESS_SHIFTED,
			(u8) CCS811_FW_BOOT_VERSION, I2C_MEMADD_SIZE_8BIT, &buf, 2,
			100);
	CCS811_WakeDown();
	while (HAL_I2C_GetState(CCS811I2C) != HAL_I2C_STATE_READY)
		;

	int version;
	version = buf[0] * 256 + buf[1];
	u8 LCD[UART_BUFFER_SIZE];
	sprintf(LCD, "CCS811 Bootloader version: 0x%04x\r\n", version);
	//HAL_UART_Transmit(&huart1, LCD, strlen(LCD), 100);

}
////////////////*****///////////////////////////////////*****///////////////////////////////////*****///////////////////
void CCS811_CHECK_BOOT_MODE()
{
	u8 status; // зміннa в яку запендючим статус CCS811

	CCS811_WakeUp();
	HAL_I2C_Mem_Read(CCS811I2C, CCS_811_ADDRESS_SHIFTED, (u8) STATUS_REG,
	I2C_MEMADD_SIZE_8BIT, &status, 1, 100);
	CCS811_WakeDown();

	u_int8_t LCD[UART_BUFFER_SIZE];
	sprintf(LCD, "Status 0x%02x\r\n", status);
	//HAL_UART_Transmit(&huart1, LCD, strlen(LCD), 100);
	while (HAL_I2C_GetState(CCS811I2C) != HAL_I2C_STATE_READY)
		;

	if (&status != 0x90)
	{
		u_int8_t LCD[UART_BUFFER_SIZE];
		sprintf(LCD,
				"Status 0x%02x ccs811: Not in app mode, or no valid app:\r\n",
				status);
		//HAL_UART_Transmit(&huart1, LCD, strlen(LCD), 100);

	}

	else
	{
		u_int8_t LCD[UART_BUFFER_SIZE];
		sprintf(LCD, "Status 0x%02x APP mode succesfull\r\n", status);
		//HAL_UART_Transmit(&huart1, LCD, strlen(LCD), 100);
	}
}
////////////////*****///////////////////////////////////*****///////////////////////////////////*****///////////////////
void CCS811_CHECK_READY_DATA()
{
		u8 status; // зміннa в яку запендючим статус CCS811

		CCS811_WakeUp();
		HAL_I2C_Mem_Read(CCS811I2C, CCS_811_ADDRESS_SHIFTED, (u8) STATUS_REG,
		I2C_MEMADD_SIZE_8BIT, &status, 1, 100);
		CCS811_WakeDown();

		if (status!=0x08)
		{
			u_int8_t LCD[UART_BUFFER_SIZE];
			sprintf(LCD,
					"Status 0x%02x ccs811: Data not ready\r\n",
					status);
			//HAL_UART_Transmit(&huart1, LCD, strlen(LCD), 100);

		}

		else
		{
			u_int8_t LCD[UART_BUFFER_SIZE];
			sprintf(LCD, "Status 0x%02x Data is ready succesfull\r\n", status);
			//HAL_UART_Transmit(&huart1, LCD, strlen(LCD), 100);
		}
	}
////////////////*****///////////////////////////////////*****///////////////////////////////////*****///////////////////
u8 CCS811_GET_StatusRegister()
{
	// Check status (after reset, CCS811 should be in boot mode with valid app)
	u8 status; // зміннa в яку запендючим статус CCS811
	CCS811_WakeUp();
	HAL_I2C_Mem_Read(CCS811I2C, CCS_811_ADDRESS_SHIFTED, (u8) STATUS_REG,
	I2C_MEMADD_SIZE_8BIT, &status, 1, 100);
	while (HAL_I2C_GetState(CCS811I2C) != HAL_I2C_STATE_READY){};
	CCS811_WakeDown();
	return status;
}
////////////////*****///////////////////////////////////*****///////////////////////////////////*****///////////////////
void CCS811_Get_Result()
{
	    u8 data_rq[4];
		/* Очікуємо, поки у регістрі статусу буде значення 0x98, це ознака того, що дані підготовлені для відправики */
		 u8 state = 0;
		 while(state != 0x98)
		 {
		    state = CCS811_GET_StatusRegister();
		 }

		CCS811_WakeUp();
		HAL_I2C_Mem_Read( CCS811I2C, CCS_811_ADDRESS_SHIFTED, (u8)CSS811_ALG_RESULT_DATA, I2C_MEMADD_SIZE_8BIT, data_rq, 4,100 );
        CCS811_WakeDown();

        /* Зчитуємо по 2 пів-байта для CO2 & VOC */

		u8 co2MSB = data_rq[0];
		u8 co2LSB = data_rq[1];
		u8 tvocMSB = data_rq[2];
		u8 tvocLSB = data_rq[3];

	/*	TVOC value, in parts per billion (ppb)
		eC02 value, in parts per million (ppm) */
		CO2 = ((unsigned int)co2MSB << 8) | co2LSB;
		tVOC = ((unsigned int)tvocMSB << 8) | tvocLSB;

		u8 LCD[UART_BUFFER_SIZE];
	    sprintf(LCD, "VOC: %-4d", tVOC);
	    ssd1306_SetCursor(0, 0);
	    ssd1306_WriteString(LCD, Font_11x18);
	    ssd1306_UpdateScreen();

	    sprintf(LCD, "CO2: %-4d", CO2);
	    ssd1306_SetCursor(0, 25);
	    ssd1306_WriteString(LCD, Font_11x18);
	    ssd1306_UpdateScreen();

}
////////////////*****///////////////////////////////////*****///////////////////////////////////*****///////////////////
void CCS811_DisableMaesurement()
{


}
void CCS811_GetSensorResistance()
{
	uint8_t sensor_Resistance_raw[2];
	uint32_t sensor_Resistance;
	CCS811_WakeUp();
	HAL_I2C_Mem_Read(CCS811I2C, CCS_811_ADDRESS_SHIFTED, CSS811_RAW_DATA , I2C_MEMADD_SIZE_8BIT, sensor_Resistance_raw, 2,100 );
    CCS811_WakeDown();
	current_value=sensor_Resistance_raw[0]>>2;
	sensor_Resistance_raw[0]=sensor_Resistance_raw[0]&0x03;
	adc_raw=(sensor_Resistance_raw[0]<<8)|sensor_Resistance_raw[1];
	sensor_Resistance=((165*adc_raw)*10000)/(current_value*1023);

	u_int8_t LCD[UART_BUFFER_SIZE];
	sprintf(LCD, "CCS811 resistanse: %i\r\n", sensor_Resistance);
	//HAL_UART_Transmit(&huart1, LCD, strlen(LCD), 100);

	//return sensor_Resistance;
}
////////////////*****///////////////////////////////////*****///////////////////////////////////*****///////////////////

void CCS811_Enable_TreshholdInterrupt()
{
	typedef struct
		__packed
		{
			u16 thresh_low;
			u16 thresh_high;
			u8 thresh_hyst;
		} threshold_reg;

		threshold_reg thresh_reg ={120,254,50};

		u8 temp_ptr;
		temp_ptr =(uint8_t*) &thresh_reg.thresh_low;

		CCS811_WakeUp();
		HAL_I2C_Mem_Write(CCS811I2C, CCS_811_ADDRESS_SHIFTED, (u8)THRESHOLDS,I2C_MEMADD_SIZE_8BIT, &temp_ptr, 5, 300);
		/*Очікую поки шина повернеться у статус готовності*/
		while (HAL_I2C_GetState(CCS811I2C) != HAL_I2C_STATE_READY)
		{
		}

		while (HAL_I2C_IsDeviceReady(CCS811I2C, CCS_811_ADDRESS_SHIFTED, 10,
				300) == HAL_TIMEOUT)
		{
		};
		/*Очікую поки шина повернеться у статус готовності*/
		while (HAL_I2C_GetState(CCS811I2C) != HAL_I2C_STATE_READY)
		{
		};
        CCS811_WakeDown();
}
