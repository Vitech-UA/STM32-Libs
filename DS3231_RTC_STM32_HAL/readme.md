![ds3231](https://user-images.githubusercontent.com/74230330/153068347-2275d7ca-9aac-4a5b-97d8-07ff6ca37ebe.jpg)
![IRQ_MODE](https://user-images.githubusercontent.com/74230330/153068350-819fe489-ad7c-4d2c-8da1-f469eec76e8c.jpg)

STM32CubeMX setup
In STM32CubeMX, set I2C1 to "I2C" and USART1 to "asynchronous"  
Set up an external interrupt pin (say PB0) in GPIO settings, use "external interrupt mode with falling edge trigger detection" and "pull-up" settings.  
Activate the external interrupt in NVIC settings by checking the corresponding box.  
Connect pin 3 (INT#/SQW) of the DS3231 to this external interrupt pin.  
Save and generate code.  
