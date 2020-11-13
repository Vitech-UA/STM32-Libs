################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
C:/STM32F0xx_Snippets/Libraries/CMSIS/Device/ST/STM32F0xx/Source/Templates/gcc/startup_stm32f072xb.s 

OBJS += \
./SW4STM32/startup_stm32f072xb.o 


# Each subdirectory must supply rules for building sources it contributes
SW4STM32/startup_stm32f072xb.o: C:/STM32F0xx_Snippets/Libraries/CMSIS/Device/ST/STM32F0xx/Source/Templates/gcc/startup_stm32f072xb.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	arm-none-eabi-as -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


