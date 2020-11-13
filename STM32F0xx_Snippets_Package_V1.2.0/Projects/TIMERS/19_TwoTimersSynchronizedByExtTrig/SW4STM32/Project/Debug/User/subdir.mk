################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/STM32F0xx_Snippets/Projects/TIMERS/19_TwoTimersSynchronizedByExtTrig/main.c \
C:/STM32F0xx_Snippets/Projects/TIMERS/19_TwoTimersSynchronizedByExtTrig/system_stm32f0xx.c 

OBJS += \
./User/main.o \
./User/system_stm32f0xx.o 

C_DEPS += \
./User/main.d \
./User/system_stm32f0xx.d 


# Each subdirectory must supply rules for building sources it contributes
User/main.o: C:/STM32F0xx_Snippets/Projects/TIMERS/19_TwoTimersSynchronizedByExtTrig/main.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -DSTM32F072xB -I..\..\..\ -I..\..\..\..\..\..\Libraries\CMSIS\Include -I..\..\..\..\..\..\Libraries\CMSIS\Device\ST\STM32F0xx\Include -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

User/system_stm32f0xx.o: C:/STM32F0xx_Snippets/Projects/TIMERS/19_TwoTimersSynchronizedByExtTrig/system_stm32f0xx.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -DSTM32F072xB -I..\..\..\ -I..\..\..\..\..\..\Libraries\CMSIS\Include -I..\..\..\..\..\..\Libraries\CMSIS\Device\ST\STM32F0xx\Include -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


