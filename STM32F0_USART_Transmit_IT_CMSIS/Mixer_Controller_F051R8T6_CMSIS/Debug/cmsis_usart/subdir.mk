################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../cmsis_usart/cmsis_usart.c 

OBJS += \
./cmsis_usart/cmsis_usart.o 

C_DEPS += \
./cmsis_usart/cmsis_usart.d 


# Each subdirectory must supply rules for building sources it contributes
cmsis_usart/cmsis_usart.o: ../cmsis_usart/cmsis_usart.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DSTM32 -DSTM32F0 -DDEBUG -DSTM32F051R8Tx -c -I../Inc -I"C:/STM32_Workspace/Mixer_Controller_F051R8T6_CMSIS/CMSIS/inc" -I"C:/STM32_Workspace/Mixer_Controller_F051R8T6_CMSIS/CMSIS/src" -I"C:/STM32_Workspace/Mixer_Controller_F051R8T6_CMSIS/cmsis_usart" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"cmsis_usart/cmsis_usart.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

