################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/MT48LC4M32B2/MT48LC4M32B2.c 

OBJS += \
./Core/MT48LC4M32B2/MT48LC4M32B2.o 

C_DEPS += \
./Core/MT48LC4M32B2/MT48LC4M32B2.d 


# Each subdirectory must supply rules for building sources it contributes
Core/MT48LC4M32B2/MT48LC4M32B2.o: ../Core/MT48LC4M32B2/MT48LC4M32B2.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F769xx -DDEBUG -c -I"C:/CubeIDE_workspace/STM32F769NiT6_SDRAM_MT48LC4M32B2B5-6A/Core/MT48LC4M32B2" -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Core/Inc -I"C:/CubeIDE_workspace/STM32F769NiT6_SDRAM_MT48LC4M32B2B5-6A/Core/ST7789" -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/MT48LC4M32B2/MT48LC4M32B2.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

