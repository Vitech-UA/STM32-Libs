################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/system_stm32f0xx.c 

CPP_SRCS += \
../Src/main.cpp 

C_DEPS += \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/system_stm32f0xx.d 

OBJS += \
./Src/main.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/system_stm32f0xx.o 

CPP_DEPS += \
./Src/main.d 


# Each subdirectory must supply rules for building sources it contributes
Src/main.o: ../Src/main.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0 -std=gnu++14 -g3 -DSTM32 -DSTM32F0 -DDEBUG -DSTM32F051R8Tx -c -I../Inc -I"C:/STM32CubeIDE_workspace_1.0.2/STM32F051_RFM69_Cpp/RFM69" -I"C:/STM32CubeIDE_workspace_1.0.2/STM32F051_RFM69_Cpp/uart" -I"C:/STM32CubeIDE_workspace_1.0.2/STM32F051_RFM69_Cpp/spi" -I"C:/STM32CubeIDE_workspace_1.0.2/STM32F051_RFM69_Cpp/gpio" -include"C:/STM32CubeIDE_workspace_1.0.2/STM32F051_RFM69_Cpp/gpio/Gpio.cpp" -include"C:/STM32CubeIDE_workspace_1.0.2/STM32F051_RFM69_Cpp/RFM69/RFM69.cpp" -include"C:/STM32CubeIDE_workspace_1.0.2/STM32F051_RFM69_Cpp/uart/uart.cpp" -include"C:/STM32CubeIDE_workspace_1.0.2/STM32F051_RFM69_Cpp/spi/SPI.cpp" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Src/main.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/syscalls.o: ../Src/syscalls.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DSTM32 -DSTM32F0 -DDEBUG -DSTM32F051R8Tx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/syscalls.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/sysmem.o: ../Src/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DSTM32 -DSTM32F0 -DDEBUG -DSTM32F051R8Tx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/sysmem.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/system_stm32f0xx.o: ../Src/system_stm32f0xx.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DSTM32 -DSTM32F0 -DDEBUG -DSTM32F051R8Tx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/system_stm32f0xx.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

