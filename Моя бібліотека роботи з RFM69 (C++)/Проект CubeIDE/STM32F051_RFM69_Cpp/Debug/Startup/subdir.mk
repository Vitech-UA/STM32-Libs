################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Startup/startup_stm32f051r8tx.s 

S_DEPS += \
./Startup/startup_stm32f051r8tx.d 

OBJS += \
./Startup/startup_stm32f051r8tx.o 


# Each subdirectory must supply rules for building sources it contributes
Startup/startup_stm32f051r8tx.o: ../Startup/startup_stm32f051r8tx.s
	arm-none-eabi-gcc -mcpu=cortex-m0 -g3 -c -x assembler-with-cpp -MMD -MP -MF"Startup/startup_stm32f051r8tx.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

