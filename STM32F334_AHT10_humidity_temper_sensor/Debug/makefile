################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F3xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
STM32F334_AHT10_humidity_temper_sensor.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
STM32F334_AHT10_humidity_temper_sensor.list \

OBJCOPY_BIN += \
STM32F334_AHT10_humidity_temper_sensor.bin \


# All Target
all: STM32F334_AHT10_humidity_temper_sensor.elf secondary-outputs

# Tool invocations
STM32F334_AHT10_humidity_temper_sensor.elf: $(OBJS) $(USER_OBJS) C:\STM32CubeIDE_workspace_1.0.2\STM32F334_AHT10_humidity_temper_sensor\STM32F334R8TX_FLASH.ld
	arm-none-eabi-gcc -o "STM32F334_AHT10_humidity_temper_sensor.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\STM32CubeIDE_workspace_1.0.2\STM32F334_AHT10_humidity_temper_sensor\STM32F334R8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="STM32F334_AHT10_humidity_temper_sensor.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

STM32F334_AHT10_humidity_temper_sensor.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "STM32F334_AHT10_humidity_temper_sensor.list"
	@echo 'Finished building: $@'
	@echo ' '

STM32F334_AHT10_humidity_temper_sensor.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "STM32F334_AHT10_humidity_temper_sensor.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
