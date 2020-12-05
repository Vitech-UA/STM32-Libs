################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include cmsis_usart/subdir.mk
-include Startup/subdir.mk
-include Src/subdir.mk
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
Mixer_Controller_F051R8T6_CMSIS.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Mixer_Controller_F051R8T6_CMSIS.list \

OBJCOPY_BIN += \
Mixer_Controller_F051R8T6_CMSIS.bin \


# All Target
all: Mixer_Controller_F051R8T6_CMSIS.elf secondary-outputs

# Tool invocations
Mixer_Controller_F051R8T6_CMSIS.elf: $(OBJS) $(USER_OBJS) C:\STM32_Workspace\Mixer_Controller_F051R8T6_CMSIS\STM32F051R8TX_FLASH.ld
	arm-none-eabi-gcc -o "Mixer_Controller_F051R8T6_CMSIS.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"C:\STM32_Workspace\Mixer_Controller_F051R8T6_CMSIS\STM32F051R8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Mixer_Controller_F051R8T6_CMSIS.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Mixer_Controller_F051R8T6_CMSIS.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Mixer_Controller_F051R8T6_CMSIS.list"
	@echo 'Finished building: $@'
	@echo ' '

Mixer_Controller_F051R8T6_CMSIS.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Mixer_Controller_F051R8T6_CMSIS.bin"
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
