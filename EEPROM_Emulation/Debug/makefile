################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F0xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
EEPROM_Emulation.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
EEPROM_Emulation.list \

OBJCOPY_BIN += \
EEPROM_Emulation.bin \


# All Target
all: EEPROM_Emulation.elf secondary-outputs

# Tool invocations
EEPROM_Emulation.elf: $(OBJS) $(USER_OBJS) /home/jerryl/Git/Github_Public/STM32F030F4P6-Core-Board-STM32CubeIDE/EEPROM_Emulation/STM32F030F4PX_FLASH.ld
	arm-none-eabi-gcc -o "EEPROM_Emulation.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"/home/jerryl/Git/Github_Public/STM32F030F4P6-Core-Board-STM32CubeIDE/EEPROM_Emulation/STM32F030F4PX_FLASH.ld" --specs=nosys.specs -Wl,-Map="EEPROM_Emulation.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

EEPROM_Emulation.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "EEPROM_Emulation.list"
	@echo 'Finished building: $@'
	@echo ' '

EEPROM_Emulation.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "EEPROM_Emulation.bin"
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
