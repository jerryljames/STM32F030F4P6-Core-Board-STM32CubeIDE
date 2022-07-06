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
BTS7960_Test.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
BTS7960_Test.list \

OBJCOPY_BIN += \
BTS7960_Test.bin \


# All Target
all: BTS7960_Test.elf secondary-outputs

# Tool invocations
BTS7960_Test.elf: $(OBJS) $(USER_OBJS) /home/jerryl/Git/Github_Public/STM32F030F4P6-Core-Board-STM32CubeIDE/BTS7960_Test/STM32F030F4PX_FLASH.ld
	arm-none-eabi-gcc -o "BTS7960_Test.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"/home/jerryl/Git/Github_Public/STM32F030F4P6-Core-Board-STM32CubeIDE/BTS7960_Test/STM32F030F4PX_FLASH.ld" --specs=nosys.specs -Wl,-Map="BTS7960_Test.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

BTS7960_Test.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "BTS7960_Test.list"
	@echo 'Finished building: $@'
	@echo ' '

BTS7960_Test.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "BTS7960_Test.bin"
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
