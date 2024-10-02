################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := 006_Staj_STM32F103C6_MODBUS_SIM_RXCARD
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
006_Staj_STM32F103C6_MODBUS_SIM_RXCARD.elf \

MAP_FILES += \
006_Staj_STM32F103C6_MODBUS_SIM_RXCARD.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
006_Staj_STM32F103C6_MODBUS_SIM_RXCARD.list \

OBJCOPY_HEX += \
006_Staj_STM32F103C6_MODBUS_SIM_RXCARD.hex \

OBJCOPY_BIN += \
006_Staj_STM32F103C6_MODBUS_SIM_RXCARD.bin \


# All Target
all: main-build

# Main-build Target
main-build: 006_Staj_STM32F103C6_MODBUS_SIM_RXCARD.elf secondary-outputs

# Tool invocations
006_Staj_STM32F103C6_MODBUS_SIM_RXCARD.elf 006_Staj_STM32F103C6_MODBUS_SIM_RXCARD.map: $(OBJS) $(USER_OBJS) C:\Users\koruc\STM32CubeIDE\workspace_1.16.0\Staj_STM32F103C6_MODBUS_SIM_RXCARD_006\STM32F103C6TX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "006_Staj_STM32F103C6_MODBUS_SIM_RXCARD.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\koruc\STM32CubeIDE\workspace_1.16.0\Staj_STM32F103C6_MODBUS_SIM_RXCARD_006\STM32F103C6TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="006_Staj_STM32F103C6_MODBUS_SIM_RXCARD.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

006_Staj_STM32F103C6_MODBUS_SIM_RXCARD.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "006_Staj_STM32F103C6_MODBUS_SIM_RXCARD.list"
	@echo 'Finished building: $@'
	@echo ' '

006_Staj_STM32F103C6_MODBUS_SIM_RXCARD.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "006_Staj_STM32F103C6_MODBUS_SIM_RXCARD.hex"
	@echo 'Finished building: $@'
	@echo ' '

006_Staj_STM32F103C6_MODBUS_SIM_RXCARD.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "006_Staj_STM32F103C6_MODBUS_SIM_RXCARD.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) 006_Staj_STM32F103C6_MODBUS_SIM_RXCARD.bin 006_Staj_STM32F103C6_MODBUS_SIM_RXCARD.elf 006_Staj_STM32F103C6_MODBUS_SIM_RXCARD.hex 006_Staj_STM32F103C6_MODBUS_SIM_RXCARD.list 006_Staj_STM32F103C6_MODBUS_SIM_RXCARD.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
