################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32G4xx_HAL_Driver/Src/subdir.mk
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


BUILD_ARTIFACT_NAME := 004_Staj_Usart_Input_Haberlesme2_Edit
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
004_Staj_Usart_Input_Haberlesme2_Edit.elf \

MAP_FILES += \
004_Staj_Usart_Input_Haberlesme2_Edit.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
004_Staj_Usart_Input_Haberlesme2_Edit.list \


# All Target
all: main-build

# Main-build Target
main-build: 004_Staj_Usart_Input_Haberlesme2_Edit.elf secondary-outputs

# Tool invocations
004_Staj_Usart_Input_Haberlesme2_Edit.elf 004_Staj_Usart_Input_Haberlesme2_Edit.map: $(OBJS) $(USER_OBJS) C:\Users\koruc\STM32CubeIDE\workspace_1.16.0\004_Staj_Usart_Input_Haberlesme2_Edit\STM32G474RETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "004_Staj_Usart_Input_Haberlesme2_Edit.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\koruc\STM32CubeIDE\workspace_1.16.0\004_Staj_Usart_Input_Haberlesme2_Edit\STM32G474RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="004_Staj_Usart_Input_Haberlesme2_Edit.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

004_Staj_Usart_Input_Haberlesme2_Edit.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "004_Staj_Usart_Input_Haberlesme2_Edit.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) 004_Staj_Usart_Input_Haberlesme2_Edit.elf 004_Staj_Usart_Input_Haberlesme2_Edit.list 004_Staj_Usart_Input_Haberlesme2_Edit.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
