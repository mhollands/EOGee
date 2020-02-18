################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include USB_DEVICE/Target/subdir.mk
-include USB_DEVICE/App/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Core/Src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
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
EOGee.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
EOGee.list \

OBJCOPY_BIN += \
EOGee.bin \


# All Target
all: EOGee.elf secondary-outputs

# Tool invocations
EOGee.elf: $(OBJS) $(USER_OBJS) /Users/matthollands/Library/Mobile\ Documents/com~apple~CloudDocs/PersonalProjects/EOGee/Github/EOGee/STM32/EOGee/STM32F042F4PX_FLASH.ld
	arm-none-eabi-gcc -o "EOGee.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"/Users/matthollands/Library/Mobile Documents/com~apple~CloudDocs/PersonalProjects/EOGee/Github/EOGee/STM32/EOGee/STM32F042F4PX_FLASH.ld" --specs=nosys.specs -Wl,-Map="EOGee.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

EOGee.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "EOGee.list"
	@echo 'Finished building: $@'
	@echo ' '

EOGee.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "EOGee.bin"
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
