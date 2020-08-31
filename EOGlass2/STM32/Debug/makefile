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
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
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
EOGlass2.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
EOGlass2.list \

OBJCOPY_BIN += \
EOGlass2.bin \


# All Target
all: EOGlass2.elf secondary-outputs

# Tool invocations
EOGlass2.elf: $(OBJS) $(USER_OBJS) /Users/matthollands/Library/Mobile\ Documents/com~apple~CloudDocs/PersonalProjects/EOGee/Github/EOGee/EOGlass2/STM32/STM32L412KBUX_FLASH.ld
	arm-none-eabi-gcc -o "EOGlass2.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/Users/matthollands/Library/Mobile Documents/com~apple~CloudDocs/PersonalProjects/EOGee/Github/EOGee/EOGlass2/STM32/STM32L412KBUX_FLASH.ld" --specs=nosys.specs -Wl,-Map="EOGlass2.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

EOGlass2.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "EOGlass2.list"
	@echo 'Finished building: $@'
	@echo ' '

EOGlass2.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "EOGlass2.bin"
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
