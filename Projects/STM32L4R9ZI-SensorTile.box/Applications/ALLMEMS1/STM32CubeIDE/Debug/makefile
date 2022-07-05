################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include STM32L4R9ZI-SensorTile.box_ALLMEMS1/User/subdir.mk
-include STM32L4R9ZI-SensorTile.box_ALLMEMS1/Startup/subdir.mk
-include Middlewares/json/subdir.mk
-include Middlewares/STM32_USBD_Library/Core/subdir.mk
-include Middlewares/STM32_USBD_Library/Class/CDC/subdir.mk
-include Middlewares/MetaDataManager/subdir.mk
-include Middlewares/BlueNRG-2/Utils/subdir.mk
-include Middlewares/BlueNRG-2/HCI_TL/Basic/subdir.mk
-include Middlewares/BlueNRG-2/Controller/subdir.mk
-include Middlewares/BLE_Manager/subdir.mk
-include Drivers/CMSIS/subdir.mk
-include Drivers/BSP/SensorTile.box/subdir.mk
-include Drivers/BSP/STM32L4xx_HAL_Driver/subdir.mk
-include Drivers/BSP/Components/subdir.mk
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


BUILD_ARTIFACT_NAME := STM32L4R9ZI-SensorTileBox_ALLMEMS1
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
STM32L4R9ZI-SensorTileBox_ALLMEMS1.elf \

MAP_FILES += \
STM32L4R9ZI-SensorTileBox_ALLMEMS1.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
STM32L4R9ZI-SensorTileBox_ALLMEMS1.list \


# All Target
all: main-build

# Main-build Target
main-build: STM32L4R9ZI-SensorTileBox_ALLMEMS1.elf secondary-outputs

# Tool invocations
STM32L4R9ZI-SensorTileBox_ALLMEMS1.elf STM32L4R9ZI-SensorTileBox_ALLMEMS1.map: $(OBJS) $(USER_OBJS) C:\MyFolder\01_GitRepoProject\01_Release\ Candidate\02_GitHub\STM32CubeFunctionPack_ALLMEMS1_V4.2.0\Projects\STM32L4R9ZI-SensorTile.box\Applications\ALLMEMS1\STM32CubeIDE\STM32L4R9AIIX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "STM32L4R9ZI-SensorTileBox_ALLMEMS1.elf" @"objects.list" $(USER_OBJS) -Wl,--start-group $(LIBS) -Wl,--end-group -mcpu=cortex-m4 -T"C:\MyFolder\01_GitRepoProject\01_Release Candidate\02_GitHub\STM32CubeFunctionPack_ALLMEMS1_V4.2.0\Projects\STM32L4R9ZI-SensorTile.box\Applications\ALLMEMS1\STM32CubeIDE\STM32L4R9AIIX_FLASH.ld" --specs=nosys.specs -Wl,-Map="STM32L4R9ZI-SensorTileBox_ALLMEMS1.map" -Wl,--gc-sections -static -L../../../../../../Middlewares/ST/STM32_MotionAR_Library/Lib -L../../../../../../Middlewares/ST/STM32_MotionCP_Library/Lib -L../../../../../../Middlewares/ST/STM32_MotionFA_Library/Lib -L../../../../../../Middlewares/ST/STM32_MotionFX_Library/Lib -L../../../../../../Middlewares/ST/STM32_MotionGR_Library/Lib -L../../../../../../Middlewares/ST/STM32_MotionID_Library/Lib -L../../../../../../Middlewares/ST/STM32_MotionPE_Library/Lib -L../../../../../../Middlewares/ST/STM32_MotionSD_Library/Lib -L../../../../../../Middlewares/ST/STM32_MotionTL_Library/Lib -L../../../../../../Middlewares/ST/STM32_MotionVC_Library/Lib --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

STM32L4R9ZI-SensorTileBox_ALLMEMS1.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "STM32L4R9ZI-SensorTileBox_ALLMEMS1.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) STM32L4R9ZI-SensorTileBox_ALLMEMS1.elf STM32L4R9ZI-SensorTileBox_ALLMEMS1.list STM32L4R9ZI-SensorTileBox_ALLMEMS1.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
