################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../STM32L4R9ZI-SensorTile.box_ALLMEMS1/Startup/startup_stm32l4r9aiix.s 

OBJS += \
./STM32L4R9ZI-SensorTile.box_ALLMEMS1/Startup/startup_stm32l4r9aiix.o 

S_DEPS += \
./STM32L4R9ZI-SensorTile.box_ALLMEMS1/Startup/startup_stm32l4r9aiix.d 


# Each subdirectory must supply rules for building sources it contributes
STM32L4R9ZI-SensorTile.box_ALLMEMS1/Startup/%.o: ../STM32L4R9ZI-SensorTile.box_ALLMEMS1/Startup/%.s STM32L4R9ZI-SensorTile.box_ALLMEMS1/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I../../Inc -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@" "$<"

clean: clean-STM32L4R9ZI-2d-SensorTile-2e-box_ALLMEMS1-2f-Startup

clean-STM32L4R9ZI-2d-SensorTile-2e-box_ALLMEMS1-2f-Startup:
	-$(RM) ./STM32L4R9ZI-SensorTile.box_ALLMEMS1/Startup/startup_stm32l4r9aiix.d ./STM32L4R9ZI-SensorTile.box_ALLMEMS1/Startup/startup_stm32l4r9aiix.o

.PHONY: clean-STM32L4R9ZI-2d-SensorTile-2e-box_ALLMEMS1-2f-Startup

