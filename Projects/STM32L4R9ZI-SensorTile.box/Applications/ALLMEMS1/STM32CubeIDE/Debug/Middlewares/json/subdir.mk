################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/MyFolder/01_GitRepoProject/01_Release\ Candidate/02_GitHub/STM32CubeFunctionPack_ALLMEMS1_V4.2.0/Middlewares/Third_Party/parson/parson.c 

OBJS += \
./Middlewares/json/parson.o 

C_DEPS += \
./Middlewares/json/parson.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/json/parson.o: C:/MyFolder/01_GitRepoProject/01_Release\ Candidate/02_GitHub/STM32CubeFunctionPack_ALLMEMS1_V4.2.0/Middlewares/Third_Party/parson/parson.c Middlewares/json/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L4R9xx -DSTM32_SENSORTILEBOX -DUSE_HAL_DRIVER -c -I../../Inc -I../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../Drivers/BSP/Components/Common -I../../../../../../Drivers/BSP/SensorTile.box -I../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../Drivers/BSP/Components/lis2mdl -I../../../../../../Drivers/BSP/Components/lps22hh -I../../../../../../Drivers/BSP/Components/lsm6dsox -I../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../Middlewares/ST/STM32_MetaDataManager -I../../../../../../Middlewares/ST/BlueNRG-2/includes -I../../../../../../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../../../../../../Middlewares/ST/BlueNRG-2/utils -I../../../../../../Middlewares/ST/STM32_MotionAR_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionCP_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionGR_Library/Inc -I../../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../../Middlewares/ST/STM32_MotionFA_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionID_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionPE_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionSD_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionTL_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionVC_Library/Inc -I../../../../../../Middlewares/ST/STM32_BLE_Manager/Inc -I../../../../../../Middlewares/Third_Party/parson -I../../../../../../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/json/parson.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"

clean: clean-Middlewares-2f-json

clean-Middlewares-2f-json:
	-$(RM) ./Middlewares/json/parson.d ./Middlewares/json/parson.o ./Middlewares/json/parson.su

.PHONY: clean-Middlewares-2f-json

