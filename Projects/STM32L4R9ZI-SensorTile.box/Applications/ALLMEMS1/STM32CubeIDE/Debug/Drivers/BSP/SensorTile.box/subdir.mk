################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/MyFolder/01_GitRepoProject/01_Release\ Candidate/02_GitHub/STM32CubeFunctionPack_ALLMEMS1_V4.2.0/Drivers/BSP/SensorTile.box/SensorTile.box.c \
C:/MyFolder/01_GitRepoProject/01_Release\ Candidate/02_GitHub/STM32CubeFunctionPack_ALLMEMS1_V4.2.0/Drivers/BSP/SensorTile.box/SensorTile.box_audio.c \
C:/MyFolder/01_GitRepoProject/01_Release\ Candidate/02_GitHub/STM32CubeFunctionPack_ALLMEMS1_V4.2.0/Drivers/BSP/SensorTile.box/SensorTile.box_bc.c \
C:/MyFolder/01_GitRepoProject/01_Release\ Candidate/02_GitHub/STM32CubeFunctionPack_ALLMEMS1_V4.2.0/Drivers/BSP/SensorTile.box/SensorTile.box_bus.c \
C:/MyFolder/01_GitRepoProject/01_Release\ Candidate/02_GitHub/STM32CubeFunctionPack_ALLMEMS1_V4.2.0/Drivers/BSP/SensorTile.box/SensorTile.box_env_sensors.c \
C:/MyFolder/01_GitRepoProject/01_Release\ Candidate/02_GitHub/STM32CubeFunctionPack_ALLMEMS1_V4.2.0/Drivers/BSP/SensorTile.box/SensorTile.box_env_sensors_ex.c \
C:/MyFolder/01_GitRepoProject/01_Release\ Candidate/02_GitHub/STM32CubeFunctionPack_ALLMEMS1_V4.2.0/Drivers/BSP/SensorTile.box/SensorTile.box_motion_sensors.c \
C:/MyFolder/01_GitRepoProject/01_Release\ Candidate/02_GitHub/STM32CubeFunctionPack_ALLMEMS1_V4.2.0/Drivers/BSP/SensorTile.box/SensorTile.box_motion_sensors_ex.c 

OBJS += \
./Drivers/BSP/SensorTile.box/SensorTile.box.o \
./Drivers/BSP/SensorTile.box/SensorTile.box_audio.o \
./Drivers/BSP/SensorTile.box/SensorTile.box_bc.o \
./Drivers/BSP/SensorTile.box/SensorTile.box_bus.o \
./Drivers/BSP/SensorTile.box/SensorTile.box_env_sensors.o \
./Drivers/BSP/SensorTile.box/SensorTile.box_env_sensors_ex.o \
./Drivers/BSP/SensorTile.box/SensorTile.box_motion_sensors.o \
./Drivers/BSP/SensorTile.box/SensorTile.box_motion_sensors_ex.o 

C_DEPS += \
./Drivers/BSP/SensorTile.box/SensorTile.box.d \
./Drivers/BSP/SensorTile.box/SensorTile.box_audio.d \
./Drivers/BSP/SensorTile.box/SensorTile.box_bc.d \
./Drivers/BSP/SensorTile.box/SensorTile.box_bus.d \
./Drivers/BSP/SensorTile.box/SensorTile.box_env_sensors.d \
./Drivers/BSP/SensorTile.box/SensorTile.box_env_sensors_ex.d \
./Drivers/BSP/SensorTile.box/SensorTile.box_motion_sensors.d \
./Drivers/BSP/SensorTile.box/SensorTile.box_motion_sensors_ex.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/SensorTile.box/SensorTile.box.o: C:/MyFolder/01_GitRepoProject/01_Release\ Candidate/02_GitHub/STM32CubeFunctionPack_ALLMEMS1_V4.2.0/Drivers/BSP/SensorTile.box/SensorTile.box.c Drivers/BSP/SensorTile.box/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L4R9xx -DSTM32_SENSORTILEBOX -DUSE_HAL_DRIVER -c -I../../Inc -I../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../Drivers/BSP/Components/Common -I../../../../../../Drivers/BSP/SensorTile.box -I../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../Drivers/BSP/Components/lis2mdl -I../../../../../../Drivers/BSP/Components/lps22hh -I../../../../../../Drivers/BSP/Components/lsm6dsox -I../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../Middlewares/ST/STM32_MetaDataManager -I../../../../../../Middlewares/ST/BlueNRG-2/includes -I../../../../../../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../../../../../../Middlewares/ST/BlueNRG-2/utils -I../../../../../../Middlewares/ST/STM32_MotionAR_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionCP_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionGR_Library/Inc -I../../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../../Middlewares/ST/STM32_MotionFA_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionID_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionPE_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionSD_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionTL_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionVC_Library/Inc -I../../../../../../Middlewares/ST/STM32_BLE_Manager/Inc -I../../../../../../Middlewares/Third_Party/parson -I../../../../../../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/SensorTile.box/SensorTile.box.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Drivers/BSP/SensorTile.box/SensorTile.box_audio.o: C:/MyFolder/01_GitRepoProject/01_Release\ Candidate/02_GitHub/STM32CubeFunctionPack_ALLMEMS1_V4.2.0/Drivers/BSP/SensorTile.box/SensorTile.box_audio.c Drivers/BSP/SensorTile.box/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L4R9xx -DSTM32_SENSORTILEBOX -DUSE_HAL_DRIVER -c -I../../Inc -I../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../Drivers/BSP/Components/Common -I../../../../../../Drivers/BSP/SensorTile.box -I../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../Drivers/BSP/Components/lis2mdl -I../../../../../../Drivers/BSP/Components/lps22hh -I../../../../../../Drivers/BSP/Components/lsm6dsox -I../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../Middlewares/ST/STM32_MetaDataManager -I../../../../../../Middlewares/ST/BlueNRG-2/includes -I../../../../../../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../../../../../../Middlewares/ST/BlueNRG-2/utils -I../../../../../../Middlewares/ST/STM32_MotionAR_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionCP_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionGR_Library/Inc -I../../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../../Middlewares/ST/STM32_MotionFA_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionID_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionPE_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionSD_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionTL_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionVC_Library/Inc -I../../../../../../Middlewares/ST/STM32_BLE_Manager/Inc -I../../../../../../Middlewares/Third_Party/parson -I../../../../../../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/SensorTile.box/SensorTile.box_audio.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Drivers/BSP/SensorTile.box/SensorTile.box_bc.o: C:/MyFolder/01_GitRepoProject/01_Release\ Candidate/02_GitHub/STM32CubeFunctionPack_ALLMEMS1_V4.2.0/Drivers/BSP/SensorTile.box/SensorTile.box_bc.c Drivers/BSP/SensorTile.box/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L4R9xx -DSTM32_SENSORTILEBOX -DUSE_HAL_DRIVER -c -I../../Inc -I../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../Drivers/BSP/Components/Common -I../../../../../../Drivers/BSP/SensorTile.box -I../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../Drivers/BSP/Components/lis2mdl -I../../../../../../Drivers/BSP/Components/lps22hh -I../../../../../../Drivers/BSP/Components/lsm6dsox -I../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../Middlewares/ST/STM32_MetaDataManager -I../../../../../../Middlewares/ST/BlueNRG-2/includes -I../../../../../../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../../../../../../Middlewares/ST/BlueNRG-2/utils -I../../../../../../Middlewares/ST/STM32_MotionAR_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionCP_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionGR_Library/Inc -I../../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../../Middlewares/ST/STM32_MotionFA_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionID_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionPE_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionSD_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionTL_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionVC_Library/Inc -I../../../../../../Middlewares/ST/STM32_BLE_Manager/Inc -I../../../../../../Middlewares/Third_Party/parson -I../../../../../../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/SensorTile.box/SensorTile.box_bc.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Drivers/BSP/SensorTile.box/SensorTile.box_bus.o: C:/MyFolder/01_GitRepoProject/01_Release\ Candidate/02_GitHub/STM32CubeFunctionPack_ALLMEMS1_V4.2.0/Drivers/BSP/SensorTile.box/SensorTile.box_bus.c Drivers/BSP/SensorTile.box/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L4R9xx -DSTM32_SENSORTILEBOX -DUSE_HAL_DRIVER -c -I../../Inc -I../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../Drivers/BSP/Components/Common -I../../../../../../Drivers/BSP/SensorTile.box -I../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../Drivers/BSP/Components/lis2mdl -I../../../../../../Drivers/BSP/Components/lps22hh -I../../../../../../Drivers/BSP/Components/lsm6dsox -I../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../Middlewares/ST/STM32_MetaDataManager -I../../../../../../Middlewares/ST/BlueNRG-2/includes -I../../../../../../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../../../../../../Middlewares/ST/BlueNRG-2/utils -I../../../../../../Middlewares/ST/STM32_MotionAR_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionCP_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionGR_Library/Inc -I../../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../../Middlewares/ST/STM32_MotionFA_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionID_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionPE_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionSD_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionTL_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionVC_Library/Inc -I../../../../../../Middlewares/ST/STM32_BLE_Manager/Inc -I../../../../../../Middlewares/Third_Party/parson -I../../../../../../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/SensorTile.box/SensorTile.box_bus.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Drivers/BSP/SensorTile.box/SensorTile.box_env_sensors.o: C:/MyFolder/01_GitRepoProject/01_Release\ Candidate/02_GitHub/STM32CubeFunctionPack_ALLMEMS1_V4.2.0/Drivers/BSP/SensorTile.box/SensorTile.box_env_sensors.c Drivers/BSP/SensorTile.box/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L4R9xx -DSTM32_SENSORTILEBOX -DUSE_HAL_DRIVER -c -I../../Inc -I../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../Drivers/BSP/Components/Common -I../../../../../../Drivers/BSP/SensorTile.box -I../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../Drivers/BSP/Components/lis2mdl -I../../../../../../Drivers/BSP/Components/lps22hh -I../../../../../../Drivers/BSP/Components/lsm6dsox -I../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../Middlewares/ST/STM32_MetaDataManager -I../../../../../../Middlewares/ST/BlueNRG-2/includes -I../../../../../../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../../../../../../Middlewares/ST/BlueNRG-2/utils -I../../../../../../Middlewares/ST/STM32_MotionAR_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionCP_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionGR_Library/Inc -I../../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../../Middlewares/ST/STM32_MotionFA_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionID_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionPE_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionSD_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionTL_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionVC_Library/Inc -I../../../../../../Middlewares/ST/STM32_BLE_Manager/Inc -I../../../../../../Middlewares/Third_Party/parson -I../../../../../../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/SensorTile.box/SensorTile.box_env_sensors.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Drivers/BSP/SensorTile.box/SensorTile.box_env_sensors_ex.o: C:/MyFolder/01_GitRepoProject/01_Release\ Candidate/02_GitHub/STM32CubeFunctionPack_ALLMEMS1_V4.2.0/Drivers/BSP/SensorTile.box/SensorTile.box_env_sensors_ex.c Drivers/BSP/SensorTile.box/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L4R9xx -DSTM32_SENSORTILEBOX -DUSE_HAL_DRIVER -c -I../../Inc -I../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../Drivers/BSP/Components/Common -I../../../../../../Drivers/BSP/SensorTile.box -I../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../Drivers/BSP/Components/lis2mdl -I../../../../../../Drivers/BSP/Components/lps22hh -I../../../../../../Drivers/BSP/Components/lsm6dsox -I../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../Middlewares/ST/STM32_MetaDataManager -I../../../../../../Middlewares/ST/BlueNRG-2/includes -I../../../../../../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../../../../../../Middlewares/ST/BlueNRG-2/utils -I../../../../../../Middlewares/ST/STM32_MotionAR_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionCP_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionGR_Library/Inc -I../../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../../Middlewares/ST/STM32_MotionFA_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionID_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionPE_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionSD_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionTL_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionVC_Library/Inc -I../../../../../../Middlewares/ST/STM32_BLE_Manager/Inc -I../../../../../../Middlewares/Third_Party/parson -I../../../../../../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/SensorTile.box/SensorTile.box_env_sensors_ex.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Drivers/BSP/SensorTile.box/SensorTile.box_motion_sensors.o: C:/MyFolder/01_GitRepoProject/01_Release\ Candidate/02_GitHub/STM32CubeFunctionPack_ALLMEMS1_V4.2.0/Drivers/BSP/SensorTile.box/SensorTile.box_motion_sensors.c Drivers/BSP/SensorTile.box/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L4R9xx -DSTM32_SENSORTILEBOX -DUSE_HAL_DRIVER -c -I../../Inc -I../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../Drivers/BSP/Components/Common -I../../../../../../Drivers/BSP/SensorTile.box -I../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../Drivers/BSP/Components/lis2mdl -I../../../../../../Drivers/BSP/Components/lps22hh -I../../../../../../Drivers/BSP/Components/lsm6dsox -I../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../Middlewares/ST/STM32_MetaDataManager -I../../../../../../Middlewares/ST/BlueNRG-2/includes -I../../../../../../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../../../../../../Middlewares/ST/BlueNRG-2/utils -I../../../../../../Middlewares/ST/STM32_MotionAR_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionCP_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionGR_Library/Inc -I../../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../../Middlewares/ST/STM32_MotionFA_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionID_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionPE_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionSD_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionTL_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionVC_Library/Inc -I../../../../../../Middlewares/ST/STM32_BLE_Manager/Inc -I../../../../../../Middlewares/Third_Party/parson -I../../../../../../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/SensorTile.box/SensorTile.box_motion_sensors.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Drivers/BSP/SensorTile.box/SensorTile.box_motion_sensors_ex.o: C:/MyFolder/01_GitRepoProject/01_Release\ Candidate/02_GitHub/STM32CubeFunctionPack_ALLMEMS1_V4.2.0/Drivers/BSP/SensorTile.box/SensorTile.box_motion_sensors_ex.c Drivers/BSP/SensorTile.box/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L4R9xx -DSTM32_SENSORTILEBOX -DUSE_HAL_DRIVER -c -I../../Inc -I../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../Drivers/BSP/Components/Common -I../../../../../../Drivers/BSP/SensorTile.box -I../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../Drivers/BSP/Components/lis2mdl -I../../../../../../Drivers/BSP/Components/lps22hh -I../../../../../../Drivers/BSP/Components/lsm6dsox -I../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../Middlewares/ST/STM32_MetaDataManager -I../../../../../../Middlewares/ST/BlueNRG-2/includes -I../../../../../../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../../../../../../Middlewares/ST/BlueNRG-2/utils -I../../../../../../Middlewares/ST/STM32_MotionAR_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionCP_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionGR_Library/Inc -I../../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../../Middlewares/ST/STM32_MotionFA_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionID_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionPE_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionSD_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionTL_Library/Inc -I../../../../../../Middlewares/ST/STM32_MotionVC_Library/Inc -I../../../../../../Middlewares/ST/STM32_BLE_Manager/Inc -I../../../../../../Middlewares/Third_Party/parson -I../../../../../../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/SensorTile.box/SensorTile.box_motion_sensors_ex.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-SensorTile-2e-box

clean-Drivers-2f-BSP-2f-SensorTile-2e-box:
	-$(RM) ./Drivers/BSP/SensorTile.box/SensorTile.box.d ./Drivers/BSP/SensorTile.box/SensorTile.box.o ./Drivers/BSP/SensorTile.box/SensorTile.box.su ./Drivers/BSP/SensorTile.box/SensorTile.box_audio.d ./Drivers/BSP/SensorTile.box/SensorTile.box_audio.o ./Drivers/BSP/SensorTile.box/SensorTile.box_audio.su ./Drivers/BSP/SensorTile.box/SensorTile.box_bc.d ./Drivers/BSP/SensorTile.box/SensorTile.box_bc.o ./Drivers/BSP/SensorTile.box/SensorTile.box_bc.su ./Drivers/BSP/SensorTile.box/SensorTile.box_bus.d ./Drivers/BSP/SensorTile.box/SensorTile.box_bus.o ./Drivers/BSP/SensorTile.box/SensorTile.box_bus.su ./Drivers/BSP/SensorTile.box/SensorTile.box_env_sensors.d ./Drivers/BSP/SensorTile.box/SensorTile.box_env_sensors.o ./Drivers/BSP/SensorTile.box/SensorTile.box_env_sensors.su ./Drivers/BSP/SensorTile.box/SensorTile.box_env_sensors_ex.d ./Drivers/BSP/SensorTile.box/SensorTile.box_env_sensors_ex.o ./Drivers/BSP/SensorTile.box/SensorTile.box_env_sensors_ex.su ./Drivers/BSP/SensorTile.box/SensorTile.box_motion_sensors.d ./Drivers/BSP/SensorTile.box/SensorTile.box_motion_sensors.o ./Drivers/BSP/SensorTile.box/SensorTile.box_motion_sensors.su ./Drivers/BSP/SensorTile.box/SensorTile.box_motion_sensors_ex.d ./Drivers/BSP/SensorTile.box/SensorTile.box_motion_sensors_ex.o ./Drivers/BSP/SensorTile.box/SensorTile.box_motion_sensors_ex.su

.PHONY: clean-Drivers-2f-BSP-2f-SensorTile-2e-box
