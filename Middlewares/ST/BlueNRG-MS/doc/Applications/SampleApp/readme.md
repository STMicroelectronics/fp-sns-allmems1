
#### <b>SampleApp Application Description</b>
  
This application shows how to create a BLE client and server connection.

Example Description:

This application shows how to simply use the BLE Stack creating a client and server connection.

To test this application you need two STM32 Nucleo boards with their respective
BlueNRG-MS/BlueNRG-M0 STM32 expansion boards. One board needs to be configured
as Server-Peripheral role, while the other needs to be configured as Client-Central
role.
Before flashing the boards, please make sure to use the right configuration by selecting
it from the menu options of the toolchain.

Keeping either defined or undefined the #define SERVER_ROLE, in file app_bluenrg_ms.c, the
target can be respectively configured either as SERVER or as CLIENT.
 - Program the CLIENT on one STM32 Nucleo board, with BlueNRG-MS/BlueNRG-M0 STM32 expansion board,
   and reset it. 
 - Program the SERVER on a second STM32 Nucleo board, with BlueNRG-MS/BlueNRG-M0 STM32 expansion
   board, and reset it. 
 - After establishing the connection between the two boards (when the LED2 on the
   CLIENT turns off),
   by pressing the USER button on one board, the LD2 LED on the other one gets toggled
   and viceversa.
 - If you have only one STM32 Nucleo board, you can program it as SERVER and use as CLIENT
   the BLE IOT app for Android devices available on the Play Store at
   https://play.google.com/store/apps/details?id=com.stmicro.bleiot 

Known limitations:

- When starting the project from Example Selector in STM32CubeMX and regenerating it
  from ioc file, you may face a build issue. To solve it, if you started the project for the
  Nucleo-L476RG board, remove from the IDE project the file stm32l4xx_nucleo.c in the Application/User
  virtual folder and delete, from Src and Inc folders, the files: stm32l4xx_nucleo.c, stm32l4xx_nucleo.h
  and stm32l4xx_nucleo_errno.h.
 
### <b>Keywords</b>

BLE, Master, Slave, Central, Peripheral, SPI, BlueNRG-M0, BlueNRG-MS

### <b>Directory contents</b>

 - app_bluenrg_ms.c       SampleApp initialization and applicative code
 
 - hci_tl_interface.c     Interface to the BlueNRG HCI Transport Layer 
 
 - main.c                 Main program body
 
 - sample_service.c       Add services using a vendor specific profile
 
 - stm32**xx_hal_msp.c    Source code for MSP Initialization and de-Initialization

 - stm32**xx_it.c         Source code for interrupt Service Routines

 - stm32**xx_nucleo.c     Source file for the BSP Common driver 
						
 - stm32**xx_nucleo_bus.c Source file for the BSP BUS IO driver
 
 - system_stm32**xx.c     CMSIS Cortex-Mx Device Peripheral Access Layer
                          System Source File

### <b>Hardware and Software environment</b>

  - This example runs on STM32 Nucleo boards with X-NUCLEO-IDB05A2 STM32 expansion board
    (the X-NUCLEO-IDB05A1 expansion board can be also used)
  - This example has been tested with STMicroelectronics:
    - NUCLEO-F401RE RevC board  
    - NUCLEO-L053R8 RevC board
    - NUCLEO-L476RG RevC board
    and can be easily tailored to any other supported device and development board.
    This example runs also on the NUCLEO-F411RE RevC board, even if the chip could
    be not exploited at its best since the projects are configured for the
    NUCLEO-F401RE target board.

ADDITIONAL_BOARD : X-NUCLEO-IDB05A2 https://www.st.com/content/st_com/en/products/ecosystems/stm32-open-development-environment/stm32-nucleo-expansion-boards/stm32-ode-connect-hw/x-nucleo-idb05a2.html
ADDITIONAL_COMP : BlueNRG-M0 https://www.st.com/content/st_com/en/products/wireless-connectivity/short-range/bluetooth-low-energy-network-co-processors/bluenrg-m0.html
    
### <b>How to use it?</b> 

In order to make the program work, you must do the following:
 - WARNING: before opening the project with any toolchain be sure your folder
   installation path is not too in-depth since the toolchain may report errors
   after building.
 - Open STM32CubeIDE (this firmware has been successfully tested with Version 1.7.0).
   Alternatively you can use the Keil uVision toolchain (this firmware
   has been successfully tested with V5.32.0) or the IAR toolchain (this firmware has 
   been successfully tested with Embedded Workbench V8.50.9).
 - Rebuild all files and load your image into target memory.
 - Run the example.
 - Alternatively, you can download the pre-built binaries in "Binary" 
   folder included in the distributed package.

### <b>Author</b>

SRA Application Team

### <b>License</b>

Copyright (c) 2021 STMicroelectronics.
All rights reserved.

This software is licensed under terms that can be found in the LICENSE file
in the root directory of this software component.
If no LICENSE file comes with this software, it is provided AS-IS.
