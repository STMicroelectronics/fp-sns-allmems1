/**
  @page Virtual_COM_Port sample application for BlueNRG-MS/BlueNRG-M0 Expansion Board and STM32 Nucleo Boards
  
  @verbatim
  ******************************************************************************
  * @file    Virtual_COM_Port/readme.txt  
  * @author  SRA Application Team
  * @brief   This application is an example to be loaded in order to use the
  *          BlueNRG GUI with BlueNRG-MS development platforms.
  ******************************************************************************
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  @endverbatim

@par Example Description 

Virtual_COM_Port is the application to be used with the BlueNRG GUI SW package 
(the STSW-BNRGUI available on st.com).
------------------------------------
WARNING: When starting the project from Example Selector in STM32CubeMX and regenerating it
from ioc file, you may face a build issue. To solve it, if you started the project for the
Nucleo-L476RG board, remove from the IDE project the file stm32l4xx_nucleo.c in the Application/User
virtual folder and delete, from Src and Inc folders, the files: stm32l4xx_nucleo.c, stm32l4xx_nucleo.h
and stm32l4xx_nucleo_errno.h.
------------------------------------
- User can also use this project in order to port the VCOM application to his
  specific BlueNRG-MS/BlueNRG-M0 PCB (assuming that the customer PCB has a USB or 
  RS232 I/O port available for PC connection).
- This application provides an interface compliant with the Bluetooth Low Energy DTM
  test commands.
- This application is not a reference application to be used for BlueNRG-MS application
  development and evaluation.

To update the BlueNRG-MS stack on the BlueNRG-M0 module:
- Flash the Virtual_COM_Port binary file on your STM32 board equipped with the X-NUCLEO-IDB05A2
  expansion board
- Launch the latest version of the BlueNRG GUI (the STSW-BNRGUI available on st.com)
- From Settings -> Set Baud Rate, set the serial port baud rate to 115200
- Select the COM port and open it
- From Tools -> Stack Updater, select BlueNRG-MS
- Click Browse
  - select the folder BlueNRG-MS_stack
  - select the .img file for 32MHz, XO32K, 4M
    (e.g., for BlueNRG-MS stack V7.3a, the bluenrg_7_3a_Mode_2-32MHz-XO32K_4M.img)
  - click Open
- Click Update and wait for the process to be completed

@par Keywords

BLE, SPI, USART, VCOM, BlueNRG-M0, BlueNRG-MS

@par Directory contents

 - app_bluenrg_ms.c       Virtual_COM_Port initialization and applicative code
  
 - hci_tl_interface.c     Interface to the BlueNRG HCI Transport Layer 
 
 - main.c                 Main program body
  
 - stm32**xx_hal_msp.c    Source code for MSP Initialization and de-Initialization

 - stm32**xx_it.c         Source code for interrupt Service Routines

 - stm32**xx_nucleo.c     Source file for the BSP Common driver 
						
 - stm32**xx_nucleo_bus.c Source file for the BSP BUS IO driver
 
 - system_stm32**xx.c     CMSIS Cortex-Mx Device Peripheral Access Layer
                          System Source File
  
@par Hardware and Software environment

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
    
@par How to use it? 

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

 - IMPORTANT NOTE: To avoid issues with USB connection (mandatory if you have USB 3.0), it is   
   suggested to update the ST-Link/V2 firmware for STM32 Nucleo boards to the latest version.
   Please refer to the readme.txt file in the Applications directory for details.

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
