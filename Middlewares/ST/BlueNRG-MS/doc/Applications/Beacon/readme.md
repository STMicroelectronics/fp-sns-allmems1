
## <b>Beacon Application Description</b>

This example shows how to implement an Eddystone Beacon device.
The communication is done using an STM32 Nucleo board, equipped with an ST's 
BlueNRG-M0/BlueNRG-MS expansion board, and a Smartphone with BLE.

Example Description:

An Eddystone Beacon is a smart Bluetooth Low Energy device that transmits 
a small data payload at regular intervals using Bluetooth advertising packets.

Beacons are used to mark important places and objects. Typically, a beacon 
is visible to a user's device from a range of a few meters, allowing for highly 
context-sensitive use cases. 

Eddystone is an open beacon format from Google that works with Android and iOS. 
Specifications can be found at https://developers.google.com/beacons/

Two different kinds of devices can be selected by defining/undefining the 
#define EDDYSTONE_BEACON_TYPE EDDYSTONE_URL_BEACON_TYPE in file app_bluenrg_ms.c:
- UID: a UID beacon broadcasts a unique ID that provides proximity and general 
  location information.
- URL: a URL beacon broadcasts a packet containing an URL code usable by compatible 
  applications.

To locate the beacon, it is necessary to have a scanner application running
on a BLE-capable smartphone, such as one of the following ones for Android:
- Physical Web, https://play.google.com/store/apps/details?id=physical_web.org.physicalweb
- iBeacon & Eddystone Scanner, https://play.google.com/store/apps/details?id=de.flurp.beaconscanner.app
- Beacon Radar, https://play.google.com/store/apps/details?id=net.beaconradar
An alternative is to use a 'Physical Web' compatible browser like Google Chrome (version >=44).

NO SUPPORT WILL BE PROVIDED TO YOU BY STMICROELECTRONICS FOR ANY OF THE
ANDROID .apk FILES INCLUDED IN OR REFERENCED BY THIS PACKAGE.

Known limitations:

- When starting the project from Example Selector in STM32CubeMX and regenerating it
  from ioc file, you may face a build issue. To solve it, if you started the project for the
  Nucleo-L476RG board, remove from the IDE project the file stm32l4xx_nucleo.c in the Application/User
  virtual folder and delete, from Src and Inc folders, the files: stm32l4xx_nucleo.c, stm32l4xx_nucleo.h
  and stm32l4xx_nucleo_errno.h.

### <b>Keywords</b>

BLE, Beacon, Eddystone Beacon, SPI, BlueNRG-M0, BlueNRG-MS

### <b>Directory contents</b>

 - app_bluenrg_ms.c       Beacon initialization and applicative code
 
 - eddystone_beacon.c     Set beacon services 
 
 - hci_tl_interface.c     Interface to the BlueNRG HCI Transport Layer 
 
 - main.c                 Main program body
 
 - sensor_service.c       Module for processing ACI events
 
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
