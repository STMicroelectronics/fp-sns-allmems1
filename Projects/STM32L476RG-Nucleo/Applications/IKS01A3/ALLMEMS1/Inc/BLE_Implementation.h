/**
  ******************************************************************************
  * @file    BLE_Implementation.h 
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   BLE Implementation header template file.
  *          This file should be copied to the application folder and renamed
  *          to BLE_Implementation.h.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/  
#ifndef _BLE_IMPLEMENTATION_H_
#define _BLE_IMPLEMENTATION_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/

/**
* User can added here the header file for the selected BLE features.
* For example:
* #include "BLE_Environmental.h"
* #include "BLE_Inertial.h"
*/

#include "BLE_Environmental.h"
#include "BLE_Inertial.h"
#include "BLE_ActivityRecognition.h"
#include "BLE_CarryPosition.h"
#include "BLE_FitnessActivities.h"
#include "BLE_GestureRecognition.h"
#include "BLE_MotionIntensity.h"
#include "BLE_SensorFusion.h"
#include "BLE_ECompass.h"
#include "BLE_MotionAlgorithms.h"
#include "BLE_TiltSensing.h"
#include "BLE_AudioLevel.h"
#include "BLE_AudioSourceLocalization.h"
#include "BLE_AccEvent.h"


/* Exported Defines --------------------------------------------------------*/
     
/* Select the used hardware platform
 *
 * STEVAL-WESU1                         --> BLE_MANAGER_STEVAL_WESU1_PLATFORM
 * STEVAL-STLKT01V1 (SensorTile)        --> BLE_MANAGER_SENSOR_TILE_PLATFORM
 * STEVAL-BCNKT01V1 (BlueCoin)          --> BLE_MANAGER_BLUE_COIN_PLATFORM
 * STEVAL-IDB008Vx                      --> BLE_MANAGER_STEVAL_IDB008VX_PLATFORM
 * STEVAL-BCN002V1B (BlueTile)          --> BLE_MANAGER_STEVAL_BCN002V1_PLATFORM
 * STEVAL-MKSBOX1V1 (SensorTile.box)    --> BLE_MANAGER_SENSOR_TILE_BOX_PLATFORM
 * DISCOVERY-IOT01A                     --> BLE_MANAGER_DISCOVERY_IOT01A_PLATFORM
 * STEVAL-STWINKT1                      --> BLE_MANAGER_STEVAL_STWINKIT1_PLATFORM
 * STM32NUCLEO Board                    --> BLE_MANAGER_NUCLEO_PLATFORM
 * STM32F401RE_NUCLEO Board             --> BLE_MANAGER_STM32F401RE_NUCLEO_PLATFORM
 * STM32L476RG_NUCLEO Board             --> BLE_MANAGER_STM32L476RG_NUCLEO_PLATFORM
 * STM32L053R8_NUCLEO Board             --> BLE_MANAGER_STM32L053R8_NUCLEO_PLATFORM
 *
 * For example:
 * #define BLE_MANAGER_USED_PLATFORM	BLE_MANAGER_NUCLEO_PLATFORM
 *
*/

/* Used platform */
#define BLE_MANAGER_USED_PLATFORM       BLE_MANAGER_STM32L476RG_NUCLEO_PLATFORM

/* STM32 Unique ID */
#define BLE_STM32_UUID STM32_UUID

/* STM32  Microcontrolles type */
#define BLE_STM32_MICRO "L476"

/* STM32 board type*/
#define BLE_STM32_BOARD "STM32L476RG-NUCLEO"

/* Package Version firmware */
#define BLE_VERSION_FW_MAJOR    ALLMEMS1_VERSION_MAJOR
#define BLE_VERSION_FW_MINOR    ALLMEMS1_VERSION_MINOR
#define BLE_VERSION_FW_PATCH    ALLMEMS1_VERSION_PATCH

/* Firmware Package Name */
#define BLE_FW_PACKAGENAME      ALLMEMS1_PACKAGENAME
   
/* Feature mask for Sensor fusion short precision */
#define FEATURE_MASK_SENSORFUSION_SHORT 0x00000100

/* Feature mask for e-compass */
#define FEATURE_MASK_ECOMPASS 0x00000040

/* Feature mask for Accelerometer events */
#define FEATURE_MASK_ACC_EVENTS 0x00000400

/* Feature mask for Temperature1 */
#define FEATURE_MASK_TEMP1 0x00040000

/* Feature mask for Temperature2 */
#define FEATURE_MASK_TEMP2 0x00010000

/* Feature mask for Pressure */
#define FEATURE_MASK_PRESS 0x00100000

/* Feature mask for Humidity */
#define FEATURE_MASK_HUM   0x00080000

/* Feature mask for Accelerometer */
#define FEATURE_MASK_ACC   0x00800000

/* Feature mask for Gyroscope */
#define FEATURE_MASK_GRYO  0x00400000

/* Feature mask for Magnetometer */
#define FEATURE_MASK_MAG   0x00200000
   
/* Feature mask for SourceLocalization */
#define FEATURE_MASK_DIR_OF_ARRIVAL 0x10000000
   
/* W2ST command - SL sensitivity */
#define W2ST_COMMAND_SL_SENSITIVITY 0xCC
/* W2ST command - SL sensitivity Low */
#define W2ST_COMMAND_SL_LOW  0x00
/* W2ST command - SL sensitivity High */
#define W2ST_COMMAND_SL_HIGH  0x01

/* W2ST command for asking the calibration status */
#define W2ST_COMMAND_CAL_STATUS 0xFF
/* W2ST command for resetting the calibration */
#define W2ST_COMMAND_CAL_RESET  0x00
/* W2ST command for stopping the calibration process */
#define W2ST_COMMAND_CAL_STOP   0x01
   
/* Exported Variables ------------------------------------------------------- */
extern uint8_t connected;
extern int32_t  NeedToClearSecureDB;
extern uint32_t ConnectionBleStatus;
extern uint32_t FirstConnectionConfig;

/* Exported functions ------------------------------------------------------- */
extern void BLE_InitCustomService(void);
extern void BLE_SetCustomAdvertizeData(uint8_t *manuf_data);
extern void BluetoothInit(void);

/* Exported macro ------------------------------------------------------------*/
#define W2ST_CHECK_CONNECTION(BleChar) ((ConnectionBleStatus&(BleChar)) ? 1 : 0)
#define W2ST_ON_CONNECTION(BleChar)    (ConnectionBleStatus|=(BleChar))
#define W2ST_OFF_CONNECTION(BleChar)   (ConnectionBleStatus&=(~BleChar))

#ifdef __cplusplus
}
#endif

#endif /* _BLE_IMPLEMENTATION_H_ */

