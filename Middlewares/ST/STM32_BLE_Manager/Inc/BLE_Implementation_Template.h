/**
  ******************************************************************************
  * @file    BLE_Implementation_Template.h 
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version 1.1.0
  * @date    23-Dec-2021
  * @brief   BLE Implementation header template file.
  *          This file should be copied to the application folder and renamed
  *          to BLE_Implementation.h.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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


/* Exported Defines --------------------------------------------------------*/

/* Define the node name MUST be 7 char long
 *
 * #define BLE_MANAGER_DEFAULT_NODE_NAME 'B','L','E','_','M','N','G'-*/
#define BLE_MANAGER_DEFAULT_NODE_NAME 'B','L','E','_','M','N','G'

/* STM32 Unique ID */
#define BLE_STM32_UUID STM32_UUID

/********************************************************
 * Insert the STM32 Microcontrolles type: for example:  *
 *                                                      *
 *   #define BLE_STM32_MICRO "L4R9"                     *
 ********************************************************/
 
/* STM32  Microcontrolles type */
#define BLE_STM32_MICRO "InsertMicrocontrollesType"

/**********************************************************
 * Insert the STM32 board type: for example:              *
 *                                                        *
 *   #define BLE_STM32_BOARD "STM32L4R9ZI-SensorTile.box" *
 **********************************************************/
 
/* STM32 board type*/
#define BLE_STM32_BOARD "InsertSTM32BoardType"

/**********************************************************
 * Insert the package version firmware: for example:      *
 *                                                        *
 *   #define BLE_VERSION_FW_MAJOR    STBOX1_VERSION_MAJOR *
 *   #define BLE_VERSION_FW_MINOR    STBOX1_VERSION_MINOR *
 *   #define BLE_VERSION_FW_PATCH    STBOX1_VERSION_PATCH *
 **********************************************************/
 
/* Package Version firmware */
#define BLE_VERSION_FW_MAJOR    1
#define BLE_VERSION_FW_MINOR    0
#define BLE_VERSION_FW_PATCH    0

/********************************************************
 * Insert the firmware package name: for example:       *
 *                                                      *
 *   #define BLE_FW_PACKAGENAME      STBOX1_PACKAGENAME *
 ********************************************************/

/* Firmware Package Name */
#define BLE_FW_PACKAGENAME      "InsertPackageName"


/* Exported Variables ------------------------------------------------------- */
extern int32_t  NeedToClearSecureDB;


/* Exported functions ------------------------------------------------------- */
extern void BLE_InitCustomService(void);
extern void BLE_SetCustomAdvertizeData(uint8_t *manuf_data);

#ifdef __cplusplus
}
#endif

#endif /* _BLE_IMPLEMENTATION_H_ */

