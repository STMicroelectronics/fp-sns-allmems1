/**
  ******************************************************************************
  * @file    BLE_MotionAlgorithms.h
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version 1.1.0
  * @date    23-Dec-2021
  * @brief   Motion Algorithms info service APIs.
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
#ifndef _BLE_MOTION_ALGORITHMS_H_
#define _BLE_MOTION_ALGORITHMS_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported defines ---------------------------------------------------------*/

/* Exported typedef --------------------------------------------------------- */
typedef void (*CustomWriteRequestMotionAlgorithms_t)(void);

typedef enum
{
  BLE_MOTION_ALGORITHMS_NO_ALGO = 0x00,
  BLE_MOTION_ALGORITHMS_PE      = 0x01,
  BLE_MOTION_ALGORITHMS_SD      = 0x02,
  BLE_MOTION_ALGORITHMS_VC      = 0x03
} BLE_MotionAlgorithmsType_t;

/* Exported Variables ------------------------------------------------------- */
extern BLE_NotifyEnv_t BLE_MotionAlgorithms_PE_NotifyEvent;
extern BLE_NotifyEnv_t BLE_MotionAlgorithms_SD_NotifyEvent;
extern BLE_NotifyEnv_t BLE_MotionAlgorithms_VC_NotifyEvent;

extern CustomWriteRequestMotionAlgorithms_t CustomWriteRequestMotionAlgorithms;

/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Init Motion Algorithms info service
 * @param  None
 * @retval BleCharTypeDef* BleCharPointer: Data structure pointer for Motion Algorithms info service
 */
extern BleCharTypeDef* BLE_InitMotionAlgorithmsService(void);

#ifndef BLE_MANAGER_SDKV2
/**
 * @brief  Setting Motion Algorithms Advertise Data
 * @param  uint8_t *manuf_data: Advertise Data
 * @retval None
 */
extern void BLE_SetMotionAlgorithmsAdvertizeData(uint8_t *manuf_data);
#endif /* BLE_MANAGER_SDKV2 */

/**
 * @brief  Update Motion Algorithms characteristic
 * @param  uint8_t MotionCode Detected Motion
 * @retval tBleStatus   Status
 */
extern tBleStatus BLE_MotionAlgorithmsUpdate(uint8_t MotionCode);

#ifdef __cplusplus
}
#endif

#endif /* _BLE_MOTION_ALGORITHMS_H_ */

