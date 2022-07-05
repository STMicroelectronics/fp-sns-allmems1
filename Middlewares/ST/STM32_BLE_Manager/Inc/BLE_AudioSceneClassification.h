/**
  ******************************************************************************
  * @file    BLE_AudioSceneClasssification.h
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version 1.1.0
  * @date    23-Dec-2021
  * @brief   Audio Scene Classification info service APIs.
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
#ifndef _BLE_AUDIO_SCENE_CALSSIFICATION_H_
#define _BLE_AUDIO_SCENE_CALSSIFICATION_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported defines ---------------------------------------------------------*/

/* Exported typedef --------------------------------------------------------- */
typedef enum
{
  BLE_ASC_HOME      = 0x00,
  BLE_ASC_OUTDOOR   = 0x01,
  BLE_ASC_CAR       = 0x02,
  BLE_ASC_OFF       = 0xF0, //Off
  BLE_ASC_ON        = 0xF1, //On
  BLE_ASC_UNDEFINED = 0xFF
} BLE_ASC_output_t;

typedef void (*CustomReadRequestAudioSceneClass_t)(BLE_ASC_output_t *ASC_Code);

/* Exported Variables ------------------------------------------------------- */
extern BLE_NotifyEnv_t BLE_AudioSceneClass_NotifyEvent;
extern CustomReadRequestAudioSceneClass_t CustomReadRequestAudioSceneClass;

/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Init Audio Scene Classification info service
 * @param  None
 * @retval BleCharTypeDef* BleCharPointer: Data structure pointer for Activity Classification info service
 */
extern BleCharTypeDef* BLE_InitAudioSceneClassService(void);

/**
 * @brief  Update Audio Scene Classification characteristic
 * @param  BLE_ASC_output_t ASC_Code Audio Scene Classification Code
 * @retval tBleStatus   Status
 */
extern tBleStatus BLE_AudioSceneClassUpdate(BLE_ASC_output_t ASC_Code);

#ifdef __cplusplus
}
#endif

#endif /* _BLE_AUDIO_SCENE_CALSSIFICATION_H_ */

