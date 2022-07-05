/**
  ******************************************************************************
  * @file    BLE_FitnessActivities.h
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version 1.1.0
  * @date    23-Dec-2021
  * @brief   Fitness Activities info service APIs.
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
#ifndef _BLE_FITNESS_ACTIVITIES_H_
#define _BLE_FITNESS_ACTIVITIES_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported defines ---------------------------------------------------------*/

/* Exported typedef --------------------------------------------------------- */
typedef void (*CustomWriteRequestFitnessActivities_t)(uint8_t FitnessActivitie);

typedef enum
{
  BLE_MFA_NOACTIVITY          = 0x00,
  BLE_MFA_BICEPCURL           = 0x01,
  BLE_MFA_SQUAT               = 0x02,
  BLE_MFA_PUSHUP              = 0x03
} BLE_FitnessActivitiesType_t;

/* Exported Variables ------------------------------------------------------- */
extern BLE_NotifyEnv_t BLE_FitnessActivities_NotifyEvent;

extern CustomWriteRequestFitnessActivities_t CustomWriteRequestFitnessActivities;

/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Init Fitness Activities info service
 * @param  None
 * @retval BleCharTypeDef* BleCharPointer: Data structure pointer for Fitness Activities info service
 */
extern BleCharTypeDef* BLE_InitFitnessActivitiesService(void);

#ifndef BLE_MANAGER_SDKV2
/**
 * @brief  Setting Fitness Activities Advertise Data
 * @param  uint8_t *manuf_data: Advertise Data
 * @retval None
 */
extern void BLE_SetFitnessActivitiesAdvertizeData(uint8_t *manuf_data);
#endif /* BLE_MANAGER_SDKV2 */

/**
 * @brief  Update Fitness Activities characteristic
 * @param  uint8_t MotionCode Detected Motion
 * @retval tBleStatus   Status
 */
extern tBleStatus BLE_FitnessActivitiesUpdate(uint8_t Activity, uint32_t Counter);

#ifdef __cplusplus
}
#endif

#endif /* _BLE_FITNESS_ACTIVITIES_H_ */

