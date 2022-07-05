/**
  ******************************************************************************
  * @file    BLE_SensorFusion.h
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version 1.1.0
  * @date    23-Dec-2021
  * @brief   Sensor Fusion info service APIs.
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
#ifndef _BLE_SENSOR_FUSION_H_
#define _BLE_SENSOR_FUSION_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported defines ---------------------------------------------------------*/

/* Exported typedef --------------------------------------------------------- */
typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
} BLE_MOTION_SENSOR_Axes_t;

/* Exported Variables ------------------------------------------------------- */
extern BLE_NotifyEnv_t BLE_SensorFusion_NotifyEvent;

/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Init Sensor Fusion info service
 * @param  uint8_t NumberQuaternionsToSend  Number of quaternions send (1,2,3)
 * @retval BleCharTypeDef* BleCharPointer: Data structure pointer for Sensor Fusion info service
 */
extern BleCharTypeDef* BLE_InitSensorFusionService(uint8_t NumberQuaternionsToSend);

#ifndef BLE_MANAGER_SDKV2
/**
 * @brief  Setting Sensor Fusion Advertise Data
 * @param  uint8_t *manuf_data: Advertise Data
 * @retval None
 */
extern void BLE_SetSensorFusionAdvertizeData(uint8_t *manuf_data);
#endif /* BLE_MANAGER_SDKV2 */

/**
 * @brief  Update quaternions characteristic value
 * @param  BLE_MOTION_SENSOR_Axes_t *data Structure containing the quaterions
 * @param  uint8_t NumberQuaternionsToSend  Number of quaternions send (1,2,3)
 * @retval tBleStatus      Status
 */
extern tBleStatus BLE_SensorFusionUpdate(BLE_MOTION_SENSOR_Axes_t *data, uint8_t NumberQuaternionsToSend);

#ifdef __cplusplus
}
#endif

#endif /* _BLE_SENSOR_FUSION_H_ */

