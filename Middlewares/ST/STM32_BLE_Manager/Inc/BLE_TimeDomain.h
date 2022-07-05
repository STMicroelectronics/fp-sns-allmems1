/**
  ******************************************************************************
  * @file    BLE_TimeDomain.h
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version 1.1.0
  * @date    23-Dec-2021
  * @brief   Time Domain info services APIs.
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
#ifndef _BLE_TIME_DOMAIN_H_
#define _BLE_TIME_DOMAIN_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Exported Types ----------------------------------------------------------- */
 /**
* @brief  X-Y-Z Generic Value in float
*/
typedef struct
{
  float x;         //!< Generic X Value in float
  float y;         //!< Generic Y Value in float
  float z;         //!< Generic Z Value in float
} BLE_MANAGER_TimeDomainGenericValue_t;

/* Exported functions ------------------------------------------------------- */
extern BLE_NotifyEnv_t BLE_TimeDomain_NotifyEvent;

/**
 * @brief  Init Time Domain info service
 * @param  None
 * @retval BleCharTypeDef* BleCharPointer: Data structure pointer for Time Domain info service
 */
BleCharTypeDef* BLE_InitTimeDomainService(void);

#ifndef BLE_MANAGER_SDKV2
/**
 * @brief  Setting Time Domain Advertize Data
 * @param  uint8_t *manuf_data: Advertize Data
 * @retval None
 */
void BLE_SetTimeDomainAdvertizeData(uint8_t *manuf_data);
#endif /* BLE_MANAGER_SDKV2 */

/*
 * @brief  Update Time Domain characteristic value
 * @param  BLE_MANAGER_TimeDomainGenericValue_t PeakValue
 * @param  BLE_MANAGER_TimeDomainGenericValue_t SpeedRmsValue
 * @retval tBleStatus   Status
 */
tBleStatus BLE_TimeDomainUpdate(BLE_MANAGER_TimeDomainGenericValue_t PeakValue, BLE_MANAGER_TimeDomainGenericValue_t SpeedRmsValue);

#ifdef __cplusplus
}
#endif

#endif /* _BLE_TIME_DOMAIN_H_ */

