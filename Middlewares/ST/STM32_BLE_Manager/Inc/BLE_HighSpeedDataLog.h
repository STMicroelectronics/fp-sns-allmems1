/**
  ******************************************************************************
  * @file    BLE_HighSpeedDataLog.h
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version 1.1.0
  * @date    23-Dec-2021
  * @brief   BLE_HighSpeedDataLog info services APIs.
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
#ifndef _BLE_HIGH_SPEED_DATA_LOG_H_
#define _BLE_HIGH_SPEED_DATA_LOG_H_

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Exported typedef --------------------------------------------------------- */
typedef void (*CustomWriteRequestHighSpeedDataLogFunction)(uint8_t * att_data, uint8_t data_length);

/* Exported Variables ------------------------------------------------------- */
extern BLE_NotifyEnv_t BLE_HighSpeedDataLog_NotifyEvent;
extern CustomWriteRequestHighSpeedDataLogFunction CustomWriteRequestHighSpeedDataLogFunctionPointer;


/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Init High Speed Data Log info service
 * @param  None
 * @retval BleCharTypeDef* BleCharPointer: Data structure pointer for High Speed Data Log info service
 */
extern BleCharTypeDef* BLE_InitHighSpeedDataLogService(void);

/**
 * @brief  Setting High Speed Data Log Advertize Data
 * @param  uint8_t *manuf_data: Advertize Data
 * @retval None
 */
extern void BLE_SetHighSpeedDataLogAdvertizeData(uint8_t *manuf_data);

/**
 * @brief  High Speed Data Log Send Buffer
 * @param  uint8_t* buffer
 * @param  uint32_t len
 * @retval tBleStatus   Status
 */
tBleStatus BLE_HighSpeedDataLogSendBuffer(uint8_t* buffer, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* _BLE_HIGH_SPEED_DATA_LOG_H_ */

