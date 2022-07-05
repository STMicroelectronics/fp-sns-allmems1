/**
  ******************************************************************************
  * @file    BLE_FFT_AlarmSubrangeStatus.h
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version 1.1.0
  * @date    23-Dec-2021
  * @brief   FFT Alarm Subrange Status info services APIs.
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
#ifndef _BLE_FFT_ALARM_SUBRANGE_STATUS_H_
#define _BLE_FFT_ALARM_SUBRANGE_STATUS_H_

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
} BLE_MANAGER_FFTAlarmSubrangeStatusGenericValue_t;

/**
 * @brief  Warning Alarm Datatype
 */
typedef enum
{
  BLE_GOOD_SUBRANGE          = (uint8_t)0x00,  //!< GOOD STATUS
  BLE_WARNING_SUBRANGE       = (uint8_t)0x01,  //!< WARNING STATUS
  BLE_ALARM_SUBRANGE         = (uint8_t)0x02,  //!< ALARM STATUS
  BLE_NONE_SUBRANGE          = (uint8_t)0x03,  //!< RFU STATUS
} BLE_Manager_FFTAlarmSubrangeStatusAlarmType_t;

/**
 * @brief  STATUS for FFT Subrange Warning-Alarm
 */
typedef struct 
{
  BLE_Manager_FFTAlarmSubrangeStatusAlarmType_t STATUS_AXIS_X;   //!< X STATUS ALARM
  BLE_Manager_FFTAlarmSubrangeStatusAlarmType_t STATUS_AXIS_Y;   //!< Y STATUS ALARM
  BLE_Manager_FFTAlarmSubrangeStatusAlarmType_t STATUS_AXIS_Z;   //!< Z STATUS ALARM
} sBLE_Manager_FFTAlarmSubrangeStatusAlarm_t;

/* Exported Variables ------------------------------------------------------- */
extern BLE_NotifyEnv_t BLE_FFTAlarmSubrangeStatus_NotifyEvent;

/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Init FFT Alarm Subrange Status info service
 * @param  None
 * @retval BleCharTypeDef* BleCharPointer: Data structure pointer for FFT Alarm Subrange Status info service
 */
BleCharTypeDef* BLE_InitFFTAlarmSubrangeStatusService(void);

#ifndef BLE_MANAGER_SDKV2
/**
 * @brief  Setting FFT Alarm Subrange Status Advertize Data
 * @param  uint8_t *manuf_data: Advertize Data
 * @retval None
 */
void BLE_SetFFTAlarmSubrangeStatusAdvertizeData(uint8_t *manuf_data);
#endif /* BLE_MANAGER_SDKV2 */

/*
 * @brief  Update FFT Alarm Subrange RMS status value
 * @param  sBLE_Manager_FFTAlarmSubrangeStatusAlarm_t              AlarmStatus
 * @param  BLE_MANAGER_FFTAlarmSubrangeStatusGenericValue_t        SubrangeMaxValue
 * @param  BLE_MANAGER_FFTAlarmSubrangeStatusGenericValue_t        SubrangeFreqMaxValue
 * @retval tBleStatus   Status
 */
tBleStatus BLE_FFTAlarmSubrangeStatusUpdate(sBLE_Manager_FFTAlarmSubrangeStatusAlarm_t          AlarmStatus,
                                            BLE_MANAGER_FFTAlarmSubrangeStatusGenericValue_t    SubrangeMaxValue,
                                            BLE_MANAGER_FFTAlarmSubrangeStatusGenericValue_t    SubrangeFreqMaxValue);

#ifdef __cplusplus
}
#endif

#endif /* _BLE_FFT_ALARM_SUBRANGE_STATUS_H_ */

