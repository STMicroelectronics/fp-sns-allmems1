/**
  ******************************************************************************
  * @file    sensor_service.h 
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   Sensors services APIs
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
#ifndef _SENSOR_SERVICE_H_
#define _SENSOR_SERVICE_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "TargetFeatures.h"

#include "bluenrg_types.h"
#include "bluenrg_gatt_server.h"
#include "bluenrg_gap.h"
#include "string.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "hci_const.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_aci_const.h"   
#include "hci.h"
#include "hci_tl.h"
#include "hci_le.h"
#include "sm.h"

#include <stdlib.h>
#include "main.h"

/* Exported functions ------------------------------------------------------- */
extern tBleStatus Add_HW_SW_ServW2ST_Service(void);
extern tBleStatus AccGyroMag_Update(BSP_MOTION_SENSOR_Axes_t *Acc,BSP_MOTION_SENSOR_Axes_t *Gyro,BSP_MOTION_SENSOR_Axes_t *Mag);
extern tBleStatus AccEvent_Notify(uint16_t Command, uint8_t dimByte);
extern tBleStatus Environmental_Update(int32_t Press, int16_t Temp);

/* Code for MotionFX integration - Start Section */
extern tBleStatus Quat_Update(BSP_MOTION_SENSOR_Axes_t *data);
extern tBleStatus ECompass_Update(uint16_t Angle);
/* Code for MotionFX integration - End Section */

extern tBleStatus AudioLevel_Update(uint16_t *Mic);
extern tBleStatus BatteryInfoUpdate(uint32_t voltage, ChrgStatus_t BattStatus, uint32_t soc);

/* Code for MotionAR integration - Start Section */
extern tBleStatus ActivityRec_Update(MAR_output_t ActivityCode);
/* Code for MotionAR integration - End Section */

/* Code for AcousticSL integration - Start Section */
extern tBleStatus AudioSourceLocalization_Update(uint16_t Angle);
/* Code for AcousticSL integration - End Section */

extern tBleStatus Add_ConsoleW2ST_Service(void);
extern tBleStatus Stderr_Update(uint8_t *data,uint8_t length);
extern tBleStatus Term_Update(uint8_t *data,uint8_t length);

extern tBleStatus Add_ConfigW2ST_Service(void);
extern tBleStatus Config_Notify(uint32_t Feature,uint8_t Command,uint8_t val);

extern void       setConnectable(void);

void       HCI_Event_CB(void *pckt);

/* Exported variables --------------------------------------------------------*/
#define  BLE_MANAGER_SDKV2

/* For enabling the capability to handle BlueNRG Congestion */
#define ACC_BLUENRG_CONGESTION

#ifdef ACC_BLUENRG_CONGESTION
/* For defining how many events skip when there is a congestion */
#define ACC_BLUENRG_CONGESTION_SKIP 30
#endif /* ACC_BLUENRG_CONGESTION */


#define BF_ASR_READY   0
#define BF_STRONG  1

/*************** Don't Change the following defines *************/

/* Define the Max dimesion of the Bluetooth characteristics
for each packet used for Console Service */
#define W2ST_CONSOLE_MAX_CHAR_LEN 20

/* Define the symbol used for defining each termination string
used in Console service */
#define W2ST_CONSOLE_END_STRING "\0"

/* @brief  Scale factor. It is used to scale acceleration from mg to g */ 
#define FROM_MG_TO_G    0.001f

/* Feature mask for Sensor fusion short precision */
#define FEATURE_MASK_SENSORFUSION_SHORT 0x00000100

/* Feature mask for e-compass */
#define FEATURE_MASK_ECOMPASS 0x00000040

/* Feature mask for hardware events */
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

/* Feature mask for Microphone */
#define FEATURE_MASK_MIC   0x04000000

/* Feature mask for BlueVoice */
#define FEATURE_MASK_BLUEVOICE   0x08000000

/* Feature mask for BlueVoice */
#define FEATURE_MASK_BEAMFORMING   0x00000800

/* Feature mask for SourceLocalization */
#define FEATURE_MASK_DIR_OF_ARRIVAL 0x10000000

/* W2ST command for asking the calibration status */
#define W2ST_COMMAND_CAL_STATUS 0xFF
/* W2ST command for resetting the calibration */
#define W2ST_COMMAND_CAL_RESET  0x00
/* W2ST command for stopping the calibration process */
#define W2ST_COMMAND_CAL_STOP   0x01

/* W2ST command - BF type */
#define W2ST_COMMAND_BF_TYPE       0xCC
/* W2ST command - BF ASR_READY */
#define W2ST_COMMAND_BF_ASR_READY   0x00
/* W2ST command - BF Strong */
#define W2ST_COMMAND_BF_STRONG  0x01


/* W2ST command - BF toggle */
#define W2ST_COMMAND_BF_TOGGLE 0xAA
/* W2ST command - toggle BF On */
#define W2ST_COMMAND_BF_OFF   0x00
/* W2ST command - toggle BF Off */
#define W2ST_COMMAND_BF_ON  0x01

/* W2ST command - BF direction */
#define W2ST_COMMAND_BF_CHANGEDIR 0xBB
/* W2ST command - BF direction 1 */
#define W2ST_COMMAND_BF_DIR1  (uint8_t) 1
/* W2ST command - BF direction 2 */
#define W2ST_COMMAND_BF_DIR2  (uint8_t) 2
/* W2ST command - BF direction 3 */
#define W2ST_COMMAND_BF_DIR3  (uint8_t) 3
/* W2ST command - BF direction 4 */
#define W2ST_COMMAND_BF_DIR4  (uint8_t) 4
/* W2ST command - BF direction 5 */
#define W2ST_COMMAND_BF_DIR5  (uint8_t) 5
/* W2ST command - BF direction 6 */
#define W2ST_COMMAND_BF_DIR6  (uint8_t) 6
/* W2ST command - BF direction 7 */
#define W2ST_COMMAND_BF_DIR7  (uint8_t) 7
/* W2ST command - BF direction 8 */
#define W2ST_COMMAND_BF_DIR8  (uint8_t) 8

/* W2ST command - SL sensitivity */
#define W2ST_COMMAND_SL_SENSITIVITY 0xCC
/* W2ST command - SL sensitivity Low */
#define W2ST_COMMAND_SL_LOW  0x00
/* W2ST command - SL sensitivity High */
#define W2ST_COMMAND_SL_HIGH  0x01

/* BLE Characteristic connection control */
/* Environmental Data */
#define W2ST_CONNECT_ENV           (1   )

/* Acceleration/Gyroscope/Magneto */
#define W2ST_CONNECT_ACC_GYRO_MAG  (1<<2)
/* Mic */
#define W2ST_CONNECT_AUDIO_LEVEL   (1<<3)

/* Code for MotionFX integration - Start Section */
/* Quaternions */
#define W2ST_CONNECT_QUAT          (1<<4)
/* ECompass Feature */
#define W2ST_CONNECT_EC            (1<<5)
/* Code for MotionFX integration - End Section */

/* Code for MotionAR integration - Start Section */
#define W2ST_CONNECT_AR            (1<<6 )
/* Code for MotionAR integration - End Section */

/* Code for AcousticSL integration - Start Section */
#define W2ST_CONNECT_SL          (1<<7)
/* Code for AcousticSL integration - End Section */

/* Standard Terminal */
#define W2ST_CONNECT_STD_TERM      (1<<8)
/* Standard Error */
#define W2ST_CONNECT_STD_ERR       (1<<9)
/* HW Advance Features */
#define W2ST_CONNECT_ACC_EVENT     (1<<10)

/* Code for BlueVoice integration - Start Section */
#define W2ST_CONNECT_BV_AUDIO      (1<<11)
#define W2ST_CONNECT_BV_SYNC       (1<<12)
/* Code for BlueVoice integration - End Section */

/* Battery Features */
#define W2ST_CONNECT_BATTERY_FEATURES_EVENT      (1<<13)

/* Code for AcousticBF integration - Start Section */
#define W2ST_CONNECT_BF         (1<<14)
/* Code for AcousticBF integration - End Section */

#define W2ST_CHECK_CONNECTION(BleChar) ((ConnectionBleStatus&(BleChar)) ? 1 : 0)
#define W2ST_ON_CONNECTION(BleChar)    (ConnectionBleStatus|=(BleChar))
#define W2ST_OFF_CONNECTION(BleChar)   (ConnectionBleStatus&=(~BleChar))

#ifdef __cplusplus
}
#endif

#endif /* _SENSOR_SERVICE_H_ */

