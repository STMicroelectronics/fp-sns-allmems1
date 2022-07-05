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
extern tBleStatus Environmental_Update(int32_t Press,uint16_t Hum,int16_t Temp2,int16_t Temp1);
extern tBleStatus AudioLevel_Update(uint16_t *Mic);
extern tBleStatus BatteryReport_Update(uint32_t soc, uint32_t voltage, int32_t current);

#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
tBleStatus SD_CardLoggingStatus_Notify(uint8_t Status, uint32_t FeatureMask,uint32_t TimeStep);
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */

/* Code for MotionAR integration - Start Section */
extern tBleStatus ActivityRec_Update(MAR_output_t ActivityCode);
/* Code for MotionAR integration - End Section */

#ifdef ALLMEMS1_MOTIONCP
extern tBleStatus CarryPosRec_Update(MCP_output_t CarryPositionCode);
#endif /*  ALLMEMS1_MOTIONCP */

#ifdef ALLMEMS1_MOTIONFA
extern tBleStatus FitnessActivities_Update(MFA_activity_t Activity, uint32_t Counter);
#endif /* ALLMEMS1_MOTIONFA */

/* Code for MotionFX integration - Start Section */
extern tBleStatus Quat_Update(BSP_MOTION_SENSOR_Axes_t *data);
extern tBleStatus ECompass_Update(uint16_t Angle);
/* Code for MotionFX integration - End Section */

/* Code for MotionGR integration - Start Section */
extern tBleStatus GestureRec_Update(MGR_output_t GestureCode);
/* Code for MotionGR integration - End Section */

/* Code for MotionID integration - Start Section */
extern tBleStatus IntensityDet_Update(MID_output_t MIDCode);
/* Code for MotionID integration - End Section */

/* Code for MotionTL integration - Start Section */
extern tBleStatus TiltMeasurement_Update(ANGLES_output_t TiltMeasurement);
/* Code for MotionTL integration - End Section */

/* Code for MotionPE & MotionSD & MotionVC integration - Start Section */
extern tBleStatus MotionAlgorithm_Update(uint8_t MotionCode);
/* Code for MotionPE & MotionSD & MotionVC integration - End Section */

extern tBleStatus Add_ConsoleW2ST_Service(void);
extern tBleStatus Stderr_Update(uint8_t *data,uint8_t length);
extern tBleStatus Term_Update(uint8_t *data,uint8_t length);

extern tBleStatus Add_ConfigW2ST_Service(void);
extern tBleStatus Config_Notify(uint32_t Feature,uint8_t Command,uint8_t val);

extern void     setConnectable(void);

void    HCI_Event_CB(void *pckt);

/* Exported variables --------------------------------------------------------*/
#define BLE_MANAGER_SDKV2

/* For enabling the capability to handle BlueNRG Congestion */
#define ACC_BLUENRG_CONGESTION

#ifdef ACC_BLUENRG_CONGESTION
/* For defining how many events skip when there is a congestion */
#define ACC_BLUENRG_CONGESTION_SKIP 30
#endif /* ACC_BLUENRG_CONGESTION */

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

/* W2ST command for asking the calibration status */
#define W2ST_COMMAND_CAL_STATUS 0xFF
/* W2ST command for resetting the calibration */
#define W2ST_COMMAND_CAL_RESET  0x00
/* W2ST command for stopping the calibration process */
#define W2ST_COMMAND_CAL_STOP   0x01

/* BLE Characteristic connection control */
/* Environmental Data */
#define W2ST_CONNECT_ENV           (1   )

/* Acceleration/Gyroscope/Magneto */
#define W2ST_CONNECT_ACC_GYRO_MAG  (1<<2)
/* Mic */
#define W2ST_CONNECT_AUDIO_LEVEL   (1<<3)

/* Code for MotionAR integration - Start Section */
#define W2ST_CONNECT_AR            (1<<4 )
/* Code for MotionAR integration - End Section */

#ifdef ALLMEMS1_MOTIONCP
#define W2ST_CONNECT_CP            (1<<5 )
#endif /*  ALLMEMS1_MOTIONCP */

#ifdef ALLMEMS1_MOTIONFA
#define W2ST_CONNECT_FA            (1<<6 )
#endif /* ALLMEMS1_MOTIONFA */

/* Code for MotionFX integration - Start Section */
/* Quaternions */
#define W2ST_CONNECT_QUAT          (1<<7)
/* ECompass Feature */
#define W2ST_CONNECT_EC            (1<<8)
/* Code for MotionFX integration - End Section */

/* Code for MotionGR integration - Start Section */
#define W2ST_CONNECT_GR            (1<<9 )
/* Code for MotionGR integration - End Section */

/* Code for MotionID integration - Start Section */
#define W2ST_CONNECT_ID            (1<<10)
/* Code for MotionID integration - End Section */

/* Code for MotionPE integration - Start Section */
#define W2ST_CONNECT_PE            (1<<11)
/* Code for MotionPE integration - End Section */

/* Code for MotionSD integration - Start Section */
#define W2ST_CONNECT_SD            (1<<12)
/* Code for MotionSD integration - End Section */

/* Code for MotionTL integration - Start Section */
#define W2ST_CONNECT_TL            (1<<13)
/* Code for MotionTL integration - End Section */

/* Code for MotionVC integration - Start Section */
#define W2ST_CONNECT_VC            (1<<14)
/* Code for MotionVC integration - End Section */

/* Code for MotionVC integration - Start Section */
#define W2ST_MOTION_ALGORITHM      (1<<15)
/* Code for MotionVC integration - End Section */

/* Standard Terminal */
#define W2ST_CONNECT_STD_TERM      (1<<16)
/* Standard Error */
#define W2ST_CONNECT_STD_ERR       (1<<17)
/* HW Advance Features */
#define W2ST_CONNECT_ACC_EVENT     (1<<18)

/* Code for BlueVoice integration - Start Section */
#define W2ST_CONNECT_BV_AUDIO      (1<<19)
#define W2ST_CONNECT_BV_SYNC       (1<<20)
/* Code for BlueVoice integration - End Section */

/* Gas Gouge Feature */
#define W2ST_CONNECT_GG_EVENT      (1<<21)

#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
#define W2ST_CONNECT_SD_CARD_LOGGING   (1<<22)
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */



#define W2ST_CHECK_CONNECTION(BleChar) ((ConnectionBleStatus&(BleChar)) ? 1 : 0)
#define W2ST_ON_CONNECTION(BleChar)    (ConnectionBleStatus|=(BleChar))
#define W2ST_OFF_CONNECTION(BleChar)   (ConnectionBleStatus&=(~BleChar))

#ifdef __cplusplus
}
#endif

#endif /* _SENSOR_SERVICE_H_ */


