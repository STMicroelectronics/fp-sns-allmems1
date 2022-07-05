/**
  ******************************************************************************
  * @file    ALLMEMS1_config.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   FP-SNS-ALLMEMS1 configuration
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
#ifndef __ALLMEMS1_CONFIG_H
#define __ALLMEMS1_CONFIG_H

/* Exported define ------------------------------------------------------------*/

/* Define The transmission interval in Multiple of 10ms for quaternions*/
#define QUAT_UPDATE_MUL_10MS 3

/* Define How Many quaterions you want to trasmit (from 1 to 3) */
#define SEND_N_QUATERNIONS 3

/* IMPORTANT 
The Sensors fusion runs at 100Hz so like MAXIMUM it possible to send:
1 quaternion every 10ms
2 quaternions every 20ms
3 quaternions every 30ms

if QUAT_UPDATE_MUL_10MS!=3, then SEND_N_QUATERNIONS must be ==1
*/

/*************** Debug Defines ******************/
/* For enabling connection and notification subscriptions debug */
#define ALLMEMS1_DEBUG_CONNECTION

/* For enabling trasmission for notified services (except for quaternions) */
#define ALLMEMS1_DEBUG_NOTIFY_TRAMISSION

/*************** Don't Change the following defines *************/

/*************************************/
/*  Remapping istance sensor defines */
/*************************************/
/* Motion Sensor Istance */
#define ACCELERO_INSTANCE        LSM6DSM_0
#define GYRO_INSTANCE            LSM6DSM_0
#define MAGNETO_INSTANCE         LSM303AGR_MAG_0

/* Environmental Sensor Istance */
#define TEMPERATURE_INSTANCE     LPS22HB_0
#define PRESSURE_INSTANCE        LPS22HB_0
/*************************************/

/* Package Version only numbers 0->9 */
#define ALLMEMS1_VERSION_MAJOR '4'
#define ALLMEMS1_VERSION_MINOR '2'
#define ALLMEMS1_VERSION_PATCH '0'

/* Define the ALLMEMS1 Name MUST be 7 charz long */
#define NAME_BLUEMS 'A','M','1','V',ALLMEMS1_VERSION_MAJOR,ALLMEMS1_VERSION_MINOR,ALLMEMS1_VERSION_PATCH

/* Package Name */
#define ALLMEMS1_PACKAGENAME "FP-SNS-ALLMEMS1"

/* Code for AcousticSL integration - Start Section */
#define M1_MIC 0
#define M2_MIC 1
#define M3_MIC 2
#define M4_MIC 3
   
#define TOP_RIGHT_MIC         M3_MIC
#define TOP_LEFT_MIC          M4_MIC
#define BOTTOM_RIGHT_MIC      M1_MIC
#define BOTTOM_LEFT_MIC       M2_MIC
  
/* Microphone distance */
#define DIAGONAL        57
#define SIDE            40
/* Code for AcousticSL integration - End Section */

/* Code for BlueVoice integration - Start Section */
#define BV_AUDIO_SAMPLING_FREQUENCY     8000
#define BV_AUDIO_VOLUME_VALUE           64
#define PCM_IN_SAMPLES_PER_MS           (((uint16_t)BV_AUDIO_SAMPLING_FREQUENCY)/1000)
#define AUDIO_IN_MS                     (1)       /*!< Number of ms of Audio given as input to the BlueVoice library.*/ 
#define BV_PCM_AUDIO_IN_SAMPLES         (PCM_IN_SAMPLES_PER_MS * AUDIO_IN_MS)
/* Code for BlueVoice integration - End Section */

#ifdef ALLMEMS1_ENABLE_PRINTF
    #include "usbd_cdc_interface.h"
    #define ALLMEMS1_PRINTF(...) {\
      char TmpBufferToWrite[256];\
      int32_t TmpBytesToWrite;\
      TmpBytesToWrite = sprintf( TmpBufferToWrite, __VA_ARGS__);\
      CDC_Fill_Buffer(( uint8_t * )TmpBufferToWrite, TmpBytesToWrite);\
    }
#else /* ALLMEMS1_ENABLE_PRINTF */
  #define ALLMEMS1_PRINTF(...)
  //#define ALLMEMS1_PRINTF(...) printf(__VA_ARGS__)
#endif /* ALLMEMS1_ENABLE_PRINTF */

/* STM32 Unique ID */
#ifdef USE_STM32F4XX_NUCLEO
#define STM32_UUID ((uint32_t *)0x1FFF7A10)
#endif /* USE_STM32F4XX_NUCLEO */

#ifdef USE_STM32L4XX_NUCLEO
#define STM32_UUID ((uint32_t *)0x1FFF7590)
#endif /* USE_STM32L4XX_NUCLEO */

/* STM32 MCU_ID */
#define STM32_MCU_ID ((uint32_t *)0xE0042000)
/* Control Section */

#if ((SEND_N_QUATERNIONS<1) || (SEND_N_QUATERNIONS>3))
  #error "SEND_N_QUATERNIONS could be only 1,2 or 3"
#endif

#if ((QUAT_UPDATE_MUL_10MS!=3) && (SEND_N_QUATERNIONS!=1))
  #error "If QUAT_UPDATE_MUL_10MS!=3 then SEND_N_QUATERNIONS must be = 1"
#endif

#endif /* __ALLMEMS1_CONFIG_H */

