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
/* For enabling MotionFA algorithm */
#define ALLMEMS1_MOTIONFA

/* For enabling MotionCP algorithm */
#ifndef ALLMEMS1_MOTIONFA
  #define ALLMEMS1_MOTIONCP
#endif /* ALLMEMS1_MOTIONFA */

/* 
   Enabling the Secure Connection:
   0 disabled
   1 Enabled
*/
#define ENABLE_SECURE_CONNECTION        0
#define PIN_FOR_PAIRING                 123456

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
/* For enabling the printf on UART - For Nucleo it's enable by default*/
#define ALLMEMS1_ENABLE_PRINTF

/* For enabling connection and notification subscriptions debug */
#define ALLMEMS1_DEBUG_CONNECTION

/* For enabling trasmission for notified services (except for quaternions) */
#define ALLMEMS1_DEBUG_NOTIFY_TRAMISSION

/*************** Don't Change the following defines *************/

/* Package Version only numbers 0->9 */
#define ALLMEMS1_VERSION_MAJOR '4'
#define ALLMEMS1_VERSION_MINOR '2'
#define ALLMEMS1_VERSION_PATCH '0'

/* Define the ALLMEMS1 Name MUST be 7 char long */
#define NAME_BLUEMS 'A','M','1','V',ALLMEMS1_VERSION_MAJOR,ALLMEMS1_VERSION_MINOR,ALLMEMS1_VERSION_PATCH

/* Package Name */
#define ALLMEMS1_PACKAGENAME "FP-SNS-ALLMEMS1"

/*************************************/
/*  Remapping istance sensor defines */
/*************************************/
/* Motion Sensor Istance */
#define ACCELERO_INSTANCE        IKS01A3_LSM6DSO_0
#define GYRO_INSTANCE            IKS01A3_LSM6DSO_0
#define MAGNETO_INSTANCE         IKS01A3_LIS2MDL_0

/* Environmental Sensor Istance */
#define TEMPERATURE_INSTANCE_1  IKS01A3_HTS221_0
#define HUMIDITY_INSTANCE        IKS01A3_HTS221_0
#define TEMPERATURE_INSTANCE_2  IKS01A3_LPS22HH_0
#define PRESSURE_INSTANCE        IKS01A3_LPS22HH_0
/*************************************/

/********************************/
/*  Remapping APIsensor defines */
/********************************/
/* Environmental Sensor API */
#define ENV_SENSOR_Init               IKS01A3_ENV_SENSOR_Init
#define ENV_SENSOR_Enable             IKS01A3_ENV_SENSOR_Enable
#define ENV_SENSOR_GetValue           IKS01A3_ENV_SENSOR_GetValue
#define ENV_SENSOR_GetOutputDataRate  IKS01A3_ENV_SENSOR_GetOutputDataRate
#define ENV_SENSOR_SetOutputDataRate  IKS01A3_ENV_SENSOR_SetOutputDataRate

/* Motion Sensor API */
#define MOTION_SENSOR_Init                    IKS01A3_MOTION_SENSOR_Init
#define MOTION_SENSOR_Enable                  IKS01A3_MOTION_SENSOR_Enable

#define MOTION_SENSOR_INT1_PIN                IKS01A3_MOTION_SENSOR_INT1_PIN

#define MOTION_SENSOR_AxesRaw_t               IKS01A3_MOTION_SENSOR_AxesRaw_t
#define MOTION_SENSOR_Axes_t                  IKS01A3_MOTION_SENSOR_Axes_t
#define MOTION_SENSOR_Event_Status_t          IKS01A3_MOTION_SENSOR_Event_Status_t

#define MOTION_SENSOR_GetAxesRaw              IKS01A3_MOTION_SENSOR_GetAxesRaw
#define MOTION_SENSOR_GetAxes                 IKS01A3_MOTION_SENSOR_GetAxes
#define MOTION_SENSOR_Get_Event_Status        IKS01A3_MOTION_SENSOR_Get_Event_Status

#define MOTION_SENSOR_GetSensitivity          IKS01A3_MOTION_SENSOR_GetSensitivity
#define MOTION_SENSOR_SetFullScale            IKS01A3_MOTION_SENSOR_SetFullScale

#define MOTION_SENSOR_SetOutputDataRate       IKS01A3_MOTION_SENSOR_SetOutputDataRate

#define MOTION_SENSOR_Get_DRDY_Status         IKS01A3_MOTION_SENSOR_Get_DRDY_Status

#define MOTION_SENSOR_GetOutputDataRate                 IKS01A3_MOTION_SENSOR_GetOutputDataRate
#define MOTION_SENSOR_Enable_Tilt_Detection             IKS01A3_MOTION_SENSOR_Enable_Tilt_Detection
#define MOTION_SENSOR_Disable_Tilt_Detection            IKS01A3_MOTION_SENSOR_Disable_Tilt_Detection
#define MOTION_SENSOR_Enable_Wake_Up_Detection          IKS01A3_MOTION_SENSOR_Enable_Wake_Up_Detection
#define MOTION_SENSOR_Disable_Wake_Up_Detection         IKS01A3_MOTION_SENSOR_Disable_Wake_Up_Detection
#define MOTION_SENSOR_Enable_Double_Tap_Detection       IKS01A3_MOTION_SENSOR_Enable_Double_Tap_Detection
#define MOTION_SENSOR_Disable_Double_Tap_Detection      IKS01A3_MOTION_SENSOR_Disable_Double_Tap_Detection
#define MOTION_SENSOR_Enable_Single_Tap_Detection       IKS01A3_MOTION_SENSOR_Enable_Single_Tap_Detection
#define MOTION_SENSOR_Disable_Single_Tap_Detection      IKS01A3_MOTION_SENSOR_Disable_Single_Tap_Detection
#define MOTION_SENSOR_Enable_Pedometer                  IKS01A3_MOTION_SENSOR_Enable_Pedometer
#define MOTION_SENSOR_Disable_Pedometer                 IKS01A3_MOTION_SENSOR_Disable_Pedometer
#define MOTION_SENSOR_Reset_Step_Counter                IKS01A3_MOTION_SENSOR_Reset_Step_Counter
#define MOTION_SENSOR_Get_Step_Count                    IKS01A3_MOTION_SENSOR_Get_Step_Count
#define MOTION_SENSOR_Enable_Free_Fall_Detection        IKS01A3_MOTION_SENSOR_Enable_Free_Fall_Detection
#define MOTION_SENSOR_Disable_Free_Fall_Detection       IKS01A3_MOTION_SENSOR_Disable_Free_Fall_Detection
#define MOTION_SENSOR_Set_Free_Fall_Threshold           IKS01A3_MOTION_SENSOR_Set_Free_Fall_Threshold
#define MOTION_SENSOR_Enable_6D_Orientation             IKS01A3_MOTION_SENSOR_Enable_6D_Orientation
#define MOTION_SENSOR_Disable_6D_Orientation            IKS01A3_MOTION_SENSOR_Disable_6D_Orientation
#define MOTION_SENSOR_Get_6D_Orientation_XL             IKS01A3_MOTION_SENSOR_Get_6D_Orientation_XL
#define MOTION_SENSOR_Get_6D_Orientation_XH             IKS01A3_MOTION_SENSOR_Get_6D_Orientation_XH
#define MOTION_SENSOR_Get_6D_Orientation_YL             IKS01A3_MOTION_SENSOR_Get_6D_Orientation_YL
#define MOTION_SENSOR_Get_6D_Orientation_YH             IKS01A3_MOTION_SENSOR_Get_6D_Orientation_YH
#define MOTION_SENSOR_Get_6D_Orientation_ZL             IKS01A3_MOTION_SENSOR_Get_6D_Orientation_ZL
#define MOTION_SENSOR_Get_6D_Orientation_ZH             IKS01A3_MOTION_SENSOR_Get_6D_Orientation_ZH 
/********************************/

/* Code for Acoustic Source Localization integration - Start Section */  
#define M1_EXT_B 0
#define M4_EXT_B 1
   
/* Microphone distance */
#define SIDE     147
/* Code for Acoustic Source Localization integration - End Section */   

#ifdef ALLMEMS1_ENABLE_PRINTF
  #define ALLMEMS1_PRINTF(...) printf(__VA_ARGS__)
#else /* ALLMEMS1_ENABLE_PRINTF */
  #define ALLMEMS1_PRINTF(...)
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

