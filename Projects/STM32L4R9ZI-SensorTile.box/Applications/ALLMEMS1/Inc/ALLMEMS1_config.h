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
/* 
   Enabling the Secure Connection:
   0 disabled
   1 Enabled
*/
#define ENABLE_SECURE_CONNECTION        0
#define PIN_FOR_PAIRING                 123456

/* For enabling MotionCP algorithm */
#define ALLMEMS1_MOTIONCP

/* Uncomment for enabling the Secure Connection */
//#define ENABLE_SECURE_CONNECTION

#define PIN_FOR_PAIRING 123456

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
/* For enabling the printf on UART:
   it will introduce a delay of 10Seconds before starting
   the application for having time to open the Terminal
   for looking the ALLMEMS1 Initialization phase */
#define ALLMEMS1_ENABLE_PRINTF

/* For enabling connection and notification subscriptions debug */
#define ALLMEMS1_DEBUG_CONNECTION

/* For enabling trasmission for notified services (except for quaternions) */
#define ALLMEMS1_DEBUG_NOTIFY_TRAMISSION

/*************** Don't Change the following defines *************/

/*************************************/
/*  Remapping istance sensor defines */
/*************************************/
/* Motion Sensor Instance */
#define ACCELERO_INSTANCE       LSM6DSOX_0
#define GYRO_INSTANCE           LSM6DSOX_0
#define MAGNETO_INSTANCE        LIS2MDL_0

#define FF_TSH_250mg            LSM6DSOX_FF_TSH_250mg

/* Environmental Sensor Instance */
#define TEMPERATURE_INSTANCE_1  HTS221_0
#define HUMIDITY_INSTANCE       HTS221_0
#define TEMPERATURE_INSTANCE_2  LPS22HH_0
#define PRESSURE_INSTANCE       LPS22HH_0
/*************************************/

/* Package Version only numbers 0->9 */
#define ALLMEMS1_VERSION_MAJOR '4'
#define ALLMEMS1_VERSION_MINOR '2'
#define ALLMEMS1_VERSION_PATCH '0'

/* Define the ALLMEMS1 Name MUST be 7 char long */
#define NAME_BLUEMS 'A','M','1','V',ALLMEMS1_VERSION_MAJOR,ALLMEMS1_VERSION_MINOR,ALLMEMS1_VERSION_PATCH

/* Package Name */
#define ALLMEMS1_PACKAGENAME "FP-SNS-ALLMEMS1"

#ifdef ALLMEMS1_ENABLE_PRINTF
  #include "usbd_cdc_interface.h"
  #define ALLMEMS1_PRINTF(...) {\
    VComBytesToWrite = sprintf((char *)VComBufferToWrite, __VA_ARGS__);\
    CDC_Fill_Buffer(VComBufferToWrite, VComBytesToWrite);\
  }
#else /* ALLMEMS1_ENABLE_PRINTF */
  #define ALLMEMS1_PRINTF(...)
#endif /* ALLMEMS1_ENABLE_PRINTF */

/* STM32 Unique ID */
#define STM32_UUID ((uint32_t *)0x1FFF7590)

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

