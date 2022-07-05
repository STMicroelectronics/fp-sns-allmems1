/**
  ******************************************************************************
  * @file    MotionID_Manager.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   Header for MotionID_Manager.c
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
#ifndef _MOTIONID_MANAGER_H_
#define _MOTIONID_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "motion_id.h"


/** @defgroup Drv_Sensor      Drv_Sensor
* @{
*/

/** @defgroup Drv_MotionAR            Drv_MotionID
* @brief    This file includes Motion Intensity Detection interface functions 
* @{
*/ 

/* Exported Functions Prototypes ---------------------------------------------*/
extern void MotionID_manager_init(void);
extern void MotionID_manager_run(MOTION_SENSOR_AxesRaw_t ACC_Value_Raw);

/* Exported Variables -------------------------------------------------------------*/
extern MID_output_t MIDCode;

/**
 * @}
 */  /* end of group Drv_MotionID */

/**
 * @}
 */  /* end of group Drv_Sensor */

#ifdef __cplusplus
}
#endif

#endif //_MOTIONID_MANAGER_H_


