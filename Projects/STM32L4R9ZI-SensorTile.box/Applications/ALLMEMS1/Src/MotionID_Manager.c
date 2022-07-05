/**
  ******************************************************************************
  * @file    MotionID_Manager.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   This file includes Motion Intensity Detection interface functions
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

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "TargetFeatures.h"

/* Imported Variable -------------------------------------------------------------*/
extern float sensitivity_Mul;

/* exported Variable -------------------------------------------------------------*/
MID_output_t MIDCode = MID_ON_DESK; /* on desk */


/* Private defines -----------------------------------------------------------*/

/** @addtogroup  Drv_Sensor      Drv_Sensor
  * @{
  */

/** @addtogroup Drv_MotionID    Drv_MotionID
  * @{
  */   

/* Exported Functions --------------------------------------------------------*/
/**
* @brief  Run motion intensity detection algorithm. This function collects and 
*         scale data from accelerometer and calls the motion Intensity Detection Algo
* @param  SensorAxesRaw_t ACC_Value_Raw Acceleration value (x/y/z)
* @retval None
*/
void MotionID_manager_run(BSP_MOTION_SENSOR_AxesRaw_t ACC_Value_Raw)
{
  MID_input_t iDataIN;

  iDataIN.AccX = ACC_Value_Raw.x * sensitivity_Mul;
  iDataIN.AccY = ACC_Value_Raw.y * sensitivity_Mul;
  iDataIN.AccZ = ACC_Value_Raw.z * sensitivity_Mul;

  MotionID_Update(&iDataIN, &MIDCode);
}

/**
* @brief  Initialises MotionID algorithm
* @param  None
* @retval None
*/
void MotionID_manager_init(void)
{
  char LibVersion[36];
  
  MotionID_Initialize(MID_MCU_STM32);
  MotionID_GetLibVersion(LibVersion);
  
  TargetBoardFeatures.MotionIDIsInitalized=1;
  ALLMEMS1_PRINTF("Initialized %s\r\n", LibVersion);
}

/**
 * @}
 */ /* end of group  Drv_MotionID        Drv_MotionID*/

/**
 * @}
 */ /* end of group Drv_Sensor          Drv_Sensor*/

