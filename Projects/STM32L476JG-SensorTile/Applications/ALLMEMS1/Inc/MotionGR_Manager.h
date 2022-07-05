 /**
  ******************************************************************************
  * @file    MotionGR_Manager.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   Header for MotionGR_Manager.c
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
#ifndef _MOTIONGR_MANAGER_H_
#define _MOTIONGR_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "motion_gr.h"


/** @defgroup Drv_Sensor      Drv_Sensor
* @{
*/

/** @defgroup Drv_MotionGR            Drv_MotionGR
* @brief    This file includes Gesture Recognition interface functions 
* @{
*/ 

/* Exported Functions Prototypes ---------------------------------------------*/
extern void MotionGR_manager_init(void);
extern void MotionGR_manager_run(BSP_MOTION_SENSOR_AxesRaw_t ACC_Value_Raw);


/**
 * @}
 */  /* end of group Drv_MotionGR */

/**
 * @}
 */  /* end of group Drv_Sensor */

#ifdef __cplusplus
}
#endif

#endif //_MOTIONGR_MANAGER_H_



