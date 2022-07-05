/**
  ******************************************************************************
  * @file    MotionPE_Manager.c
  * @author  MEMS Software Solutions Team
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   This file contains Pose Estimation interface functions
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
#include "TargetFeatures.h"

/** @addtogroup MOTION_APPLICATIONS MOTION APPLICATIONS
 * @{
 */

/** @addtogroup POSE_ESTIMATION POSE ESTIMATION
 * @{
 */

/* exported Variable -------------------------------------------------------------*/
MPE_output_t PoseEstimationCode;

/* Private typedef -----------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 * @brief  Initialize the MotionPE engine
 * @param  None
 * @retval None
 */
void MotionPE_manager_init(void)
{
  char LibVersion[36];
  char acc_orientation[3];

  MotionPE_Initialize();
  MotionPE_GetLibVersion(LibVersion);

  acc_orientation[0] ='w';
  acc_orientation[1] ='s';
  acc_orientation[2] ='u';

  MotionPE_SetOrientation_Acc(acc_orientation);

  TargetBoardFeatures.MotionPEIsInitalized=1;
  ALLMEMS1_PRINTF("Initialized %s\r\n", LibVersion);
}

/**
 * @brief  Run Pose Estimation algorithm
 * @param  data_in  Structure containing input data
 * @param  data_out Structure containing output data
 * @retval None
 */
void MotionPE_manager_run(MPE_input_t *data_in, MPE_output_t *data_out)
{
  MotionPE_Update(data_in, data_out);
}

/**
 * @brief  Get the library version
 * @param  version  Library version string (must be array of 35 char)
 * @param  length  Library version string length
 * @retval None
 */
void MotionPE_manager_get_version(char *version, int *length)
{
  *length = (int)MotionPE_GetLibVersion(version);
}

/**
 * @}
 */

/**
 * @}
 */

