/**
  ******************************************************************************
  * @file    MotionVC_Manager.c
  * @author  MEMS Software Solutions Team
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   This file contains Vertical Context interface functions
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

/** @addtogroup VERTICAL_CONTEXT VERTICAL CONTEXT
 * @{
 */

/* exported Variable -------------------------------------------------------------*/
MVC_context_t VerticalContextCode;

/* Private typedef -----------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 * @brief  Initialize the MotionVC engine
 * @param  None
 * @retval None
 */
void MotionVC_manager_init(void)
{
  char LibVersion[36];
  
  MotionVC_Initialize();
  MotionVC_GetLibVersion(LibVersion);
  
  TargetBoardFeatures.MotionVCIsInitalized=1;
  ALLMEMS1_PRINTF("Initialized %s\r\n", LibVersion);
}

/**
 * @brief  Run Vertical Context algorithm
 * @param  data_in  Structure containing input data
 * @param  data_out Structure containing output data
 * @retval None
 */
void MotionVC_manager_update(MVC_input_t *data_in, MVC_output_t *data_out)
{
  MotionVC_Update(data_in, data_out);
}

/**
 * @brief  Get the library version
 * @param  version  Library version string (must be array of 35 char)
 * @param  length  Library version string length
 * @retval None
 */
void MotionVC_manager_get_version(char *version, int *length)
{
  *length = (int)MotionVC_GetLibVersion(version);
}

/**
 * @}
 */

/**
 * @}
 */

