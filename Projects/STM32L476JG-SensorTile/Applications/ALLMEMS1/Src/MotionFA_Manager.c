/**
  ******************************************************************************
  * @file    MotionFA_Manager.c
  * @author  MEMS Software Solutions Team
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   This file contains Fitness Activities interface functions
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

#ifdef ALLMEMS1_MOTIONFA
/** @addtogroup MOTION_APPLICATIONS MOTION APPLICATIONS
 * @{
 */

/** @addtogroup FITNESS_ACTIVITIES FITNESS ACTIVITIES
 * @{
 */

/* Private variables ---------------------------------------------------------*/
MFA_activity_t SelectedActivity = MFA_BICEPCURL;

/* Private typedef -----------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 * @brief  Initialisze MotionFA algorithm
 * @param  None
 * @retval None
 */
void MotionFA_manager_init(void)
{
  char LibVersion[36];
  char acc_orientation[3];
  char gyr_orientation[3];

  MotionFA_Initialize();
  MotionFA_GetLibVersion(LibVersion);

  acc_orientation[0] ='w';
  acc_orientation[1] ='s';
  acc_orientation[2] ='u';

  gyr_orientation[0] = 'w';
  gyr_orientation[1] = 's';
  gyr_orientation[2] = 'u';

  MotionFA_SetOrientation_Acc(acc_orientation);
  MotionFA_SetOrientation_Gyr(gyr_orientation);
  
  TargetBoardFeatures.MotionFAIsInitalized=1;
  ALLMEMS1_PRINTF("Initialized %s\r\n", LibVersion);
}

/**
 * @brief  Run Fitness Activities algorithm
 * @param  data_in Structure containing input data
 * @param  data_out Structure containing ouput data
 * @retval None
 */
void MotionFA_manager_run(MFA_input_t *data_in, MFA_output_t *data_out)
{
  switch (SelectedActivity)
  {
    case MFA_BICEPCURL:
      MotionFA_BicepCurl_Update(data_in, data_out);
      break;

    case MFA_SQUAT:
      MotionFA_Squat_Update(data_in, data_out);
      break;

    case MFA_PUSHUP:
      MotionFA_Pushup_Update(data_in, data_out);
      break;

    default:
      break;
  }
}

/**
 * @brief  Get the library version
 * @param  version  Library version string (must be array of 35 char)
 * @param  length  Library version string length
 * @retval None
 */
void MotionFA_manager_get_version(char *version, int *length)
{
  *length = (int)MotionFA_GetLibVersion(version);
}

/**
 * @brief  Reset counter or current activity
 * @param  None
 * @retval None
 */
void MotionFA_manager_reset_counter(void)
{
  switch (SelectedActivity)
  {
    case MFA_BICEPCURL:
      MotionFA_BicepCurl_Reset();
      break;

    case MFA_SQUAT:
      MotionFA_Squat_Reset();
      break;

    case MFA_PUSHUP:
      MotionFA_Pushup_Reset();
      break;

    default:
      break;
  }
}

/**
 * @brief  Set activity
 * @param  activity Activity type
 * @retval None
 */
void MotionFA_manager_set_activity(MFA_activity_t activity)
{
  SelectedActivity = activity;
}

/**
 * @brief  Get activity
 * @param  None
 * @retval activity type
 */
void MotionFA_manager_get_activity(MFA_activity_t *activity)
{
  *activity = SelectedActivity;
}

/**
 * @}
 */

/**
 * @}
 */

#endif /* ALLMEMS1_MOTIONFA */

