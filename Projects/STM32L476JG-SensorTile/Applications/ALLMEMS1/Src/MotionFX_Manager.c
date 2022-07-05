/**
  ******************************************************************************
  * @file    MotionFX_Manager.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   Header for MotionFX_Manager.c
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
#include "main.h"

/* Private defines -----------------------------------------------------------*/
#define STATE_SIZE                      (size_t)(2432)

#define SAMPLETODISCARD                 15

#define GBIAS_ACC_TH_SC                 (2.0f*0.000765f)
#define GBIAS_GYRO_TH_SC                (2.0f*0.002f)
#define GBIAS_MAG_TH_SC                 (2.0f*0.001500f)

#define DECIMATION                      1U

/* Exported Variables -------------------------------------------------------------*/
MFX_output_t iDataOUT;
MFX_input_t iDataIN;

/* Imported Variables -------------------------------------------------------------*/
extern float sensitivity_Mul;

extern BSP_MOTION_SENSOR_Axes_t MAG_Offset;

/* Private Variables -------------------------------------------------------------*/
static MFX_knobs_t iKnobs;
static MFX_knobs_t *ipKnobs = &iKnobs;

static volatile int sampleToDiscard = SAMPLETODISCARD;
static int discardedCount = 0;

static uint8_t mfxstate[STATE_SIZE];

/* Private function prototypes -----------------------------------------------*/


/**
  * @brief  Initialize MotionFX engine
  * @retval None
  */
void MotionFX_manager_init(void)
{
  char LibVersion[36];
  
  if (STATE_SIZE < MotionFX_GetStateSize())
    Error_Handler();

  MotionFX_initialize((MFXState_t *)mfxstate);
  MotionFX_GetLibVersion(LibVersion);

  MotionFX_getKnobs(mfxstate, ipKnobs);

  ipKnobs->gbias_acc_th_sc = GBIAS_ACC_TH_SC;
  ipKnobs->gbias_gyro_th_sc = GBIAS_GYRO_TH_SC;
  ipKnobs->gbias_mag_th_sc = GBIAS_MAG_TH_SC;
  
  ipKnobs->acc_orientation[0] ='w';
  ipKnobs->acc_orientation[1] ='s';
  ipKnobs->acc_orientation[2] ='u';

  ipKnobs->gyro_orientation[0] = 'w';
  ipKnobs->gyro_orientation[1] = 's';
  ipKnobs->gyro_orientation[2] = 'u';
  
  ipKnobs->mag_orientation[0] = 's';
  ipKnobs->mag_orientation[1] = 'w';
  ipKnobs->mag_orientation[2] = 'u';

  ipKnobs->output_type = MFX_ENGINE_OUTPUT_ENU;
  ipKnobs->LMode = 1;
  ipKnobs->modx  = DECIMATION;
  
  MotionFX_setKnobs(mfxstate, ipKnobs);
  
  MotionFX_enable_6X(mfxstate, MFX_ENGINE_DISABLE);
  MotionFX_enable_9X(mfxstate, MFX_ENGINE_DISABLE);

  discardedCount = 0;

  TargetBoardFeatures.MotionFXIsInitalized=1;
  ALLMEMS1_PRINTF("Initialized %s\n\r", LibVersion);
}

/**
 * @brief  Run Motion Sensor Data Fusion algorithm
 * @param  data_in  Structure containing input data
 * @param  data_out Structure containing output data
 * @param  delta_time Delta time
 * @retval None
 */
void MotionFX_manager_run(MFX_input_t *data_in, MFX_output_t *data_out, float delta_time)
{
  if (discardedCount == sampleToDiscard)
  {
    MotionFX_propagate(mfxstate, data_out, data_in, &delta_time);
    MotionFX_update(mfxstate, data_out, data_in, &delta_time, NULL);
  }
  else
  {
    discardedCount++;
  }
}

/**
 * @brief  Start 6 axes MotionFX engine
 * @param  None
 * @retval None
 */
void MotionFX_manager_start_6X(void)
{
  MotionFX_enable_6X(mfxstate, MFX_ENGINE_ENABLE);
}

/**
 * @brief  Stop 6 axes MotionFX engine
 * @param  None
 * @retval None
 */
void MotionFX_manager_stop_6X(void)
{
  MotionFX_enable_6X(mfxstate, MFX_ENGINE_DISABLE);
}

/**
 * @brief  Start 9 axes MotionFX engine
 * @retval None
 */
void MotionFX_manager_start_9X(void)
{
  MotionFX_enable_9X(mfxstate, MFX_ENGINE_ENABLE);
}

/**
 * @brief  Stop 9 axes MotionFX engine
 * @retval None
 */
void MotionFX_manager_stop_9X(void)
{
  MotionFX_enable_9X(mfxstate, MFX_ENGINE_DISABLE);
}

/**
* @brief  Get MotionFX Engine data Out
* @param  None
* @retval MFX_output *iDataOUT MotionFX Engine data Out
*/
MFX_output_t* MotionFX_manager_getDataOUT(void)
{
  return &iDataOUT;
}

/**
* @brief  Get MotionFX Engine data IN
* @param  None
* @retval MFX_input *iDataIN MotionFX Engine data IN
*/
MFX_input_t* MotionFX_manager_getDataIN(void)
{
  return &iDataIN;
}

/**
 * @brief  Get the library version
 * @param  version  Library version string (must be array of 35 char)
 * @param  length  Library version string length
 * @retval None
 */
void MotionFX_manager_get_version(char *version, int *length)
{
  *length = (int)MotionFX_GetLibVersion(version);
}

/**
  * @brief  Run magnetometer calibration algorithm
  * @param  None
  * @retval None
  */
void MotionFX_manager_MagCal_run(MFX_MagCal_input_t *data_in, MFX_MagCal_output_t *data_out)
{
   MotionFX_MagCal_run(data_in);
   MotionFX_MagCal_getParams(data_out);
}


/**
 * @brief  Start magnetometer calibration
 * @param  None
 * @retval None
 */
void MotionFX_manager_MagCal_start(int sampletime)
{
  MotionFX_MagCal_init(sampletime, 1);
}


/**
 * @brief  Stop magnetometer calibration
 * @param  None
 * @retval None
 */
void MotionFX_manager_MagCal_stop(int sampletime)
{
  MotionFX_MagCal_init(sampletime, 0);
}
