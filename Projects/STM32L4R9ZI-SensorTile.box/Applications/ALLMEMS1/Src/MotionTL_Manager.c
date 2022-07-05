/**
  ******************************************************************************
  * @file    MotionTL_Manager.c
  * @author  MEMS Software Solutions Team
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   This file contains Tilt Sensing interface functions
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
#include <math.h>
#include "TargetFeatures.h"

/* Private defines -----------------------------------------------------------*/
#define CAL_DATA_NUM_RECORDS  100  /* Number of accelerometer data (3 axes per data) to be taken */

/** @addtogroup MOTION_APPLICATIONS MOTION APPLICATIONS
 * @{
 */

/** @addtogroup TILT_SENSING TILT SENSING
 * @{
 */

/* Private defines -----------------------------------------------------------*/
#define CAL_DATA_NUM_RECORDS  100  /* Number of accelerometer data (3 axes per data) to be taken */

/* Private typedef -----------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 * @brief  Initialize the MotionTL engine
 * @param  None
 * @retval None
 */
void MotionTL_manager_init(void)
{
  char LibVersion[36];
  char acc_orientation[4];
  MTL_acc_cal_t acc_cal;

  acc_orientation[0] ='w';
  acc_orientation[1] ='s';
  acc_orientation[2] ='u';

  MotionTL_Initialize(MTL_MCU_STM32, acc_orientation);
  
  MotionTL_GetLibVersion(LibVersion);
  TargetBoardFeatures.MotionTLIsInitalized=1;
  ALLMEMS1_PRINTF("Initialized %s\r\n", LibVersion);

  /* Get calibration values from flash memory and set them in library if valid */
  MotionTL_manager_LoadCalValuesFromNVM(&acc_cal);

  if(MotionTL_SetCalValues(&acc_cal) != CAL_PASS)
  {
    ALLMEMS1_PRINTF("\t--> Set Accelerometer Calibration Failed\r\n");
    ALLMEMS1_PRINTF("\t--> ");
    ResetAccellerometerCalibrationInMemory();
  }
}

/**
 * @brief  Run Tilt Sensing algorithm
 * @param  data_in  Structure containing input data
 * @retval None
 */
void MotionTL_manager_run(MTL_input_t *data_in, uint64_t timestamp_ms, MTL_output_t *data_out)
{
  MotionTL_Update(data_in, timestamp_ms, data_out);
}


/**
  * @brief  Get the angle computation mode
  * @param  mode  angle computation mode
  * @retval None
  */
void MotionTL_manager_getAngleMode(MTL_angle_mode_t *mode)
{
  MTL_knobs_t knobs;
  MotionTL_GetKnobs(&knobs);
  *mode = knobs.mode;
}

/**
  * @brief  Set the angle computation mode
  * @param  mode  angle computation mode
  * @retval None
  */
void MotionTL_manager_setAngleMode(MTL_angle_mode_t mode)
{
  MTL_knobs_t knobs;
  MotionTL_GetKnobs(&knobs);
  knobs.mode = mode;
  MotionTL_SetKnobs(&knobs);
}

/**
 * @brief  Get the library version
 * @param  version  Library version string (must be array of 35 char)
 * @param  length  Library version string length
 * @retval None
 */
void MotionTL_manager_get_version(char *version, int *length)
{
  *length = (int)MotionTL_GetLibVersion(version);
}

/**
 * @brief  Calibrate accelerometer in specific position - collect data and pass them to library
 * @param  cal_position Calibration position the data belong to
 * @retval None
 */
void MotionTL_manager_calibratePosition(MTL_cal_position_t cal_position)
{
  float cal_data[CAL_DATA_NUM_RECORDS][3];

  if (CollectData(cal_data, CAL_DATA_NUM_RECORDS) == 0)
  {
    MotionTL_CalibratePosition(cal_data, CAL_DATA_NUM_RECORDS, cal_position);
  }
}

/**
 * @brief  Get accelerometer calibration values from library
 * @param  acc_cal Pointer to calibration values structure
 * @retval Enum with calibration result
 */
MTL_cal_result_t MotionTL_manager_getCalibrationValues(MTL_acc_cal_t *acc_cal)
{
  return MotionTL_GetCalValues(acc_cal);
}

/**
 * @brief  Set accelerometer calibration values into library
 * @param  acc_cal Pointer to calibration values structure
 * @retval None
 */
void MotionTL_manager_setCalibrationValues(MTL_acc_cal_t *acc_cal)
{
  MotionTL_SetCalValues(acc_cal);
}

/**
 * @brief  Get estimated measurement time
 * @param  time_s Pointer to time in [s]
 * @retval None
 */
void MotionTL_manager_getEstimatedMeasTime(float *time_s)
{
  GetEstimatedMeasTime(time_s, CAL_DATA_NUM_RECORDS);
}

/**
 * @brief  Load calibration values from memory
 * @param  acc_cal Pointer to structure with offset and gain values
 * @retval (1) fail, (0) success
 */
uint8_t MotionTL_manager_LoadCalValuesFromNVM(MTL_acc_cal_t *acc_cal)
{
  char Success=1;
  float data[6] = {0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f};
  
  Success= ReCallAccellerometerCalibrationFromMemory(6, (uint32_t *)data);
  
  for (int i = 0; i < 3; i++)
  {
    acc_cal->offset[i] = data[i];
    acc_cal->gain[i] = data[i + 3];
  }
  
  return Success;
}

/**
 * @brief  Save calibration values to memory
 * @param  acc_cal Pointer to calibration values structure
 * @retval (0) fail, (1) success
 */
uint8_t MotionTL_manager_SaveCalValuesInNVM(MTL_acc_cal_t *acc_cal)
{
  char Success=1;
  float data[6] = {0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f};

  for (int i = 0; i < 3; i++)
  {
    data[i] = acc_cal->offset[i];
    data[i + 3] = acc_cal->gain[i];
  }

  Success= SaveAccellerometerCalibrationToMemory(6, (uint32_t *)data);
  
  return Success;
}

/**
 * @}
 */

/**
 * @}
 */

