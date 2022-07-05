/**
  ******************************************************************************
  * @file    MotionTL_Manager.h
  * @author  MEMS Software Solutions Team
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   This file contains definitions for the MotionTL_Manager.c file
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
#ifndef MOTIONTL_MANAGER_H
#define MOTIONTL_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "string.h"
#include "motion_tl.h"
#include "main.h"

/* Extern variables ----------------------------------------------------------*/
/* Exported Macros -----------------------------------------------------------*/
/* Exported Types ------------------------------------------------------------*/
/* Either pitch, roll and gravity inclination or theta, psi and phi */
typedef struct
{
  float AnglesArray[3];
} ANGLES_output_t;

/* Imported Variables --------------------------------------------------------*/
/* Exported Functions Prototypes ---------------------------------------------*/
void MotionTL_manager_init(void);
void MotionTL_manager_run(MTL_input_t *data_in, uint64_t timestamp_ms, MTL_output_t *data_out);

void MotionTL_manager_getAngleMode(MTL_angle_mode_t *mode);
void MotionTL_manager_setAngleMode(MTL_angle_mode_t mode);

void MotionTL_manager_get_version(char *version, int *length);

void MotionTL_manager_calibratePosition(MTL_cal_position_t cal_position);
MTL_cal_result_t MotionTL_manager_getCalibrationValues(MTL_acc_cal_t *acc_cal);
void MotionTL_manager_setCalibrationValues(MTL_acc_cal_t *acc_cal);

void MotionTL_manager_getEstimatedMeasTime(float *time_s);

uint8_t MotionTL_manager_LoadCalValuesFromNVM(MTL_acc_cal_t *acc_cal);
uint8_t MotionTL_manager_SaveCalValuesInNVM(MTL_acc_cal_t *acc_cal);

/* Imported Functions Prototypes ---------------------------------------------*/
uint8_t CollectData(float cal_data[][3], uint32_t num_records);
void GetEstimatedMeasTime(float *time_s, uint32_t num_records);

#ifdef __cplusplus
}
#endif

#endif /* MOTIONTL_MANAGER_H */




