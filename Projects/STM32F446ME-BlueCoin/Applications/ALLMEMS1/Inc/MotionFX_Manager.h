/**
  ******************************************************************************
  * @file    MotionFX_Manager.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   This file includes sensor fusion interface functions
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
  
#ifndef _MOTIONFX_MANAGER_H_
#define _MOTIONFX_MANAGER_H_

#include "motion_fx.h"
#include "TargetFeatures.h"

#define FROM_MGAUSS_TO_UT50 (0.1f/50.0f)
#define FROM_UT50_TO_MGAUSS 500.0f
#define FROM_MDPS_TO_DPS    0.001f

/* Delta time mSec for Deltafusion */
#define MOTION_FX_ENGINE_DELTATIME       0.01f

#define SAMPLE_PERIOD       ((uint8_t)10)   /* [ms] */

/* Exported functions ------------------------------------------------------- */
extern void MotionFX_manager_init(void);
extern void MotionFX_manager_run(MFX_input_t *data_in, MFX_output_t *data_out, float delta_time);
extern void MotionFX_manager_start_6X(void);
extern void MotionFX_manager_stop_6X(void);
extern void MotionFX_manager_start_9X(void);
extern void MotionFX_manager_stop_9X(void);
void MotionFX_manager_get_version(char *version, int *length);
extern void MotionFX_manager_MagCal_run(MFX_MagCal_input_t *data_in, MFX_MagCal_output_t *data_out);
extern void MotionFX_manager_MagCal_start(int sampletime);
extern void MotionFX_manager_MagCal_stop(int sampletime);

extern MFX_output_t* MotionFX_manager_getDataOUT(void);
extern MFX_input_t* MotionFX_manager_getDataIN(void);

#endif /* _MOTIONFX_MANAGER_H_ */


