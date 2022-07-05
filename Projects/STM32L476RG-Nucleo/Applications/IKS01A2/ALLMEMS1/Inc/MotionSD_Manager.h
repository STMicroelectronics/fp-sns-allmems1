/**
  ******************************************************************************
  * @file    MotionSD_Manager.h
  * @author  MEMS Software Solutions Team
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   This file contains definitions for the MotionSD_Manager.c file
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
#ifndef MOTIONSD_MANAGER_H
#define MOTIONSD_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "string.h"
#include "motion_sd.h"
#include "main.h"

/* Extern Variables ----------------------------------------------------------*/
/* Exported Macros -----------------------------------------------------------*/
/* Exported Types ------------------------------------------------------------*/
/* Imported Variables --------------------------------------------------------*/
/* Exported Functions Prototypes ---------------------------------------------*/
void MotionSD_manager_init(void);
void MotionSD_manager_run(MSD_input_t *data_in, MSD_output_t *data_out);
void MotionSD_manager_reset(void);
void MotionSD_manager_get_version(char *version, int *length);

#ifdef __cplusplus
}
#endif

#endif /* MOTIONSD_MANAGER_H */

