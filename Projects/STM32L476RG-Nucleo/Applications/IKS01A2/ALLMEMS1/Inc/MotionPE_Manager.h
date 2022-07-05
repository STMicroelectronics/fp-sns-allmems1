/**
  ******************************************************************************
  * @file    MotionPE_Manager.h
  * @author  MEMS Software Solutions Team
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   This file contains definitions for the MotionPE_Manager.c file.
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
#ifndef MOTIONPE_MANAGER_H
#define MOTIONPE_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "string.h"
#include "motion_pe.h"
#include "main.h"

/* Extern variables ----------------------------------------------------------*/
/* Exported Macros -----------------------------------------------------------*/
/* Exported Types ------------------------------------------------------------*/
/* Imported Variables --------------------------------------------------------*/
/* Exported Functions Prototypes ---------------------------------------------*/
void MotionPE_manager_init(void);
void MotionPE_manager_run(MPE_input_t *data_in, MPE_output_t *data_out);
void MotionPE_manager_get_version(char *version, int *length);

#ifdef __cplusplus
}
#endif

#endif /* MOTIONPE_MANAGER_H */

