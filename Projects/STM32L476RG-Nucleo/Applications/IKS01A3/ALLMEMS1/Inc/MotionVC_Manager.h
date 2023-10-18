/**
  ******************************************************************************
  * @file    MotionVC_Manager.h
  * @author  MEMS Software Solutions Team
  * @version 4.3.0
  * @date    30-June-2023
  * @brief   This file contains definitions for the MotionVC_Manager.c file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MOTIONVC_MANAGER_H
#define MOTIONVC_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "string.h"
#include "motion_vc.h"
#include "main.h"

/* Extern variables ----------------------------------------------------------*/
/* Exported Macros -----------------------------------------------------------*/
/* Exported Types ------------------------------------------------------------*/
/* Imported Variables --------------------------------------------------------*/
/* Exported Functions Prototypes ---------------------------------------------*/
void MotionVC_manager_init(void);
void MotionVC_manager_update(MVC_input_t *data_in, MVC_output_t *data_out);
void MotionVC_manager_get_version(char *version, int *length);

#ifdef __cplusplus
}
#endif

#endif /* MOTIONVC_MANAGER_H */

