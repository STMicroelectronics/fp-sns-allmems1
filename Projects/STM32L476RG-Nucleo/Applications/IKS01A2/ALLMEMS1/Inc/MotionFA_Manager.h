/**
  ******************************************************************************
  * @file    MotionFA_Manager.h
  * @author  MEMS Software Solutions Team
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   This file contains definitions for the MotionFA_Manager.c file.
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
#ifndef MOTIONFA_MANAGER_H
#define MOTIONFA_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "string.h"
#include "motion_fa.h"
#include "main.h"

/* Extern variables ----------------------------------------------------------*/
/* Exported Macros -----------------------------------------------------------*/
/* Exported Types ------------------------------------------------------------*/
/* Imported Variables --------------------------------------------------------*/
/* Exported Functions Prototypes ---------------------------------------------*/
void MotionFA_manager_init(void);
void MotionFA_manager_run(MFA_input_t *data_in, MFA_output_t *data_out);
void MotionFA_manager_get_version(char *version, int *length);
void MotionFA_manager_reset_counter(void);
void MotionFA_manager_set_activity(MFA_activity_t activity);
void MotionFA_manager_get_activity(MFA_activity_t *activity);

#ifdef __cplusplus
}
#endif

#endif /* MOTIONFA_MANAGER_H */

