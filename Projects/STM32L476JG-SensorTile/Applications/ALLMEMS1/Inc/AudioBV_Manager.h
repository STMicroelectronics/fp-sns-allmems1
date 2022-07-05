/**
  ******************************************************************************
  * @file    AudioBV_Manager.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   Header for AudioBV_Manager.c
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
#ifndef _AUDIOBV_MANAGER_H_
#define _AUDIOBV_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Exported Functions Prototypes ---------------------------------------------*/
extern void AudioBV_Manager_init(void);
extern void AudioProcess_BV(void);

extern void SW_BV_send_Callback(void);

#if (defined(STM32F401xE) || defined(STM32F446xx))
extern void BV_SetOutForBF(void);
extern void BV_SetInForBF(void);
#endif /* (defined(STM32F401xE) || defined(STM32F446xx)) */

#ifdef __cplusplus
}
#endif

#endif //_AUDIOBV_MANAGER_H_



