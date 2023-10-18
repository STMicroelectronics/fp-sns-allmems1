/**
  ******************************************************************************
  * @file    MetaDataManager_Config.h 
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.3.0
  * @date    30-June-2023
  * @brief   Meta Data Manager Config Header File
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
#ifndef _META_DATA_MANAGER_CONFIG_H_
#define _META_DATA_MANAGER_CONFIG_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ---------------------------------------------------- */
#include <stdlib.h>

#include "stm32l4xx_hal.h"
#include "stm32l4xx_nucleo.h"

/* Exported Defines --------------------------------------------------------*/
#define STM32L4xx

/* Enable/Disable printf message */
#define ENABLE_MDM_PRINTF      1
/* Enable/Disable the DEBUG of Meta Data Manager */
#define ENABLE_MDM_DEBUG_PARSING      0
/* Exported define ------------------------------------------------------------*/
/* USER CODE BEGIN ED */
#if ENABLE_MDM_PRINTF
  #define MDM_PRINTF(...) printf(__VA_ARGS__)
  #if ENABLE_MDM_DEBUG_PARSING
    /* Define for enabling the DEBUG of Meta Data Manager */
    #define MDM_DEBUG_PARSING
  #endif /* ENABLE_MDM_DEBUG_PARSING */
#else /* ENABLE_MDM_PRINTF */
  #define MDM_PRINTF(...)
#endif /* ENABLE_MDM_PRINTF */
/* USER CODE END ED */

#ifdef __cplusplus
}
#endif

#endif /* _META_DATA_MANAGER_CONFIG_H_ */


