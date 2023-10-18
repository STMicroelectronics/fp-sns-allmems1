/**
  ******************************************************************************
  * @file    component.h
  * @author  MEMS Application Team
  * @version V1.1.0
  * @date    27-March-2023
  * @brief   This header file contains the functions prototypes common for all
  *          drivers
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
#ifndef __COMPONENT_H
#define __COMPONENT_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup COMMON COMMON
 * @{
 */

/** @addtogroup COMPONENT COMPONENT
 * @{
 */

/** @addtogroup COMPONENT_Public_Constants Public constants
 * @{
 */

/**
 * @brief  NULL pointer definition
 */
#ifndef NULL
#define NULL ( void * )0
#endif

/**
 * @}
 */

/** @addtogroup COMPONENT_Public_Types COMPONENT Public Types
 * @{
 */

/**
 * @brief  Component's Context structure definition.
 */
typedef struct
{

  /* Identity */
  uint8_t who_am_i;

  /* Configuration */
  uint8_t ifType;        /* 0 means I2C, 1 means SPI, etc. */
  uint8_t address;       /* Sensor I2C address (NOTE: Not a unique sensor ID). */
  uint8_t spiDevice;     /* Sensor Chip Select for SPI Bus */
  uint8_t instance;      /* Sensor instance (NOTE: Sensor ID unique only within its class). */
  uint8_t isInitialized; /* Sensor setup done. */
  uint8_t isEnabled;     /* Sensor ON. */
  uint8_t isCombo;       /* Combo sensor (component consists of more sensors). */

  /* Pointer to the Data */
  void *pData;

  /* Pointer to the Virtual Table */
  void *pVTable;
  /* Pointer to the Extended Virtual Table */
  void *pExtVTable;
} DrvContextTypeDef;



/**
 * @brief  Component's Status enumerator definition.
 */
typedef enum
{
  COMPONENT_OK = 0,
  COMPONENT_ERROR,
  COMPONENT_TIMEOUT,
  COMPONENT_NOT_IMPLEMENTED,
  COMPONENT_BATT_FAIL
} DrvStatusTypeDef;

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __COMPONENT_H */

