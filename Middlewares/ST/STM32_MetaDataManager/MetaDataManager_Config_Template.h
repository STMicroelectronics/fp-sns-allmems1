/**
  ******************************************************************************
  * @file    MetaDataManager_Config_Template.h 
  * @author  System Research & Applications Team - Catania Lab.
  * @version 1.5.0
  * @date    18-Nov-2021
  * @brief   Meta Data Manager Config Header File
  *          This file should be copied to the application folder and renamed
  *          to MetaDataManager_Config.h.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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

/* Replace the header file names with the ones of the target platform */
#include "stm32yyxx_hal.h"
#include "stm32yyxx_nucleo.h"

#ifdef STM32WB5Mxx
/**
* Base @ of Page 201, 4 Kbytes
* User can change here meta data manager address base with a custom implementation for STM32WB5Mxx.
*/
#define MDM_ADDR_FLASH_PAGE_BASE ((uint32_t)0x080C9000)
#endif /* STM32WB5Mxx */

/*---------- Print messages from MetaDataManager files at middleware level -----------*/
#define MDM_DEBUG	0

#if MDM_DEBUG
	/**
	* User can change here printf with a custom implementation.
	* For example:
	* #include "PREDMNT1_config.h"
	* #define MDM_PRINTF	PREDMNT1_PRINTF
	*/
	#include <stdio.h>
	#define MDM_PRINTF(...)	printf(__VA_ARGS__)
#else
	#define MDM_PRINTF(...)
#endif

#ifdef __cplusplus
}
#endif

#endif /* _META_DATA_MANAGER_CONFIG_H_ */


