/**
  ******************************************************************************
  * @file    BLE_Manager_Conf_Template.h
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version 1.1.0
  * @date    23-Dec-2021
  * @brief   BLE Manager configuration template file.
  *          This file should be copied to the application folder and renamed
  *          to BLE_Manager_Conf.h.
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
#ifndef __BLE_MANAGER_CONF_H__
#define __BLE_MANAGER_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Exported define -----------------------------------------------------------*/

/* Enable one of the following defines */
//#define BLUENRG_MS
//#define BLUENRG_LP
//#define BLUENRG_1_2
  
/* Uncomment the following define for BlueST-SDK V2 */
//#define BLE_MANAGER_SDKV2

/* Select the used hardware platform for example:
 *
 * STEVAL-STWINKT1                      --> BLE_MANAGER_STEVAL_STWINKIT1_PLATFORM
 * 
 * Take a look on BLE_Manager.h for finding all the available platforms
*/
  
/* Identify the used hardware platform  */
#define BLE_MANAGER_USED_PLATFORM	BLE_MANAGER_UNDEF_PLATFORM

/* Define the Max dimesion of the Bluetooth characteristics for Debug Console services */
#define DEFAULT_MAX_STDOUT_CHAR_LEN     20
#define DEFAULT_MAX_STDERR_CHAR_LEN     20
   
#define BLE_MANAGER_MAX_ALLOCABLE_CHARS 16U

/* For enabling the capability to handle BlueNRG Congestion */
#define ACC_BLUENRG_CONGESTION
  
/* Define the Delay function to use inside the BLE Manager */
#define BLE_MANAGER_DELAY HAL_Delay
  
/****************** Malloc/Free **************************/
#define BLE_MallocFunction malloc
#define BLE_FreeFunction free
  
/*---------- Print messages from BLE Manager files at middleware level -------*/
/* Uncomment the following define for  enabling print debug messages */
#define BLE_MANAGER_DEBUG

#ifdef BLE_MANAGER_DEBUG

  /* Define the Verbosity level (1/2/3) */
  #define BLE_DEBUG_LEVEL 1

  /**
  * User can change here printf with a custom implementation.
  */

 #include <stdio.h>
 #define BLE_MANAGER_PRINTF(...)	printf(__VA_ARGS__)

#else
  #define BLE_MANAGER_PRINTF(...)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __BLE_MANAGER_CONF_H__*/

