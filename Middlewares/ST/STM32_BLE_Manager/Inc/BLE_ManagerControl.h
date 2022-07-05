/**
  ******************************************************************************
  * @file    BLE_ManagerControl.h
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version 1.1.0
  * @date    23-Dec-2021
  * @brief   Control Compilation defines
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
#ifndef _UUID_BLE_MANGER_CONTROL_H_
#define _UUID_BLE_MANGER_CONTROL_H_

#ifdef __cplusplus
 extern "C" {
#endif

#ifndef BLE_DEBUG_LEVEL
  #define BLE_DEBUG_LEVEL 3
#else /* BLE_DEBUG_LEVEL */
  #if ((BLE_DEBUG_LEVEL<0) || (BLE_DEBUG_LEVEL>3))
   #error "Valid 0<BLE_DEBUG_LEVEL <3"
  #endif /* Check the define value) */
#endif /* BLE_DEBUG_LEVEL */

#ifndef BLE_MANAGER_USED_PLATFORM
  #error "It's necessary to set the BLE_MANAGER_USED_PLATFORM Compilation Define"
#endif /* BLE_MANAGER_USED_PLATFORM */

   
#if ((!defined BLUENRG_LP) && (!defined BLUENRG_MS) && (!defined BLUENRG_1_2))
  #error "It's necessary to set the of the BlueNRG type: BLUENRG_LP/BLUENRG_MS/BLUENRG_1_2"
#endif /* ((!defined BLUENRG_LP) && (!defined BLUENRG_MS) && (!defined BLUENRG_1_2)) */

#ifdef __cplusplus
}
#endif

#endif /* _UUID_BLE_MANGER_CONTROL_H_ */

