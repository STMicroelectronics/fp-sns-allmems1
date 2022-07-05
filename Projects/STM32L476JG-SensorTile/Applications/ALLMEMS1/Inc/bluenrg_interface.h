/**
  ******************************************************************************
  * @file    bluenrg_interface.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   
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
#ifndef __BLUENRG_INTERFACE_H_
#define __BLUENRG_INTERFACE_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32_bluenrg_ble.h"
#include "hal_types.h"

void Hal_Write_Serial(const void* data1, const void* data2, int32_t n_bytes1,
                      int32_t n_bytes2);


#endif //__BLUENRG_INTERFACE_H_


