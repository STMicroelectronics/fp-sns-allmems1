/**
  ******************************************************************************
  * @file    usbd_cdc_interface.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   Header for usbd_cdc_interface.c file.
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
#ifndef USBD_CDC_IF_H
#define USBD_CDC_IF_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern USBD_CDC_ItfTypeDef  USBD_CDC_fops;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
uint8_t CDC_Fill_Buffer(uint8_t *Buf, uint32_t TotalLen);
uint8_t CDC_Next_Packet_Rx(void);
void CDC_PeriodElapsedCallback(void);

#endif /* USBD_CDC_IF_H */

