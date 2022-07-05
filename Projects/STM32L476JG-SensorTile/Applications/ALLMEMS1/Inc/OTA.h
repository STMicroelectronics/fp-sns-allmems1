/**
  ******************************************************************************
  * @file    OTA.h 
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   Over-the-Air Update API
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
#ifndef _OTA_H_
#define _OTA_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported defines ---------------------------------------------------------*/

#ifdef STM32_SENSORTILEBOX
  /* 1008Kbytes Max Program Size */
  #define OTA_MAX_PROG_SIZE (0x100000-0x4000-8)
#else /* STM32_SENSORTILEBOX */
  /* 496Kbytes Max Program Size */
  #define OTA_MAX_PROG_SIZE (0x80000-0x4000-8)
#endif /* STM32_SENSORTILEBOX */

/* Exported functions ---------------------------------------------------------*/

/* API for preparing the Flash for receiving the Update. It defines also the Size of the Update and the CRC value aspected */
extern void StartUpdateFWBlueMS(uint32_t SizeOfUpdate,uint32_t uwCRCValue);
/* API for storing chuck of data to Flash.
 * When it has recived the total number of byte defined by StartUpdateFWBlueMS,
 * it computes the CRC value and if it matches the aspected CRC value,
 * it writes the Magic Number in Flash for BootLoader */
extern int8_t UpdateFWBlueMS(uint32_t *SizeOfUpdateBlueFW,uint8_t * att_data, int32_t data_length,uint8_t WriteMagicNum);

/* API for checking the BootLoader compliance */
extern int8_t CheckBootLoaderCompliance(void);

/* API for checking if it's the first Run after a FOTA */
extern int8_t CheckFirstRunAfterFOTA(void);

#ifdef __cplusplus
}
#endif

#endif /* _OTA_H_ */

