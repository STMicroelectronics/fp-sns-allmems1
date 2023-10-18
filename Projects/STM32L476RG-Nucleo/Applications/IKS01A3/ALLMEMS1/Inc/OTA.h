/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    OTA.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.3.0
  * @date    30-June-2023
  * @brief   Over-the-Air Update API
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

/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _OTA_H_
#define _OTA_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported defines ---------------------------------------------------------*/

#define FLASH_FOTA_END          (((FLASH_END - FLASH_BASE) + 0x1) / 0x2)
#define OTA_MAX_PROG_SIZE       (FLASH_FOTA_END - 0x4000 - 0x8)

/* Exported functions ---------------------------------------------------------*/

/* API for preparing the Flash for receiving the Update. It defines also the Size of the Update and the CRC value aspected */
extern void StartUpdateFW(uint32_t SizeOfUpdate,uint32_t uwCRCValue);

/* API for storing chuck of data to Flash.
 * When it has received the total number of byte defined by StartUpdateFW,
 * it computes the CRC value and if it matches the aspected CRC value,
 * it writes the Magic Number in Flash for BootLoader */
extern int8_t UpdateFW(uint32_t *SizeOfUpdate,uint8_t * att_data, int32_t data_length,uint8_t WriteMagicNum);

/* API for checking the BootLoader compliance */
extern int8_t CheckBootLoaderCompliance(void);

#ifdef __cplusplus
}
#endif

#endif /* _OTA_H_ */

