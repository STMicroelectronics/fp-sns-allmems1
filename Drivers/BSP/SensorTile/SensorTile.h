/**
******************************************************************************
* @file    SensorTile.h
* @author  SRA - Central Labs
* @version v2.1.6
* @date    10-Feb-2022
* @brief   This file contains definitions for SensorTile.c file
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

/* IMPORTANT: in order to compile with RevA following flag shall be defined  */
/* in the preprocessor options:  USE_SENSORTILE_REVA !!!!!!!!!! */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SENSORTILE_H
#define __SENSORTILE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "SensorTile_conf.h"
   

#define LSM303AGR_ACC_WHO_AM_I         0x33
#define LSM303AGR_MAG_WHO_AM_I         0x40
#define HTS221_WHO_AM_I_VAL         (uint8_t)0xBC

/** @addtogroup BSP
  * @{
  */

/** @addtogroup SENSORTILE
  * @{
  */
      
  /** @addtogroup SENSORTILE_LOW_LEVEL
  * @{
  */

/** @defgroup SENSORTILE_LOW_LEVEL_Exported_Types SENSORTILE_LOW_LEVEL Exported Types
  * @{
  */
typedef enum 
{
  LED1 = 0,
  LEDSWD = 1
}Led_TypeDef;


typedef enum
{
//  TEMPERATURE_SENSORS_AUTO = -1, /* Always first element and equal to -1 */
  LSM6DSM = 0,                  /* LSM6DSM. */
  LSM303AGR_X,                  /* LSM303AGR Accelerometer */
  LSM303AGR_M,                  /* LSM303AGR Magnetometer */
  LPS22HB                       /* LPS22HB */
} SPI_Device_t;

/**
  * @}
  */ 

/** @defgroup SENSORTILE_LOW_LEVEL_Exported_Constants SENSORTILE_LOW_LEVEL Exported Constants
  * @{
  */ 

#define LEDn                             2

#define LED1_PIN                         GPIO_PIN_12
#define LED1_GPIO_PORT                   GPIOG
#define LED1_GPIO_CLK_ENABLE()           __GPIOG_CLK_ENABLE()  
#define LED1_GPIO_CLK_DISABLE()          __GPIOG_CLK_DISABLE()  

#define LEDSWD_PIN                       GPIO_PIN_14
#define LEDSWD_GPIO_PORT                 GPIOA
#define LEDSWD_GPIO_CLK_ENABLE()         __GPIOA_CLK_ENABLE()  
#define LEDSWD_GPIO_CLK_DISABLE()        __GPIOA_CLK_DISABLE()  

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)  do{if((__INDEX__) == 0) LED1_GPIO_CLK_ENABLE(); \
                                            if((__INDEX__) == 1) LEDSWD_GPIO_CLK_ENABLE(); \
                                            }while(0)
												
#define LEDx_GPIO_CLK_DISABLE(__INDEX__) do{if((__INDEX__) == 0) LED1_GPIO_CLK_DISABLE(); \
                                            if((__INDEX__) == 1) LEDSWD_GPIO_CLK_DISABLE(); \
                                            }while(0)
                                              
                                              
/*##################### SD ###################################*/
/* Chip Select macro definition */
#define SENSORTILE_SD_CS_LOW()       HAL_GPIO_WritePin(SENSORTILE_SD_CS_GPIO_PORT, SENSORTILE_SD_CS_PIN, GPIO_PIN_RESET)
#define SENSORTILE_SD_CS_HIGH()      HAL_GPIO_WritePin(SENSORTILE_SD_CS_GPIO_PORT, SENSORTILE_SD_CS_PIN, GPIO_PIN_SET)

/**
  * @brief  SD Control Interface pins
  */
#define SENSORTILE_SD_CS_PIN                               GPIO_PIN_12
#define SENSORTILE_SD_CS_GPIO_PORT                         GPIOG
#define SENSORTILE_SD_CS_GPIO_CLK_ENABLE()                 __GPIOG_CLK_ENABLE()
#define SENSORTILE_SD_CS_GPIO_CLK_DISABLE()                __GPIOG_CLK_DISABLE()
      
#define SENSORTILE_SD_DUMMY_BYTE   0xFF
#define SENSORTILE_SD_NO_RESPONSE_EXPECTED 0x80
    
                                                
/*##################### SPI3 SensorTile ###################################*/
#define SENSORTILE_SD_SPI                               SPI3
#define SENSORTILE_SD_SPI_CLK_ENABLE()                  __SPI3_CLK_ENABLE()

#define SENSORTILE_SD_SPI_SCK_AF                        GPIO_AF6_SPI3
#define SENSORTILE_SD_SPI_SCK_GPIO_PORT                 GPIOG
#define SENSORTILE_SD_SPI_SCK_PIN                       GPIO_PIN_9
#define SENSORTILE_SD_SPI_SCK_GPIO_CLK_ENABLE()         __GPIOG_CLK_ENABLE()
#define SENSORTILE_SD_SPI_SCK_GPIO_CLK_DISABLE()        __GPIOG_CLK_DISABLE()

#define SENSORTILE_SD_SPI_MISO_MOSI_AF                  GPIO_AF6_SPI3
#define SENSORTILE_SD_SPI_MISO_MOSI_GPIO_PORT           GPIOG
#define SENSORTILE_SD_SPI_MISO_MOSI_GPIO_CLK_ENABLE()   __GPIOG_CLK_ENABLE()
#define SENSORTILE_SD_SPI_MISO_MOSI_GPIO_CLK_DISABLE()  __GPIOG_CLK_DISABLE()
#define SENSORTILE_SD_SPI_MISO_PIN                      GPIO_PIN_10
#define SENSORTILE_SD_SPI_MOSI_PIN                      GPIO_PIN_11
/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define SENSORTILE_SD_SPI_TIMEOUT_MAX                   1000

/**
  * @}
  */ 


/** @defgroup SENSORTILE_LOW_LEVEL_Exported_Macros SENSORTILE_LOW_LEVEL Exported Macros
  * @{
  */  
/**
  * @}
  */ 
/** @defgroup SENSORTILE_LOW_LEVEL_Exported_Functions SENSORTILE_LOW_LEVEL Exported Functions
  * @{
  */
uint32_t         BSP_GetVersion(void);  
void             BSP_LED_Init(Led_TypeDef Led);
void             BSP_LED_DeInit(Led_TypeDef Led);
void             BSP_LED_On(Led_TypeDef Led);
void             BSP_LED_Off(Led_TypeDef Led);
void             BSP_LED_Toggle(Led_TypeDef Led);


void SD_IO_CS_Init(void);
void SD_IO_CS_DeInit(void);

uint8_t Sensor_IO_SPI_CS_Init_All(void);

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

#endif /* __SENSORTILE_H */
