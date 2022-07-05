/**
******************************************************************************
* @file    SensorTile.c
* @author  SRA - Central Labs
* @version v2.1.6
* @date    10-Feb-2022
* @brief   This file provides low level functionalities for SensorTile board
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

/* Includes ------------------------------------------------------------------*/
#include "SensorTile.h"


/** @addtogroup BSP
* @{
*/ 

/** @addtogroup SENSORTILE
* @{
*/

  /** @addtogroup SENSORTILE_LOW_LEVEL
  * @brief This file provides a set of low level firmware functions 
  * @{
  */

/** @defgroup SENSORTILE_LOW_LEVEL_Private_TypesDefinitions SENSORTILE_LOW_LEVEL Private Typedef
* @{
*/

/**
* @}
*/

/** @defgroup SENSORTILE_LOW_LEVEL__Private_Defines SENSORTILE_LOW_LEVEL Private Defines
* @{
*/


/**
* @brief SensorTile BSP Driver version number v2.1.6
*/
#define __SensorTile_BSP_VERSION_MAIN   (0x02) /*!< [31:24] main version */
#define __SensorTile_BSP_VERSION_SUB1   (0x01) /*!< [23:16] sub1 version */
#define __SensorTile_BSP_VERSION_SUB2   (0x04) /*!< [15:8]  sub2 version */
#define __SensorTile_BSP_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __SensorTile_BSP_VERSION         ((__SensorTile_BSP_VERSION_MAIN << 24)\
|(__SensorTile_BSP_VERSION_SUB1 << 16)\
  |(__SensorTile_BSP_VERSION_SUB2 << 8 )\
    |(__SensorTile_BSP_VERSION_RC))


/**
* @}
*/

/** @defgroup SENSORTILE_LOW_LEVEL_Private_Variables SENSORTILE_LOW_LEVEL Private Variables 
* @{
*/

SPI_HandleTypeDef SPI_SD_Handle;
DMA_HandleTypeDef hdma_tx;

GPIO_TypeDef* GPIO_PORT[LEDn] = {LED1_GPIO_PORT, LEDSWD_GPIO_PORT};
const uint32_t GPIO_PIN[LEDn] = {LED1_PIN, LEDSWD_PIN};

uint32_t SpixTimeout = SENSORTILE_SD_SPI_TIMEOUT_MAX;        /*<! Value of Timeout when SPI communication fails */

/**
* @}
*/


/** @defgroup SENSORTILE_LOW_LEVEL_Private_Functions SENSORTILE_LOW_LEVEL Private Functions
* @{
*/ 

static void               SD_IO_SPI_Init(void);/*high speed*/

static void               SD_IO_SPI_Init_LS(void); /*low speed*/

static void               SD_IO_SPI_Write(uint8_t Value);
static uint32_t           SD_IO_SPI_Read(void);
static void               SD_IO_SPI_Error (void);
static void               SD_IO_SPI_MspInit(SPI_HandleTypeDef *hspi);

/* Link functions for SD Card peripheral over SPI */
void                      SD_IO_Init(void);
void                      SD_IO_Init_LS(void);/*low speed*/

HAL_StatusTypeDef         SD_IO_WriteCmd(uint8_t Cmd, uint32_t Arg, uint8_t Crc, uint8_t Response);
uint8_t                   SD_IO_WriteCmd_wResp(uint8_t Cmd, uint32_t Arg, uint8_t Crc);
HAL_StatusTypeDef         SD_IO_WaitResponse(uint8_t Response);
void                      SD_IO_WriteDummy(void);
void                      SD_IO_WriteByte(uint8_t Data);
uint8_t                   SD_IO_ReadByte(void);
void                      SD_IO_WriteDMA(uint8_t *pData, uint16_t Size);

/**
* @}
*/



/** @defgroup SENSORTILE_LOW_LEVEL_Exported_Functions SENSORTILE_LOW_LEVEL Exported Functions
  * @{
  */


/**
* @brief  This method returns the STM32446E EVAL BSP Driver revision
* @param  None
* @retval version: 0xXYZR (8bits for each decimal, R for RC)
*/
uint32_t BSP_GetVersion(void)
{
  return __SensorTile_BSP_VERSION;
}


/**
* @brief  Configures LEDs.
* @param  Led: LED to be configured. 
*          This parameter can be one of the following values:
*            @arg  LED1
* @retval None
*/
void BSP_LED_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable VddIO2 for GPIOG  */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /* Enable the GPIO_LED clock */
  LEDx_GPIO_CLK_ENABLE(Led);
  
  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = GPIO_PIN[Led];
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  
  HAL_GPIO_Init(GPIO_PORT[Led], &GPIO_InitStruct);
}


/**
* @brief  DeInit LEDs.
* @param  Led: LED to be configured. 
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
*            @arg  LED4
* @note Led DeInit does not disable the GPIO clock nor disable the Mfx 
* @retval None
*/
void BSP_LED_DeInit(Led_TypeDef Led)
{
  
}

/**
* @brief  Turns selected LED On.
* @param  Led: LED to be set on 
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
*            @arg  LED4
* @retval None
*/
void BSP_LED_On(Led_TypeDef Led)
{
  if(Led == LED1)
  {
    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET);
  }
  else if (Led == LEDSWD)
  {
    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
  }
}

/**
* @brief  Turns selected LED Off. 
* @param  Led: LED to be set off
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
*            @arg  LED4
* @retval None
*/
void BSP_LED_Off(Led_TypeDef Led)
{
  if(Led == LED1)
  {
    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
  }
  else if (Led == LEDSWD)
  {
    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET);
  }
}

/**
* @brief  Toggles the selected LED.
* @param  Led: LED to be toggled
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
*            @arg  LED4
* @retval None
*/
void BSP_LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}


/******************************* SD SPI Routines**********************************/
/**
  * @brief  Initializes SPI MSP.
  * @param  None
  * @retval None
  */
static void SD_IO_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef  GPIO_InitStructure;  
  
  /* Enable VddIO2 for GPIOG  */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  
    /* Enable SPI clock */
  SENSORTILE_SD_SPI_CLK_ENABLE();
  
  /* Enable DMA clock */
  __HAL_RCC_DMA2_CLK_ENABLE();
  
  
  /*** Configure the GPIOs ***/  
  /* Enable GPIO clock */
  SENSORTILE_SD_SPI_SCK_GPIO_CLK_ENABLE();
  SENSORTILE_SD_SPI_MISO_MOSI_GPIO_CLK_ENABLE();
  
  /* configure SPI SCK */
  GPIO_InitStructure.Pin = SENSORTILE_SD_SPI_SCK_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Alternate = SENSORTILE_SD_SPI_SCK_AF;
  HAL_GPIO_Init(SENSORTILE_SD_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

  /* configure SPI MISO and MOSI */
  GPIO_InitStructure.Pin = (SENSORTILE_SD_SPI_MOSI_PIN);
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Alternate = SENSORTILE_SD_SPI_MISO_MOSI_AF;
  HAL_GPIO_Init(SENSORTILE_SD_SPI_MISO_MOSI_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = (SENSORTILE_SD_SPI_MISO_PIN );
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull  = GPIO_PULLUP;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Alternate = SENSORTILE_SD_SPI_MISO_MOSI_AF;
  HAL_GPIO_Init(SENSORTILE_SD_SPI_MISO_MOSI_GPIO_PORT, &GPIO_InitStructure);

  /*** Configure the SPI peripheral ***/ 

  /*##-3- Configure the DMA ##################################################*/
  /* Configure the DMA handler for Transmission process */
  hdma_tx.Instance                 = DMA2_Channel2;
  hdma_tx.Init.Request             = DMA_REQUEST_3;
  hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_tx.Init.Mode                = DMA_NORMAL;
  hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
  
  HAL_DMA_Init(&hdma_tx);
  
  /* Associate the initialized DMA handle to the the SPI handle */
  __HAL_LINKDMA(hspi, hdmatx, hdma_tx);
  
  /*##-4- Configure the NVIC for DMA #########################################*/ 
  /* NVIC configuration for DMA transfer complete interrupt (SPI1_TX) */
  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 2, 1);
  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);
    
}

/**
* @brief  Initializes SPI HAL. Low baundrate for initializazion phase.
* @param  None
* @retval None
*/
static void SD_IO_SPI_Init_LS(void)
{
  if(HAL_SPI_GetState(&SPI_SD_Handle) == HAL_SPI_STATE_RESET)
  {
    SPI_SD_Handle.Instance = SENSORTILE_SD_SPI;
    SPI_SD_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;    /* SPI baudrate is PCLK2/SPI_BaudRatePrescaler */
    SPI_SD_Handle.Init.Direction = SPI_DIRECTION_2LINES;
    SPI_SD_Handle.Init.CLKPhase = SPI_PHASE_1EDGE;
    SPI_SD_Handle.Init.CLKPolarity = SPI_POLARITY_LOW;
    SPI_SD_Handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    SPI_SD_Handle.Init.CRCPolynomial = 7;
    SPI_SD_Handle.Init.DataSize = SPI_DATASIZE_8BIT;
    SPI_SD_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    SPI_SD_Handle.Init.NSS = SPI_NSS_SOFT;
    SPI_SD_Handle.Init.TIMode = SPI_TIMODE_DISABLED;
    SPI_SD_Handle.Init.Mode = SPI_MODE_MASTER;
    
    SD_IO_SPI_MspInit(&SPI_SD_Handle);
    HAL_SPI_Init(&SPI_SD_Handle);
  }
}
/**
* @brief  Initializes SPI HAL. High baundrate
* @param  None
* @retval None
*/
static void SD_IO_SPI_Init(void)
{
  
  HAL_SPI_DeInit(&SPI_SD_Handle);
  
  HAL_Delay(1);
  
  if(HAL_SPI_GetState(&SPI_SD_Handle) == HAL_SPI_STATE_RESET)
  {
    SPI_SD_Handle.Instance = SENSORTILE_SD_SPI;
    SPI_SD_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;    /* SPI baudrate is PCLK2/SPI_BaudRatePrescaler */
    SPI_SD_Handle.Init.Direction = SPI_DIRECTION_2LINES;
    SPI_SD_Handle.Init.CLKPhase = SPI_PHASE_1EDGE;
    SPI_SD_Handle.Init.CLKPolarity = SPI_POLARITY_LOW;
    SPI_SD_Handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    SPI_SD_Handle.Init.CRCPolynomial = 7;
    SPI_SD_Handle.Init.DataSize = SPI_DATASIZE_8BIT;
    SPI_SD_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    SPI_SD_Handle.Init.NSS = SPI_NSS_SOFT;
    SPI_SD_Handle.Init.TIMode = SPI_TIMODE_DISABLED;
    SPI_SD_Handle.Init.Mode = SPI_MODE_MASTER;
    
    SD_IO_SPI_MspInit(&SPI_SD_Handle);
    HAL_SPI_Init(&SPI_SD_Handle);
  }
}

/**
  * @brief SPI Read 4 bytes from device
  * @param None
  * @retval Read data
*/
static uint32_t SD_IO_SPI_Read(void)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t readvalue = 0;
  uint32_t writevalue = 0xFFFFFFFF;
  
  status = HAL_SPI_TransmitReceive(&SPI_SD_Handle, (uint8_t*) &writevalue, (uint8_t*) &readvalue, 1, SpixTimeout);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    SD_IO_SPI_Error();
  }

  return readvalue;
}

/**
  * @brief SPI Write a byte to device
  * @param Value: value to be written
  * @retval None
  */
static void SD_IO_SPI_Write(uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_SPI_Transmit(&SPI_SD_Handle, (uint8_t*) &Value, 1, SpixTimeout);

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    SD_IO_SPI_Error();
  }
}

/**
  * @brief SPI error treatment function
  * @param None
  * @retval None
  */
static void SD_IO_SPI_Error (void)
{
  /* De-initialize the SPI communication BUS */
  HAL_SPI_DeInit(&SPI_SD_Handle);
  
  /* Re- Initiaize the SPI communication BUS */
  SD_IO_SPI_Init();
}

/******************************** LINK SD Card ********************************/

/**
  * @brief  Initializes the SD Card and put it into StandBy State (Ready for 
  *         data transfer).
  * @param  None
  * @retval None
  */
void SD_IO_Init(void)
{
  uint8_t counter;

  /* SD SPI Config */
  SD_IO_CS_Init();
  
  /* SD SPI Config */
  SD_IO_SPI_Init();
  
  SENSORTILE_SD_CS_HIGH();
  
  /* Send dummy byte 0xFF, 10 times with CS high */
  /* Rise CS and MOSI for 80 clocks cycles */
  for (counter = 0; counter <= 9; counter++)
  {
    /* Send dummy byte 0xFF */
    SD_IO_WriteByte(SENSORTILE_SD_DUMMY_BYTE);
  }
}

/**
  * @brief  Initializes the SD Card and put it into StandBy State (Ready for 
  *         data transfer). Low baundrate
  * @param  None
  * @retval None
  */
void SD_IO_Init_LS()
{ 
  uint8_t counter;

  /* SD SPI Config */
  SD_IO_CS_Init();
  
  /* SD SPI Config */
  SD_IO_SPI_Init_LS();
  
  SENSORTILE_SD_CS_HIGH();
  
  /* Send dummy byte 0xFF, 10 times with CS high */
  /* Rise CS and MOSI for 80 clocks cycles */
  for (counter = 0; counter <= 9; counter++)
  {
    /* Send dummy byte 0xFF */
    SD_IO_WriteByte(SENSORTILE_SD_DUMMY_BYTE);
  }
}

void SD_IO_CS_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* SD_CS_GPIO and SD_DETECT_GPIO Periph clock enable */
  SENSORTILE_SD_CS_GPIO_CLK_ENABLE();

  /* Configure SD_CS_PIN pin: SD Card CS pin */
  GPIO_InitStructure.Pin = SENSORTILE_SD_CS_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(SENSORTILE_SD_CS_GPIO_PORT, &GPIO_InitStructure);
}

void SD_IO_CS_DeInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Configure SD_CS_PIN pin: SD Card CS pin */
  GPIO_InitStructure.Pin = SENSORTILE_SD_CS_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(SENSORTILE_SD_CS_GPIO_PORT, &GPIO_InitStructure);
}


/**
  * @brief  Writes a byte on the SD.
  * @param  Data: byte to send.
  * @retval None
  */
void SD_IO_WriteByte(uint8_t Data)
{
  /* Send the byte */
  SD_IO_SPI_Write(Data);
  
}


/**
  * @brief  Writes a block by DMA on the SD.
  * @param  Data: byte to send.
  * @retval None
  */
void SD_IO_WriteDMA(uint8_t *pData, uint16_t Size)
{
  
  if(HAL_SPI_Transmit_DMA(&SPI_SD_Handle, (uint8_t*)pData, Size) != HAL_OK)
  {
    /* Transfer error in transmission process */
    while(1);
  }
  
}

/**
  * @brief  Reads a byte from the SD.
  * @param  None
  * @retval The received byte.
  */
uint8_t SD_IO_ReadByte(void)
{
  uint8_t data = 0;
  
  /* Get the received data */
  data = SD_IO_SPI_Read();

  /* Return the shifted data */
  return data;
}


/**
  * @brief  Sends 5 bytes command to the SD card and get response
  * @param  Cmd: The user expected command to send to SD card.
  * @param  Arg: The command argument.
  * @param  Crc: The CRC.
  * @param  Response: Expected response from the SD card
  * @retval  HAL_StatusTypeDef HAL Status
  */
uint8_t SD_IO_WriteCmd_wResp(uint8_t Cmd, uint32_t Arg, uint8_t Crc)
{
  uint32_t n = 0x00, resp;
  uint8_t frame[6];

  /* Prepare Frame to send */
  frame[0] = (Cmd | 0x40); /* Construct byte 1 */
  frame[1] = (uint8_t)(Arg >> 24); /* Construct byte 2 */
  frame[2] = (uint8_t)(Arg >> 16); /* Construct byte 3 */
  frame[3] = (uint8_t)(Arg >> 8); /* Construct byte 4 */
  frame[4] = (uint8_t)(Arg); /* Construct byte 5 */
  frame[5] = (Crc); /* Construct CRC: byte 6 */
  
  /* SD chip select low */
  SENSORTILE_SD_CS_LOW();
    
  /* Send Frame */
  for (n = 0; n < 6; n++)
  {
    SD_IO_WriteByte(frame[n]); /* Send the Cmd bytes */
  }

  n = 10; /* Wait for response (10 bytes max) */
  do {
    resp = SD_IO_ReadByte();
  } while ((resp & 0x80) && --n);
  
  return resp;	/* Return received response */
}


/**
  * @brief  Sends 5 bytes command to the SD card and get response
  * @param  Cmd: The user expected command to send to SD card.
  * @param  Arg: The command argument.
  * @param  Crc: The CRC.
  * @param  Response: Expected response from the SD card
  * @retval  HAL_StatusTypeDef HAL Status
  */
HAL_StatusTypeDef SD_IO_WriteCmd(uint8_t Cmd, uint32_t Arg, uint8_t Crc, uint8_t Response)
{
  uint32_t counter = 0x00;
  uint8_t frame[6];

  /* Prepare Frame to send */
  frame[0] = (Cmd | 0x40); /* Construct byte 1 */
  frame[1] = (uint8_t)(Arg >> 24); /* Construct byte 2 */
  frame[2] = (uint8_t)(Arg >> 16); /* Construct byte 3 */
  frame[3] = (uint8_t)(Arg >> 8); /* Construct byte 4 */
  frame[4] = (uint8_t)(Arg); /* Construct byte 5 */
  frame[5] = (Crc); /* Construct CRC: byte 6 */
  
  /* SD chip select low */
  SENSORTILE_SD_CS_LOW();
    
  /* Send Frame */
  for (counter = 0; counter < 6; counter++)
  {
    SD_IO_WriteByte(frame[counter]); /* Send the Cmd bytes */
  }

  if(Response != SENSORTILE_SD_NO_RESPONSE_EXPECTED)
  {
    return SD_IO_WaitResponse(Response);
  }
  
  return HAL_OK;
}

/**
  * @brief  Waits response from the SD card
  * @param  Response: Expected response from the SD card
  * @retval  HAL_StatusTypeDef HAL Status
  */
HAL_StatusTypeDef SD_IO_WaitResponse(uint8_t Response)
{
  uint32_t timeout = 0xFF0;//0x400;//

  /* Check if response is got or a timeout is happen */
  while ((SD_IO_ReadByte() != Response) && timeout)
  {
    timeout--;
  }

  if (timeout == 0)
  {
    /* After time out */
    return HAL_TIMEOUT;
  }
  else
  {
    /* Right response got */
    return HAL_OK;
  }
}

/**
  * @brief  Sends dummy byte with CS High
  * @param  None
  * @retval None
  */
void SD_IO_WriteDummy(void)
{
    /* SD chip select high */
    SENSORTILE_SD_CS_HIGH();
    
    /* Send Dummy byte 0xFF */
    SD_IO_WriteByte(SENSORTILE_SD_DUMMY_BYTE);
}


/**
  * @brief  Set all sensor Chip Select high. To be called before any SPI read/write
  * @param  None
  * @retval HAL_StatusTypeDef HAL Status
  */
uint8_t Sensor_IO_SPI_CS_Init_All(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Set all the pins before init to avoid glitch */
  BSP_LSM6DSM_CS_GPIO_CLK_ENABLE();
  BSP_LSM303AGR_M_CS_GPIO_CLK_ENABLE();
  BSP_LSM303AGR_X_CS_GPIO_CLK_ENABLE();
  BSP_LPS22HB_CS_GPIO_CLK_ENABLE();

  HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BSP_LSM303AGR_X_CS_PORT, BSP_LSM303AGR_X_CS_PIN,GPIO_PIN_SET);
  HAL_GPIO_WritePin(BSP_LSM303AGR_M_CS_PORT, BSP_LSM303AGR_M_CS_PIN,GPIO_PIN_SET);
  HAL_GPIO_WritePin(BSP_LPS22HB_CS_PORT, BSP_LPS22HB_CS_PIN, GPIO_PIN_SET);

  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

  GPIO_InitStruct.Pin = BSP_LSM6DSM_CS_PIN;
  HAL_GPIO_Init(BSP_LSM6DSM_CS_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = BSP_LSM303AGR_X_CS_PIN;
  HAL_GPIO_Init(BSP_LSM303AGR_X_CS_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(BSP_LSM303AGR_X_CS_PORT, BSP_LSM303AGR_X_CS_PIN,GPIO_PIN_SET);

  GPIO_InitStruct.Pin = BSP_LSM303AGR_M_CS_PIN;
  HAL_GPIO_Init(BSP_LSM303AGR_M_CS_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(BSP_LSM303AGR_M_CS_PORT, BSP_LSM303AGR_M_CS_PIN,GPIO_PIN_SET);

  GPIO_InitStruct.Pin = BSP_LPS22HB_CS_PIN;
  HAL_GPIO_Init(BSP_LPS22HB_CS_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(BSP_LPS22HB_CS_PORT, BSP_LPS22HB_CS_PIN, GPIO_PIN_SET);

  return HAL_OK;
}


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
