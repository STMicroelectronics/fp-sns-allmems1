/**
******************************************************************************
* @file    SensorTile_bus.c
* @author  SRA - Central Labs
* @version v2.1.6
* @date    10-Feb-2022
* @brief   This file provides BSP BUS IO Driver
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
#include "SensorTile_bus.h"
#include "SensorTile_conf.h"
#include "stm32l4xx_hal.h"

#define TIMEOUT_DURATION 1000
/** @addtogroup BSP
  * @{
  */
__weak HAL_StatusTypeDef MX_I2C3_Init(I2C_HandleTypeDef* hi2c);											
__weak HAL_StatusTypeDef MX_SPI2_Init(SPI_HandleTypeDef* hspi);
__weak HAL_StatusTypeDef MX_SPI1_Init(SPI_HandleTypeDef* hspi);

/** @addtogroup CUSTOM
  * @{
  */

/** @defgroup SENSORTILE_BUS CUSTOM BUS
  * @{
  */

/** @defgroup CUSTOM_Private_Variables BUS Private Variables
  * @{
  */
I2C_HandleTypeDef hbusi2c3;											
SPI_HandleTypeDef hbusspi2;
SPI_HandleTypeDef hbusspi1;						
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
static uint32_t IsI2C3MspCbValid = 0;										
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */				
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1)						
static uint32_t IsSPI2MspCbValid = 0;	
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */							
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1)						
static uint32_t IsSPI1MspCbValid = 0;	
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */			
/**
  * @}
  */

/** @defgroup CUSTOM_Private_FunctionPrototypes  Private Function Prototypes
  * @{
  */  

static void I2C3_MspInit(I2C_HandleTypeDef* i2cHandle); 
static void I2C3_MspDeInit(I2C_HandleTypeDef* i2cHandle);
static void SPI2_MspInit(SPI_HandleTypeDef* spiHandle); 
static void SPI2_MspDeInit(SPI_HandleTypeDef* spiHandle);
static void SPI1_MspInit(SPI_HandleTypeDef* spiHandle); 
static void SPI1_MspDeInit(SPI_HandleTypeDef* spiHandle);

/**
  * @}
  */

/** @defgroup CUSTOM_LOW_LEVEL_Private_Functions CUSTOM LOW LEVEL Private Functions
  * @{
  */ 
  
/** @defgroup SENSORTILE_BUS_Exported_Functions SENSORTILE_BUS Exported Functions
  * @{
  */   
/* BUS IO driver over I2C Peripheral */
/*******************************************************************************
                            BUS OPERATIONS OVER I2C
*******************************************************************************/
/**
  * @brief  Initialize a bus
  * @param None
  * @retval BSP status
  */
int32_t BSP_I2C3_Init(void) {

  int32_t ret = BSP_ERROR_NONE;
  
  hbusi2c3.Instance  = I2C3;

  if (HAL_I2C_GetState(&hbusi2c3) == HAL_I2C_STATE_RESET)
  {  
    #if (USE_HAL_I2C_REGISTER_CALLBACKS == 0)
      /* Init the I2C Msp */
      I2C3_MspInit(&hbusi2c3);
    #else
      if(IsI2C3MspCbValid == 0U)
      {
        if(BSP_I2C3_RegisterDefaultMspCallbacks() != BSP_ERROR_NONE)
        {
          return BSP_ERROR_MSP_FAILURE;
        }
      }
    #endif

    /* Init the I2C */
    if(MX_I2C3_Init(&hbusi2c3) != HAL_OK)
    {
      ret = BSP_ERROR_BUS_FAILURE;
    }
    else
    {
      ret = BSP_ERROR_NONE;
    }	
  }

  return ret;
}

/**
  * @brief  DeInitialize a bus
  * @param None
  * @retval BSP status
  */
int32_t BSP_I2C3_DeInit(void) {
  int32_t ret = BSP_ERROR_BUS_FAILURE;
  
  #if (USE_HAL_I2C_REGISTER_CALLBACKS == 0)
    /* DeInit the I2C */ 
    I2C3_MspDeInit(&hbusi2c3);
  #endif  
  
  if (HAL_I2C_DeInit(&hbusi2c3) == HAL_OK) {
    ret = BSP_ERROR_NONE;
  }
  
  return ret;
}

/**
  * @brief Return the status of the Bus
  *	@retval bool
  */
int32_t BSP_I2C3_IsReady(void) {
	return (HAL_I2C_GetState(&hbusi2c3) == HAL_I2C_STATE_READY);
}

/**
  * @brief  Write registers through bus (8 bits)
  * @param  Addr: Device address on Bus.
  * @param  Reg: The target register address to write
  * @param  Value: The target register value to be written
  * @retval BSP status
  */
int32_t BSP_I2C3_WriteReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t len) {
  int32_t ret = BSP_ERROR_BUS_FAILURE;

  if(HAL_I2C_Mem_Write(&hbusi2c3, (uint8_t)DevAddr,
                       (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT,
                       (uint8_t *)pData, len, TIMEOUT_DURATION) == HAL_OK)
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief  Read registers through a bus (8 bits)
  * @param  DevAddr: Device address on BUS
  * @param  Reg: The target register address to read
  * @retval BSP status
  */
int32_t  BSP_I2C3_ReadReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t len) {
  int32_t ret = BSP_ERROR_BUS_FAILURE;

  if (HAL_I2C_Mem_Read(&hbusi2c3, DevAddr, (uint16_t)Reg,
                       I2C_MEMADD_SIZE_8BIT, pData,
                       len, TIMEOUT_DURATION) == HAL_OK)
  {
    ret = HAL_OK;
  }

  return ret;
}

/**
  * @brief  Write registers through bus (16 bits)
  * @param  Addr: Device address on Bus.
  * @param  Reg: The target register address to write
  * @param  Value: The target register value to be written
  * @retval BSP status
  */
int32_t BSP_I2C3_WriteReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t len) {
  int32_t ret = BSP_ERROR_BUS_FAILURE;
  
  if(HAL_I2C_Mem_Write(&hbusi2c3, (uint8_t)DevAddr,
                       (uint16_t)Reg, I2C_MEMADD_SIZE_16BIT,
                       (uint8_t *)pData, len, TIMEOUT_DURATION) == HAL_OK)
  {
    ret = BSP_ERROR_NONE;
  }
 
  return ret;
}

/**
  * @brief  Read registers through a bus (16 bits)
  * @param  DevAddr: Device address on BUS
  * @param  Reg: The target register address to read
  * @retval BSP status
  */
int32_t  BSP_I2C3_ReadReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t len) {
  int32_t ret = BSP_ERROR_BUS_FAILURE;
  
  if (HAL_I2C_Mem_Read(&hbusi2c3, DevAddr, (uint16_t)Reg,
                       I2C_MEMADD_SIZE_16BIT, pData,
                       len, TIMEOUT_DURATION) == HAL_OK)
  {
    ret = BSP_ERROR_NONE;
  }
  
  return ret;
}

/**
  * @brief  Send an amount width data through bus (Simplex)
  * @param  DevAddr: Device address on Bus.
  * @param  pData: Data pointer
  * @param  Length: Data length
  * @retval BSP status
  */
int32_t BSP_I2C3_Send(uint16_t DevAddr, uint8_t *pData, uint16_t len) {
	int32_t ret = BSP_ERROR_BUS_FAILURE;

	if (HAL_I2C_Master_Transmit (&hbusi2c3, DevAddr, pData, len, TIMEOUT_DURATION) == HAL_OK) {
		ret = len;
	}

	return ret;
}

/**
  * @brief  Receive an amount of data through a bus (Simplex)
  * @param  DevAddr: Device address on Bus.
  * @param  pData: Data pointer
  * @param  Length: Data length
  * @retval BSP status
  */
int32_t BSP_I2C3_Recv(uint16_t DevAddr, uint8_t *pData, uint16_t len) {
	int32_t ret = BSP_ERROR_BUS_FAILURE;

	if (HAL_I2C_Master_Receive (&hbusi2c3, DevAddr, pData, len, TIMEOUT_DURATION) == HAL_OK) {
		ret = len;
	}

	return ret;
}

/**
  * @brief  Send and receive an amount of data through bus (Full duplex)
  * @param  DevAddr: Device address on Bus.
  * @param  pTxdata: Transmit data pointer
  * @param 	pRxdata: Receive data pointer
  * @param  Length: Data length
  * @retval BSP status
  */
int32_t BSP_I2C3_SendRecv(uint16_t DevAddr, uint8_t *pTxdata, uint8_t *pRxdata, uint16_t len) {
	int32_t ret = BSP_ERROR_BUS_FAILURE;
	
	/*
	 * Send and receive an amount of data through bus (Full duplex)
	 * I2C is Half-Duplex protocol
	 */
	if ((BSP_I2C3_Send(DevAddr, pTxdata, len) == len) && \
		(BSP_I2C3_Recv(DevAddr, pRxdata, len) == len))
			{
				ret = len;
			}
			
	return ret;
}
/* BUS IO driver over SPI Peripheral */
/*******************************************************************************
                            BUS OPERATIONS OVER SPI
*******************************************************************************/
/**
  * @brief  Initializes SPI HAL.
  * @retval None
  * @retval BSP status
  */
int32_t BSP_SPI2_Init(void) {
  int32_t ret = BSP_ERROR_NONE;
  
  hbusspi2.Instance  = SPI2;
  if (HAL_SPI_GetState(&hbusspi2) == HAL_SPI_STATE_RESET) 
  { 
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 0)
    /* Init the SPI Msp */
    SPI2_MspInit(&hbusspi2);
#else
    if(IsSPI2MspCbValid == 0U)
    {
      if(BSP_SPI2_RegisterDefaultMspCallbacks() != BSP_ERROR_NONE)
      {
        return BSP_ERROR_MSP_FAILURE;
      }
    }
#endif   
    
    /* Init the SPI */
    if (MX_SPI2_Init(&hbusspi2) != HAL_OK)
    {
      ret = BSP_ERROR_BUS_FAILURE;
    }
  } 

  return ret;
}

/**
  * @brief  DeInitializes SPI HAL.
  * @retval None
  * @retval BSP status
  */
int32_t BSP_SPI2_DeInit(void) {
  int32_t ret = BSP_ERROR_BUS_FAILURE;

#if (USE_HAL_SPI_REGISTER_CALLBACKS == 0)
  SPI2_MspDeInit(&hbusspi2);
#endif  
  
  if (HAL_SPI_DeInit(&hbusspi2) == HAL_OK) {
    ret = BSP_ERROR_NONE;
  }
  
  return ret;
}

/**
  * @brief  Write Data through SPI BUS.
  * @param  pData: Data
  * @param  len: Length of data in byte
  * @retval BSP status
  */
int32_t BSP_SPI2_Send(uint8_t *pData, uint16_t len)
{
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;
  
  if(HAL_SPI_Transmit(&hbusspi2, pData, len, TIMEOUT_DURATION) == HAL_OK)
  {
      ret = len;
  }
  return ret;
}

/**
  * @brief  Receive Data from SPI BUS
  * @param  pData: Data
  * @param  len: Length of data in byte
  * @retval BSP status
  */
int32_t  BSP_SPI2_Recv(uint8_t *pData, uint16_t len)
{
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;
  
  if(HAL_SPI_Receive(&hbusspi2, pData, len, TIMEOUT_DURATION) == HAL_OK)
  {
      ret = len;
  }
  return ret;
}

/**
  * @brief  Send and Receive data to/from SPI BUS (Full duplex)
  * @param  pData: Data
  * @param  len: Length of data in byte
  * @retval BSP status
  */
int32_t BSP_SPI2_SendRecv(uint8_t *pTxData, uint8_t *pRxData, uint16_t len)
{
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;
  
  if(HAL_SPI_TransmitReceive(&hbusspi2, pTxData, pRxData, len, TIMEOUT_DURATION) == HAL_OK)
  {
      ret = len;
  }
  return ret;
}


/**
  * @brief  Initializes SPI HAL.
  * @retval None
  * @retval BSP status
  */
int32_t BSP_SPI1_Init(void) {
  int32_t ret = BSP_ERROR_NONE;
  
  hbusspi1.Instance  = SPI1;
  if (HAL_SPI_GetState(&hbusspi1) == HAL_SPI_STATE_RESET) 
  { 
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 0)
    /* Init the SPI Msp */
    SPI1_MspInit(&hbusspi1);
#else
    if(IsSPI1MspCbValid == 0U)
    {
      if(BSP_SPI1_RegisterDefaultMspCallbacks() != BSP_ERROR_NONE)
      {
        return BSP_ERROR_MSP_FAILURE;
      }
    }
#endif   
    
    /* Init the SPI */
    if (MX_SPI1_Init(&hbusspi1) != HAL_OK)
    {
      ret = BSP_ERROR_BUS_FAILURE;
    }
  } 

  return ret;
}

/**
  * @brief  DeInitializes SPI HAL.
  * @retval None
  * @retval BSP status
  */
int32_t BSP_SPI1_DeInit(void) {
  int32_t ret = BSP_ERROR_BUS_FAILURE;

#if (USE_HAL_SPI_REGISTER_CALLBACKS == 0)
  SPI1_MspDeInit(&hbusspi1);
#endif  
  
  if (HAL_SPI_DeInit(&hbusspi1) == HAL_OK) {
    ret = BSP_ERROR_NONE;
  }
  
  return ret;
}

/**
  * @brief  Write Data through SPI BUS.
  * @param  pData: Data
  * @param  len: Length of data in byte
  * @retval BSP status
  */
int32_t BSP_SPI1_Send(uint8_t *pData, uint16_t len)
{
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;
  
  if(HAL_SPI_Transmit(&hbusspi1, pData, len, TIMEOUT_DURATION) == HAL_OK)
  {
      ret = len;
  }
  return ret;
}

/**
  * @brief  Receive Data from SPI BUS
  * @param  pData: Data
  * @param  len: Length of data in byte
  * @retval BSP status
  */
int32_t  BSP_SPI1_Recv(uint8_t *pData, uint16_t len)
{
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;
  
  if(HAL_SPI_Receive(&hbusspi1, pData, len, TIMEOUT_DURATION) == HAL_OK)
  {
      ret = len;
  }
  return ret;
}

/**
  * @brief  Send and Receive data to/from SPI BUS (Full duplex)
  * @param  pData: Data
  * @param  len: Length of data in byte
  * @retval BSP status
  */
int32_t BSP_SPI1_SendRecv(uint8_t *pTxData, uint8_t *pRxData, uint16_t len)
{
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;
  
  if(HAL_SPI_TransmitReceive(&hbusspi1, pTxData, pRxData, len, TIMEOUT_DURATION) == HAL_OK)
  {
      ret = len;
  }
  return ret;
}


#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)  
/**
  * @brief Register Default BSP I2C3 Bus Msp Callbacks
  * @retval BSP status
  */
int32_t BSP_I2C3_RegisterDefaultMspCallbacks (void)
{

  __HAL_I2C_RESET_HANDLE_STATE(&hbusi2c3);
  
  /* Register MspInit Callback */
  if (HAL_I2C_RegisterCallback(&hbusi2c3, HAL_I2C_MSPINIT_CB_ID, I2C3_MspInit)  != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }
  
  /* Register MspDeInit Callback */
  if (HAL_I2C_RegisterCallback(&hbusi2c3, HAL_I2C_MSPDEINIT_CB_ID, I2C3_MspDeInit) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }
  IsI2C3MspCbValid = 1;
  
  return BSP_ERROR_NONE;  
}

/**
  * @brief BSP I2C3 Bus Msp Callback registering
  * @param Callbacks     pointer to I2C3 MspInit/MspDeInit callback functions
  * @retval BSP status
  */
int32_t BSP_I2C3_RegisterMspCallbacks (BSP_I2C_Cb_t *Callbacks)
{
  /* Prevent unused argument(s) compilation warning */
  __HAL_I2C_RESET_HANDLE_STATE(&hbusi2c3);  
 
   /* Register MspInit Callback */
  if (HAL_I2C_RegisterCallback(&hbusi2c3, HAL_I2C_MSPINIT_CB_ID, Callbacks->pMspI2cInitCb)  != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }
  
  /* Register MspDeInit Callback */
  if (HAL_I2C_RegisterCallback(&hbusi2c3, HAL_I2C_MSPDEINIT_CB_ID, Callbacks->pMspI2cDeInitCb) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }
  
  IsI2C3MspCbValid = 1;
  
  return BSP_ERROR_NONE;  
}
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1)  
/**
  * @brief Register Default BSP SPI2 Bus Msp Callbacks
  * @retval BSP status
  */
int32_t BSP_SPI2_RegisterDefaultMspCallbacks (void)
{

  __HAL_SPI_RESET_HANDLE_STATE(&hbusspi2);
  
  /* Register MspInit Callback */
  if (HAL_SPI_RegisterCallback(&hbusspi2, HAL_SPI_MSPINIT_CB_ID, SPI2_MspInit)  != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }
  
  /* Register MspDeInit Callback */
  if (HAL_SPI_RegisterCallback(&hbusspi2, HAL_SPI_MSPDEINIT_CB_ID, SPI2_MspDeInit) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }
  IsSPI2MspCbValid = 1;
  
  return BSP_ERROR_NONE;  
}

/**
  * @brief BSP SPI2 Bus Msp Callback registering
  * @param Callbacks     pointer to SPI2 MspInit/MspDeInit callback functions
  * @retval BSP status
  */
int32_t BSP_SPI2_RegisterMspCallbacks (BSP_SPI_Cb_t *Callbacks)
{
  /* Prevent unused argument(s) compilation warning */
  __HAL_SPI_RESET_HANDLE_STATE(&hbusspi2);  
 
   /* Register MspInit Callback */
  if (HAL_SPI_RegisterCallback(&hbusspi2, HAL_SPI_MSPINIT_CB_ID, Callbacks->pMspSpiInitCb)  != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }
  
  /* Register MspDeInit Callback */
  if (HAL_SPI_RegisterCallback(&hbusspi2, HAL_SPI_MSPDEINIT_CB_ID, Callbacks->pMspSpiDeInitCb) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }
  
  IsSPI2MspCbValid = 1;
  
  return BSP_ERROR_NONE;  
}
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1)  
/**
  * @brief Register Default BSP SPI1 Bus Msp Callbacks
  * @retval BSP status
  */
int32_t BSP_SPI1_RegisterDefaultMspCallbacks (void)
{

  __HAL_SPI_RESET_HANDLE_STATE(&hbusspi1);
  
  /* Register MspInit Callback */
  if (HAL_SPI_RegisterCallback(&hbusspi1, HAL_SPI_MSPINIT_CB_ID, SPI1_MspInit)  != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }
  
  /* Register MspDeInit Callback */
  if (HAL_SPI_RegisterCallback(&hbusspi1, HAL_SPI_MSPDEINIT_CB_ID, SPI1_MspDeInit) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }
  IsSPI1MspCbValid = 1;
  
  return BSP_ERROR_NONE;  
}

/**
  * @brief BSP SPI1 Bus Msp Callback registering
  * @param Callbacks     pointer to SPI1 MspInit/MspDeInit callback functions
  * @retval BSP status
  */
int32_t BSP_SPI1_RegisterMspCallbacks (BSP_SPI_Cb_t *Callbacks)
{
  /* Prevent unused argument(s) compilation warning */
  __HAL_SPI_RESET_HANDLE_STATE(&hbusspi1);  
 
   /* Register MspInit Callback */
  if (HAL_SPI_RegisterCallback(&hbusspi1, HAL_SPI_MSPINIT_CB_ID, Callbacks->pMspSpiInitCb)  != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }
  
  /* Register MspDeInit Callback */
  if (HAL_SPI_RegisterCallback(&hbusspi1, HAL_SPI_MSPDEINIT_CB_ID, Callbacks->pMspSpiDeInitCb) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }
  
  IsSPI1MspCbValid = 1;
  
  return BSP_ERROR_NONE;  
}
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */


/**
  * @brief  Return system tick in ms
  * @retval Current HAL time base time stamp
  */
int32_t BSP_GetTick(void) {
  return HAL_GetTick();
}

/* SPI2 init function */ 

__weak HAL_StatusTypeDef MX_SPI2_Init(SPI_HandleTypeDef* hspi)
{
  HAL_StatusTypeDef ret = HAL_OK;
  hspi->Instance = SPI2;
  hspi->Init.Mode = SPI_MODE_MASTER;
  hspi->Init.Direction = SPI_DIRECTION_1LINE;
  hspi->Init.DataSize = SPI_DATASIZE_8BIT;
  hspi->Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi->Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi->Init.NSS = SPI_NSS_SOFT;
  hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi->Init.TIMode = SPI_TIMODE_DISABLE;
  hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi->Init.CRCPolynomial = 7;
  hspi->Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi->Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  
  if (HAL_SPI_Init(hspi) != HAL_OK)
  {
    ret = HAL_ERROR;
  }
  
  HAL_Delay(5);
  SPI_1LINE_TX(hspi);
  HAL_Delay(5);
  __HAL_SPI_ENABLE(hspi);
  
  return ret;
}

static void SPI2_MspInit(SPI_HandleTypeDef* spiHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* Enable Peripheral clock */
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
  
    /**SPI2 GPIO Configuration    
    PB15     ------> SPI2_MOSI
    PB13     ------> SPI2_SCK 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
}

static void SPI2_MspDeInit(SPI_HandleTypeDef* spiHandle)
{
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();
  
    /**SPI2 GPIO Configuration    
    PB15     ------> SPI2_MOSI
    PB14     ------> SPI2_MISO
    PB13     ------> SPI2_SCK 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_15|GPIO_PIN_13);

  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
}

/* I2C3 init function */ 

__weak HAL_StatusTypeDef MX_I2C3_Init(I2C_HandleTypeDef* hi2c)
{
  HAL_StatusTypeDef ret = HAL_OK;
  hi2c->Instance = I2C3;
  hi2c->Init.Timing = 0x10801541;
  hi2c->Init.OwnAddress1 = 0x33;
  hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  
  if (HAL_I2C_Init(hi2c) != HAL_OK)
  {
    ret = HAL_ERROR;
  }
  
  return ret;
}

static void I2C3_MspInit(I2C_HandleTypeDef* i2cHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  /* USER CODE BEGIN I2C3_MspInit 0 */
  RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;
  RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C3;
  RCC_PeriphCLKInitStruct.I2c3ClockSelection = RCC_I2C3CLKSOURCE_SYSCLK;
  if(HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct)!=HAL_OK)
  {
    while(1);
  }
  
  /* Enable I2C GPIO clocks */
  __GPIOC_CLK_ENABLE();
  /* USER CODE END I2C3_MspInit 0 */
  
  /**I2C3 GPIO Configuration    
  PC1     ------> I2C3_SDA
  PC0     ------> I2C3_SCL 
  */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  /* Peripheral clock enable */
  __HAL_RCC_I2C3_CLK_ENABLE();
  
  /* Force the I2C peripheral clock reset */
  __I2C3_FORCE_RESET();
  
  /* Release the I2C peripheral clock reset */
  __I2C3_RELEASE_RESET();
  
  /* Peripheral interrupt init */
  HAL_NVIC_SetPriority(I2C3_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);
  HAL_NVIC_SetPriority(I2C3_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C3_ER_IRQn);
  /* USER CODE BEGIN I2C3_MspInit 1 */
  
  /* USER CODE END I2C3_MspInit 1 */
}

static void I2C3_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{
  /* USER CODE BEGIN I2C3_MspDeInit 0 */

  /* USER CODE END I2C3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C3_CLK_DISABLE();
  
    /**I2C3 GPIO Configuration    
    PC1     ------> I2C3_SDA
    PC0     ------> I2C3_SCL 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1|GPIO_PIN_0);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(I2C3_EV_IRQn);

    HAL_NVIC_DisableIRQ(I2C3_ER_IRQn);

  /* USER CODE BEGIN I2C3_MspDeInit 1 */

  /* USER CODE END I2C3_MspDeInit 1 */
}


/* SPI1 init function */ 

__weak HAL_StatusTypeDef MX_SPI1_Init(SPI_HandleTypeDef* hspi)
{
  HAL_StatusTypeDef ret = HAL_OK;
  hspi->Instance = SPI1;
  hspi->Init.Mode = SPI_MODE_MASTER;
  hspi->Init.Direction = SPI_DIRECTION_2LINES;
  hspi->Init.DataSize = SPI_DATASIZE_8BIT;
  hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi->Init.NSS = SPI_NSS_SOFT;
  hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi->Init.TIMode = SPI_TIMODE_DISABLE;
  hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi->Init.CRCPolynomial = 7;
  hspi->Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi->Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(hspi) != HAL_OK)
  {
    ret = HAL_ERROR;
  }

  return ret;
}

static void SPI1_MspInit(SPI_HandleTypeDef* spiHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* Enable Peripheral clock */
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
  
    /**SPI1 GPIO Configuration    
    PA7     ------> SPI1_MOSI
    PA6     ------> SPI1_MISO
    PA5     ------> SPI1_SCK 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
}

static void SPI1_MspDeInit(SPI_HandleTypeDef* spiHandle)
{
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();
  
    /**SPI1 GPIO Configuration    
    PA7     ------> SPI1_MOSI
    PA6     ------> SPI1_MISO
    PA5     ------> SPI1_SCK 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5);

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
}
