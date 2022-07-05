/**
  ******************************************************************************
  * @file    TargetPlatform.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   Initialization of the Target Platform
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
#include <stdio.h>
#include "TargetFeatures.h"
#include "main.h"

/* Exported variables ---------------------------------------------------------*/
TargetFeatures_t TargetBoardFeatures;

volatile float RMS_Ch[AUDIO_CHANNELS];
float DBNOISE_Value_Old_Ch[AUDIO_CHANNELS];
uint16_t PCM_Buffer[((AUDIO_CHANNELS*AUDIO_SAMPLING_FREQUENCY)/1000)  * N_MS ];
uint16_t PDM_Buffer[((((AUDIO_CHANNELS * AUDIO_SAMPLING_FREQUENCY) / 1000) * MAX_DECIMATION_FACTOR) / 16)* N_MS ];
uint32_t NumSample= ((AUDIO_CHANNELS*AUDIO_SAMPLING_FREQUENCY)/1000)  * N_MS;

/* Private variables ---------------------------------------------------------*/
BSP_AUDIO_Init_t MicParams;

/* Local function prototypes --------------------------------------------------*/
static void Init_MEM1_Sensors(void);
static void MX_GPIO_Init(void);
static void Init_MEMS_Mics(uint32_t AudioFreq, uint32_t AudioVolume);

/**
  * @brief  Initialize all the Target platform's Features
  * @param  None
  * @retval None
  */
void InitTargetPlatform(void)
{
  /* Configure Shutdown button */
  BSP_ShutDown_Init();
  
  /* if Watchdog_Refresh() is not called each 1,2s the BC will reset */
#ifdef USE_WATCHDOG
  BSP_Watchdog_Init(300);
#endif /* USE_WATCHDOG */
  
  /* Configure the User Button in EXTI Mode */
  BSP_PB_Init(BUTTON_1, BUTTON_MODE_EXTI);
  BSP_PB_Init(BUTTON_2, BUTTON_MODE_EXTI);
  
  /* Initialize Battery Measurement*/
  BSP_BatMS_Init();
    
  /* Initialize Charger Pin*/
  BSP_ChrgPin_Init();

  /* Enable Battery Measurement*/
  BSP_BatMS_Enable();
  
  /* Configure Power Voltage Detector(PVD) to detect if battery voltage is low */
  PVD_Config();
  
  /* Initialize LED */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED5);
  BSP_LED_Init(LED6);
  BSP_LED_Init(LED7);
  BSP_LED_Init(LED8);

  ALLMEMS1_PRINTF("\r\nSTMicroelectronics %s:\r\n"
         "\tVersion %c.%c.%c\r\n"
         "\tSTM32F446ME-BlueCoin board"
         "\r\n\n",
         ALLMEMS1_PACKAGENAME,
         ALLMEMS1_VERSION_MAJOR,ALLMEMS1_VERSION_MINOR,ALLMEMS1_VERSION_PATCH);

  /* Reset all the Target's Features */
  memset(&TargetBoardFeatures, 0, sizeof(TargetFeatures_t));
  /* Discovery and Intialize all the MEMS Target's Features */
  Init_MEM1_Sensors();
  
  /* Initialize Mic */
  Init_MEMS_Mics(AUDIO_SAMPLING_FREQUENCY, AUDIO_VOLUME_INPUT);

  ALLMEMS1_PRINTF("\n\r");
}

/** @brief Initialize all the MEMS1 sensors
 * @param None
 * @retval None
 */
static void Init_MEM1_Sensors(void)
{
  // THIS MUST BE CHECKED!!
  HAL_Delay(200); 

   /* Accelero & Gyro */
  if (BSP_MOTION_SENSOR_Init(ACCELERO_INSTANCE, MOTION_ACCELERO | MOTION_GYRO)==BSP_ERROR_NONE){
    ALLMEMS1_PRINTF("\tOK Accelero Sensor\n\r");
    ALLMEMS1_PRINTF("\tOK Gyroscope Sensor\n\r");
  } else {
    ALLMEMS1_PRINTF("\tError Accelero Sensor\n\r");
    ALLMEMS1_PRINTF("\tError Gyroscope Sensor\n\r");
    while(1);
  }
  
  TargetBoardFeatures.HWAdvanceFeatures = 1;

  /* Magneto */
  if(BSP_MOTION_SENSOR_Init(MAGNETO_INSTANCE, MOTION_MAGNETO)==BSP_ERROR_NONE){
    ALLMEMS1_PRINTF("\tOK Magneto Sensor\n\r");
  } else {
    ALLMEMS1_PRINTF("\tError Magneto Sensor\n\r");
    while(1);
  }

  /* Temperarure & Pressure */ 
  if(BSP_ENV_SENSOR_Init(TEMPERATURE_INSTANCE,ENV_TEMPERATURE| ENV_PRESSURE)==BSP_ERROR_NONE) {
    ALLMEMS1_PRINTF("\tOK Temperature and Pressure Sensor\n\r");
  } else {
    ALLMEMS1_PRINTF("\tError Temperature and Pressure Sensor\n\r");
  }

  /*  Enable all the sensors */
  if(BSP_MOTION_SENSOR_Enable(ACCELERO_INSTANCE, MOTION_ACCELERO)==BSP_ERROR_NONE)
    ALLMEMS1_PRINTF("\tEnabled Accelero Sensor\n\r");
  if(BSP_MOTION_SENSOR_Enable(GYRO_INSTANCE, MOTION_GYRO)==BSP_ERROR_NONE)
    ALLMEMS1_PRINTF("\tEnabled Gyroscope Sensor\n\r");
  if(BSP_MOTION_SENSOR_Enable(MAGNETO_INSTANCE, MOTION_MAGNETO)==BSP_ERROR_NONE)
    ALLMEMS1_PRINTF("\tEnabled Magneto Sensor\n\r");
  if(BSP_ENV_SENSOR_Enable(TEMPERATURE_INSTANCE, ENV_TEMPERATURE)==BSP_ERROR_NONE)
    ALLMEMS1_PRINTF("\tEnabled Temperature Sensor\n\r");
  if(BSP_ENV_SENSOR_Enable(PRESSURE_INSTANCE, ENV_PRESSURE)==BSP_ERROR_NONE)
    ALLMEMS1_PRINTF("\tEnabled Pressure Sensor\n\r");
  
  /* Enable interruption LSM6DSM_INT2_PIN  */
  MX_GPIO_Init();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  BSP_LSM6DSM_INT2_GPIO_CLK_ENABLE();

  /*Configure GPIO pins : PC4 */
  GPIO_InitStruct.Pin = BSP_LSM6DSM_INT2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(BSP_LSM6DSM_INT2_EXTI_IRQn, 0x08, 0x00);
  HAL_NVIC_EnableIRQ(BSP_LSM6DSM_INT2_EXTI_IRQn);
}

/** @brief Initialize all the MEMS's Microphones
 * @param None
 * @retval None
 */
void Init_MEMS_Mics(uint32_t AudioFreq, uint32_t AudioVolume)
{
  /* Initialize microphone acquisition */  
  MicParams.BitsPerSample = 16;
  MicParams.ChannelsNbr = AUDIO_CHANNELS;
  MicParams.Device = AUDIO_IN_DIGITAL_MIC;
  MicParams.SampleRate = AudioFreq;
  MicParams.Volume = AudioVolume;
  
  if( BSP_AUDIO_IN_Init(BSP_AUDIO_IN_INSTANCE, &MicParams) != BSP_ERROR_NONE )
  {
    ALLMEMS1_PRINTF("\nError Audio Init\r\n");
    
    while(1) {
      ;
    }
  }
  else
  {
    ALLMEMS1_PRINTF("\nOK Audio Init\t(Audio Freq.= %ld)\r\n", AudioFreq);
  }
  
  /* Set the volume level */
  if( BSP_AUDIO_IN_SetVolume(BSP_AUDIO_IN_INSTANCE, AudioVolume) != BSP_ERROR_NONE )
  {
    ALLMEMS1_PRINTF("Error Audio Volume\r\n\n");
    
    while(1) {
      ;
    }
  }
  else
  {
    ALLMEMS1_PRINTF("OK Audio Volume\t(Volume= %ld)\r\n", AudioVolume);
  }
}

/** @brief Initialize all the MEMS's Microphones
 * @param None
 * @retval None
 */
void InitMics(uint32_t AudioFreq, uint32_t AudioVolume, uint16_t AudioInSamples)
{
  Init_MEMS_Mics(AudioFreq, AudioVolume);
  BSP_AUDIO_IN_Record(BSP_AUDIO_IN_INSTANCE, (uint8_t *) PDM_Buffer, AudioInSamples*2);
}

/** @brief DeInitialize all the MEMS's Microphones
 * @param None
 * @retval None
 */
void DeInitMics(void)
{
  if( BSP_AUDIO_IN_Stop(BSP_AUDIO_IN_INSTANCE) != BSP_ERROR_NONE )
  {
    ALLMEMS1_PRINTF("Error Audio Stop\r\n");
    
    while(1) {
      ;
    }
  }
  else
    ALLMEMS1_PRINTF("OK Audio Stop\r\n");

  
  if( BSP_AUDIO_IN_DeInit(BSP_AUDIO_IN_INSTANCE) != BSP_ERROR_NONE )
  {
    ALLMEMS1_PRINTF("Error Audio DeInit\r\n");
    
    while(1) {
      ;
    }
  }
  else
    ALLMEMS1_PRINTF("OK Audio DeInit\r\n");
  
  __HAL_RCC_SPI3_FORCE_RESET();
  __HAL_RCC_SPI3_RELEASE_RESET();
}


/**
  * @brief  This function switches on the LED
  * @param  None
  * @retval None
  */
void LedOnTargetPlatform(void)
{
  BSP_LED_On(LED1);
  BSP_LED_On(LED3);
  BSP_LED_On(LED5);
  BSP_LED_On(LED7);
  
  TargetBoardFeatures.LedStatus=1;
}

/**
  * @brief  This function switches off the LED
  * @param  None
  * @retval None
  */
void LedOffTargetPlatform(void)
{
  BSP_LED_Off(LED1);
  BSP_LED_Off(LED3);
  BSP_LED_Off(LED5);
  BSP_LED_Off(LED7);
  
  TargetBoardFeatures.LedStatus=0;
}

/** @brief  This function toggles the LED
  * @param  None
  * @retval None
  */
void LedToggleTargetPlatform(void)
{
  BSP_LED_Toggle(LED1);
  BSP_LED_Toggle(LED3);
  BSP_LED_Toggle(LED5);
  BSP_LED_Toggle(LED7);
}

/**
 * @brief User function for Erasing the Flash data for MDM
 * @param None
 * @retval uint32_t Success/NotSuccess [1/0]
 */
uint32_t UserFunctionForErasingFlash(void)
{
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;
  uint32_t Success=1;

  EraseInitStruct.TypeErase = TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = FLASH_SECTOR_7;
  EraseInitStruct.NbSectors = 1;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK){
    /* Error occurred while sector erase. 
      User can add here some code to deal with this error. 
      SectorError will contain the faulty sector and then to know the code error on this sector,
      user can call function 'HAL_FLASH_GetError()'
      FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
    Success=0;
    Error_Handler();
  }

  /* Lock the Flash to disable the flash control register access (recommended
  to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();

  return Success;
}

/**
 * @brief User function for Saving the MDM  on the Flash
 * @param void *InitMetaDataVector Pointer to the MDM beginning
 * @param void *EndMetaDataVector Pointer to the MDM end
 * @retval uint32_t Success/NotSuccess [1/0]
 */
uint32_t UserFunctionForSavingFlash(void *InitMetaDataVector,void *EndMetaDataVector)
{
  uint32_t Success=1;

  /* Store in Flash Memory */
  uint32_t Address = MDM_FLASH_ADD;
  uint32_t *WriteIndex;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  for(WriteIndex =((uint32_t *) InitMetaDataVector); WriteIndex<((uint32_t *) EndMetaDataVector); WriteIndex++) {
    if (HAL_FLASH_Program(TYPEPROGRAM_WORD, Address,*WriteIndex) == HAL_OK){
      Address = Address + 4;
    } else {
      /* Error occurred while writing data in Flash memory.
         User can add here some code to deal with this error
         FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
      Error_Handler();
      Success=0;
    }
  }

  /* Lock the Flash to disable the flash control register access (recommended
   to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
 
  return Success;
}

