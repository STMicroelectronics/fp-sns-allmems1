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
#include "HWAdvanceFeatures.h"
#include "main.h"

#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
#include "datalog_application.h"
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */

#ifdef ALLMEMS1_ENABLE_PRINTF
  #include "usbd_core.h"
  #include "usbd_cdc.h"
  #include "usbd_cdc_interface.h"
#endif /* ALLMEMS1_ENABLE_PRINTF */

/* Imported variables ---------------------------------------------------------*/
#ifdef ALLMEMS1_ENABLE_PRINTF
   extern USBD_DescriptorsTypeDef VCP_Desc;
#endif /* ALLMEMS1_ENABLE_PRINTF */

/* Exported variables ---------------------------------------------------------*/
TargetFeatures_t TargetBoardFeatures;

#ifdef ALLMEMS1_ENABLE_PRINTF
  USBD_HandleTypeDef  USBD_Device;
#endif /* ALLMEMS1_ENABLE_PRINTF */

volatile float RMS_Ch[AUDIO_CHANNELS];
float DBNOISE_Value_Old_Ch[AUDIO_CHANNELS];
uint16_t PCM_Buffer[((AUDIO_CHANNELS*AUDIO_SAMPLING_FREQUENCY)/1000)  * N_MS ];
uint32_t NumSample= ((AUDIO_CHANNELS*AUDIO_SAMPLING_FREQUENCY)/1000)  * N_MS;

/* Private variables ---------------------------------------------------------*/
BSP_AUDIO_Init_t MicParams;

/* Local defines -------------------------------------------------------------*/

/* Local function prototypes --------------------------------------------------*/
static void MX_GPIO_Init(void);
static void Init_MEM1_Sensors(void);
static void Enable_MEM1_Sensors(void);

static void Init_MEMS_Mics(uint32_t AudioFreq, uint32_t AudioVolume);

extern tBleStatus Term_Update(uint8_t *data,uint8_t length);

/**
  * @brief  Initialize all the Target platform's Features
  * @param  None
  * @retval None
  */
void InitTargetPlatform(void)
{
#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
  /* Configure the SDCard */
  DATALOG_SD_Init();
  HAL_Delay(200);
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */
  
  Sensor_IO_SPI_CS_Init_All();
  
#ifdef ALLMEMS1_ENABLE_PRINTF
  /* enable USB power on Pwrctrl CR2 register */
  HAL_PWREx_EnableVddUSB();

  /* Configure the CDC */
  /* Init Device Library */
  USBD_Init(&USBD_Device, &VCP_Desc, 0);
  /* Add Supported Class */
  USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);
  /* Add Interface callbacks for AUDIO and CDC Class */
  USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);
  /* Start Device Process */
  USBD_Start(&USBD_Device);
  /* 10 seconds ... for having time to open the Terminal
   * for looking the ALLMEMS1 Initialization phase */
  HAL_Delay(10000);
#endif /* ALLMEMS1_ENABLE_PRINTF */

  /* Initialize LED */
  BSP_LED_Init( LED1 );  

  ALLMEMS1_PRINTF("\r\nSTMicroelectronics %s:\r\n"
         "\tVersion %c.%c.%c\r\n"
         "\tSTM32476JG-SensorTile board"
         "\r\n\n",
         ALLMEMS1_PACKAGENAME,
         ALLMEMS1_VERSION_MAJOR,ALLMEMS1_VERSION_MINOR,ALLMEMS1_VERSION_PATCH);

  /* Reset all the Target's Features */
  memset(&TargetBoardFeatures, 0, sizeof(TargetFeatures_t));
  
  /* Discovery and Intialize all the MEMS Target's Features */
  Init_MEM1_Sensors();
  
  /* Inialize the Gas Gouge if the battery is present */
  if(BSP_GG_Init(&TargetBoardFeatures.HandleGGComponent) == COMPONENT_OK){
    ALLMEMS1_PRINTF("\tOK Gas Gouge Component\n\r");
  } else {
    ALLMEMS1_PRINTF("\tBattery not present\n\r");
  }
  
  Enable_MEM1_Sensors();
  
  ALLMEMS1_PRINTF("\r\n");
}

/** @brief Initialize all the MEMS1 sensors
 * @param None
 * @retval None
 */
static void Init_MEM1_Sensors(void)
{
  ALLMEMS1_PRINTF("\nSensorTile board\n\r");
  
   /* Accelero & Gyro */
  if (BSP_MOTION_SENSOR_Init(ACCELERO_INSTANCE, MOTION_ACCELERO | MOTION_GYRO)==BSP_ERROR_NONE){
    ALLMEMS1_PRINTF("\tOK Accelero Sensor\n\r");
    ALLMEMS1_PRINTF("\tOK Gyroscope Sensor\n\r");
  } else {
    ALLMEMS1_PRINTF("\tError Accelero Sensor\n\r");
    ALLMEMS1_PRINTF("\tError Gyroscope Sensor\n\r");
    while(1);
  }
  
  /* Set Accelerometer Full Scale to 2G */
  Set2GAccelerometerFullScale();
  /* For accelero HW features */
  InitHWFeatures();

  /* Magneto */
  if(BSP_MOTION_SENSOR_Init(MAGNETO_INSTANCE, MOTION_MAGNETO)==BSP_ERROR_NONE){
    ALLMEMS1_PRINTF("\tOK Magneto Sensor\n\r");
  } else {
    ALLMEMS1_PRINTF("\tError Magneto Sensor\n\r");
    while(1);
  }

  /* Temperarure & Humidity */  
  if(BSP_ENV_SENSOR_Init(TEMPERATURE_INSTANCE_1,ENV_TEMPERATURE| ENV_HUMIDITY)==BSP_ERROR_NONE){
    ALLMEMS1_PRINTF("\tOK Temperature and Humidity (Sensor1)\n\r");
    TargetBoardFeatures.NumTempSensors++;
    TargetBoardFeatures.InitHumiditySensor= 1;
  } else {
    ALLMEMS1_PRINTF("\tError Temperature and Humidity (Sensor1)\n\r");
  }

  /* Temperarure & Pressure */ 
  if(BSP_ENV_SENSOR_Init(TEMPERATURE_INSTANCE_2,ENV_TEMPERATURE| ENV_PRESSURE)==BSP_ERROR_NONE) {
    ALLMEMS1_PRINTF("\tOK Temperature and Pressure (Sensor2)\n\r");
    TargetBoardFeatures.NumTempSensors++;
    TargetBoardFeatures.InitPressureSensor= 1;
  } else {
    ALLMEMS1_PRINTF("\tError Temperature and Pressure (Sensor2)\n\r");
  }
  
  /* Enable interruption LSM6DSM_INT2_PIN  */
  MX_GPIO_Init();
}

/** @brief Enable all the MEMS1 sensors
 *  @param None
 *  @retval None
 */
static void Enable_MEM1_Sensors(void)
{
  /* Accelero */
  if(BSP_MOTION_SENSOR_Enable(ACCELERO_INSTANCE, MOTION_ACCELERO)==BSP_ERROR_NONE)
    ALLMEMS1_PRINTF("\tEnabled Accelero Sensor\n\r");
  
  /* Gyro */
  if(BSP_MOTION_SENSOR_Enable(GYRO_INSTANCE, MOTION_GYRO)==BSP_ERROR_NONE)
    ALLMEMS1_PRINTF("\tEnabled Gyroscope Sensor\n\r");
  
  /* Magneto */
  if(BSP_MOTION_SENSOR_Enable(MAGNETO_INSTANCE, MOTION_MAGNETO)==BSP_ERROR_NONE)
    ALLMEMS1_PRINTF("\tEnabled Magneto Sensor\n\r");
     
  if(TargetBoardFeatures.InitHumiditySensor)
  {
    /* Temperarure */ 
    if(BSP_ENV_SENSOR_Enable(TEMPERATURE_INSTANCE_1, ENV_TEMPERATURE)==BSP_ERROR_NONE)
      ALLMEMS1_PRINTF("\tEnabled Temperature\t(Sensor1)\n\r");
    
    /* Humidity */ 
    if(BSP_ENV_SENSOR_Enable(HUMIDITY_INSTANCE, ENV_HUMIDITY)==BSP_ERROR_NONE)
      ALLMEMS1_PRINTF("\tEnabled Humidity\t(Sensor1)\n\r");
  }
  
  /* Temperarure & Pressure */ 
  if(TargetBoardFeatures.InitPressureSensor) {
    if(BSP_ENV_SENSOR_Enable(TEMPERATURE_INSTANCE_2, ENV_TEMPERATURE)==BSP_ERROR_NONE)
      ALLMEMS1_PRINTF("\tEnabled Temperature\t(Sensor2)\n\r");
    if(BSP_ENV_SENSOR_Enable(PRESSURE_INSTANCE, ENV_PRESSURE)==BSP_ERROR_NONE)
      ALLMEMS1_PRINTF("\tEnabled Pressure\t(Sensor2)\n\r");
  }
}

/** @brief Initialize GPIO
 *  @param None
 *  @retval None
 */
static void MX_GPIO_Init(void)
{
  /* Enable interruption LSM6DSM_INT2_PIN  */
  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = BSP_LSM6DSM_INT2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(BSP_LSM6DSM_INT2_EXTI_IRQn, 0x08, 0x00);
  HAL_NVIC_EnableIRQ(BSP_LSM6DSM_INT2_EXTI_IRQn);
}

/** @brief Initialize all the MEMS's Microphones
 * @param None
 * @retval None
 */
static void Init_MEMS_Mics(uint32_t AudioFreq, uint32_t AudioVolume)
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
  BSP_AUDIO_IN_Record(BSP_AUDIO_IN_INSTANCE, (uint8_t *) PCM_Buffer, AudioInSamples*2);
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
}


/**
  * @brief  This function switches on the LED
  * @param  None
  * @retval None
  */
void LedOnTargetPlatform(void)
{
  BSP_LED_On( LED1 );  
  TargetBoardFeatures.LedStatus=1;
}

/**
  * @brief  This function switches off the LED
  * @param  None
  * @retval None
  */
void LedOffTargetPlatform(void)
{
  BSP_LED_Off( LED1 );  
  TargetBoardFeatures.LedStatus=0;
}

/** @brief  This function toggles the LED
  * @param  None
  * @retval None
  */
void LedToggleTargetPlatform(void)
{
  BSP_LED_Toggle( LED1 );
}

/**
 * @brief User function for Erasing the MDM on Flash
 * @param None
 * @retval uint32_t Success/NotSuccess [1/0]
 */
uint32_t UserFunctionForErasingFlash(void) {
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;
  uint32_t Success=1;

  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks       = GetBank(MDM_FLASH_ADD);
  EraseInitStruct.Page        = GetPage(MDM_FLASH_ADD);
  EraseInitStruct.NbPages     = 2;

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
  uint64_t *WriteIndex;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();
  for(WriteIndex =((uint64_t *) InitMetaDataVector); WriteIndex<((uint64_t *) EndMetaDataVector); WriteIndex++) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address,*WriteIndex) == HAL_OK){
      Address = Address + 8;
    } else {
      /* Error occurred while writing data in Flash memory.
         User can add here some code to deal with this error
         FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
      Error_Handler();
      Success =0;
    }
  }

  /* Lock the Flash to disable the flash control register access (recommended
   to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
 
  return Success;
}

/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;
  
  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }
  
  return page;
}

/**
  * @brief  Gets the bank of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The bank of a given address
  */
uint32_t GetBank(uint32_t Addr)
{
  uint32_t bank = 0;
  
  if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0)
  {
  	/* No Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_1;
    }
    else
    {
      bank = FLASH_BANK_2;
    }
  }
  else
  {
  	/* Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_2;
    }
    else
    {
      bank = FLASH_BANK_1;
    }
  }
  
  return bank;
}
