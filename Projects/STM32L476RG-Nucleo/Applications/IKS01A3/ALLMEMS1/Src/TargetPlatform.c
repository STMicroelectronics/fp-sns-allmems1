/**
  ******************************************************************************
  * @file    TargetPlatform.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.3.0
  * @date    30-June-2023
  * @brief   Initialization of the Target Platform
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
#include "TargetFeatures.h"
#include "main.h"

/* Imported variables ---------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/
TargetFeatures_t TargetBoardFeatures;

volatile float RMS_Ch[AUDIO_IN_CHANNELS];
float DBNOISE_Value_Old_Ch[AUDIO_IN_CHANNELS];
uint16_t PCM_Buffer[((AUDIO_IN_CHANNELS*AUDIO_IN_SAMPLING_FREQUENCY)/1000)  * N_MS ];
uint32_t NumSample= ((AUDIO_IN_CHANNELS*AUDIO_IN_SAMPLING_FREQUENCY)/1000)  * N_MS;

/* Private variables ---------------------------------------------------------*/
CCA02M2_AUDIO_Init_t MicParams;

/* Local defines -------------------------------------------------------------*/

/* Local function prototypes --------------------------------------------------*/
static void Init_MEM1_Sensors(void);
static void Int_GPIO_Init(void);

static void Init_MEMS_Mics(uint32_t AudioFreq, uint32_t AudioVolume);

//extern tBleStatus Term_Update(uint8_t *data,uint8_t length);

/**
  * @brief  Initialize all the Target platform's Features
  * @param  None
  * @retval None
  */
void InitTargetPlatform(void)
{
#ifdef ALLMEMS1_ENABLE_PRINTF
  /* UART Initialization */
  if(BSP_COM_Init(COM1) != BSP_ERROR_NONE) {
    Error_Handler();
  } else {
    HAL_Delay(20);
    ALLMEMS1_PRINTF("\033[2J"); /* serial console clear screen */
    ALLMEMS1_PRINTF("\033[H"); /* serial console cursor to home */
    ALLMEMS1_PRINTF("UART Initialized\r\n");
  }
#endif /* ALLMEMS1_ENABLE_PRINTF */
  
  /* Initialize button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* Initialize LED */
  BSP_LED_Init(LED2);

  ALLMEMS1_PRINTF("\r\nSTMicroelectronics %s:\r\n"
                  "\tVersion %c.%c.%c\r\n"
                  "\tSTM32L476RG-Nucleo board\r\n"
                  "\t\tMCU clock set at %ld MHz"
                  "\r\n\n",
                  ALLMEMS1_PACKAGENAME,
                  ALLMEMS1_VERSION_MAJOR,ALLMEMS1_VERSION_MINOR,ALLMEMS1_VERSION_PATCH, SystemCoreClock/1000000);

  /* Reset all the Target's Features */
  memset(&TargetBoardFeatures, 0, sizeof(TargetFeatures_t));
  /* Discovery and Intialize all the MEMS Target's Features */
  Init_MEM1_Sensors();
 
  ALLMEMS1_PRINTF("\n\r");
}

/** @brief Initialize all the MEMS1 sensors
 * @param None
 * @retval None
 */
static void Init_MEM1_Sensors(void)
{
  ALLMEMS1_PRINTF("\nCode compiled for X-NUCLEO-IKS01A3 board\n\r");
  TargetBoardFeatures.mems_expansion_board= _IKS01A3;
      
   /* Accelero & Gyro */
  if (MOTION_SENSOR_Init(ACCELERO_INSTANCE, MOTION_ACCELERO | MOTION_GYRO)==BSP_ERROR_NONE){
    ALLMEMS1_PRINTF("\tOK Accelero Sensor\n\r");
    ALLMEMS1_PRINTF("\tOK Gyroscope Sensor\n\r");
  } else {
    ALLMEMS1_PRINTF("\tError Accelero Sensor\n\r");
    ALLMEMS1_PRINTF("\tError Gyroscope Sensor\n\r");
    //while(1);
  }

  if(MOTION_SENSOR_Init(MAGNETO_INSTANCE, MOTION_MAGNETO)==BSP_ERROR_NONE){
    ALLMEMS1_PRINTF("\tOK Magneto Sensor\n\r");
  } else {
    ALLMEMS1_PRINTF("\tError Magneto Sensor\n\r");
    //while(1);
  }

  /* Temperarure & Humidity */  
  if(ENV_SENSOR_Init(TEMPERATURE_INSTANCE_1,ENV_TEMPERATURE| ENV_HUMIDITY)==BSP_ERROR_NONE){
    ALLMEMS1_PRINTF("\tOK Temperature and Humidity (Sensor1)\n\r");
    TargetBoardFeatures.NumTempSensors++;
  } else {
    ALLMEMS1_PRINTF("\tError Temperature and Humidity (Sensor1)\n\r");
  }

  /* Temperarure & Pressure */ 
  if(ENV_SENSOR_Init(TEMPERATURE_INSTANCE_2,ENV_TEMPERATURE| ENV_PRESSURE)==BSP_ERROR_NONE) {
    ALLMEMS1_PRINTF("\tOK Temperature and Pressure (Sensor2)\n\r");
    TargetBoardFeatures.NumTempSensors++;
  } else {
    ALLMEMS1_PRINTF("\tError Sensor2 Sensor\n\r");
  }

  /*  Enable all the sensors */
  if(MOTION_SENSOR_Enable(ACCELERO_INSTANCE, MOTION_ACCELERO)==BSP_ERROR_NONE)
    ALLMEMS1_PRINTF("\tEnabled Accelero Sensor\n\r");
  if(MOTION_SENSOR_Enable(GYRO_INSTANCE, MOTION_GYRO)==BSP_ERROR_NONE)
    ALLMEMS1_PRINTF("\tEnabled Gyroscope Sensor\n\r");
  if(MOTION_SENSOR_Enable(MAGNETO_INSTANCE, MOTION_MAGNETO)==BSP_ERROR_NONE)
    ALLMEMS1_PRINTF("\tEnabled Magneto Sensor\n\r");
     
  if(ENV_SENSOR_Enable(TEMPERATURE_INSTANCE_1, ENV_TEMPERATURE)==BSP_ERROR_NONE)
    ALLMEMS1_PRINTF("\tEnabled Temperature\t(Sensor1)\n\r");
  if(ENV_SENSOR_Enable(HUMIDITY_INSTANCE, ENV_HUMIDITY)==BSP_ERROR_NONE)
    ALLMEMS1_PRINTF("\tEnabled Humidity\t(Sensor1)\n\r");

  if(TargetBoardFeatures.NumTempSensors==2) {
    if(ENV_SENSOR_Enable(TEMPERATURE_INSTANCE_2, ENV_TEMPERATURE)==BSP_ERROR_NONE)
      ALLMEMS1_PRINTF("\tEnabled Temperature\t(Sensor2)\n\r");
    if(ENV_SENSOR_Enable(PRESSURE_INSTANCE, ENV_PRESSURE)==BSP_ERROR_NONE)
      ALLMEMS1_PRINTF("\tEnabled Pressure\t(Sensor2)\n\r");
  }
  
  Int_GPIO_Init();
}

static void Int_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/** @brief Initialize all the MEMS's Microphones
 * @param None
 * @retval None
 */
static void Init_MEMS_Mics(uint32_t AudioFreq, uint32_t AudioVolume)
{
  /* Initialize microphone acquisition */  
  MicParams.BitsPerSample = 16;
  MicParams.ChannelsNbr = AUDIO_IN_CHANNELS;
  MicParams.Device = AUDIO_IN_DIGITAL_MIC;
  MicParams.SampleRate = AudioFreq;
  MicParams.Volume = AudioVolume;
  
  if( CCA02M2_AUDIO_IN_Init(CCA02M2_AUDIO_INSTANCE, &MicParams) != BSP_ERROR_NONE )
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
  if( CCA02M2_AUDIO_IN_SetVolume(CCA02M2_AUDIO_INSTANCE, AudioVolume) != BSP_ERROR_NONE )
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
  CCA02M2_AUDIO_IN_Record(CCA02M2_AUDIO_INSTANCE, (uint8_t *) PCM_Buffer, AudioInSamples*2);
}

/** @brief DeInitialize all the MEMS's Microphones
 * @param None
 * @retval None
 */
void DeInitMics(void)
{
  if( CCA02M2_AUDIO_IN_Stop(CCA02M2_AUDIO_INSTANCE) != BSP_ERROR_NONE )
  {
    ALLMEMS1_PRINTF("Error Audio Stop\r\n");
    
    while(1) {
      ;
    }
  }
  else
    ALLMEMS1_PRINTF("OK Audio Stop\r\n");

  
  if( CCA02M2_AUDIO_IN_DeInit(CCA02M2_AUDIO_INSTANCE) != BSP_ERROR_NONE )
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
  BSP_LED_On(LED2);
  TargetBoardFeatures.LedStatus=1;
}

/**
  * @brief  This function switches off the LED
  * @param  None
  * @retval None
  */
void LedOffTargetPlatform(void)
{
  BSP_LED_Off(LED2);
  TargetBoardFeatures.LedStatus=0;
}

/** @brief  This function toggles the LED
  * @param  None
  * @retval None
  */
void LedToggleTargetPlatform(void)
{
  BSP_LED_Toggle(LED2);
}

