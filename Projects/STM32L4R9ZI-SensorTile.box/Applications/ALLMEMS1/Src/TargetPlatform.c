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
#include <stdio.h>
#include "TargetFeatures.h"
#include "HWAdvanceFeatures.h"
#include "main.h"

/* Imported variables ---------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/
TargetFeatures_t TargetBoardFeatures;

#ifdef ALLMEMS1_ENABLE_PRINTF
  USBD_HandleTypeDef USBD_Device;
  volatile uint8_t VCOM_RxData;
  volatile uint8_t *VCOM_RxBuffer = NULL; /* Pointer to data buffer (received from USB). */
  volatile uint32_t VCOM_RxLength = 0;    /* Data length (received from USB). */
  uint8_t VComBufferToWrite[256];
  int32_t VComBytesToWrite;
#endif /* ALLMEMS1_ENABLE_PRINTF */

volatile float RMS_Ch[AUDIO_IN_CHANNELS];
float DBNOISE_Value_Old_Ch[AUDIO_IN_CHANNELS];
uint16_t PCM_Buffer[((AUDIO_IN_CHANNELS*AUDIO_SAMPLING_FREQUENCY)/1000)  * N_MS ];
uint32_t NumSample= ((AUDIO_IN_CHANNELS*AUDIO_SAMPLING_FREQUENCY)/1000)  * N_MS;

/* Private variables ---------------------------------------------------------*/
BSP_AUDIO_Init_t MicParams;

/* Local defines -------------------------------------------------------------*/

/* Local function prototypes --------------------------------------------------*/
static void MX_GPIO_Init(void);
static void Init_MEM1_Sensors(void);

static void Init_MEMS_Mics(uint32_t AudioFreq, uint32_t AudioVolume);

/**
  * @brief  Initialize all the Target platform's Features
  * @param  None
  * @retval None
  */
void InitTargetPlatform(void)
{
  /* Init Led1/Led2 */
  LedInitTargetPlatform();
  
  /* Initialize User Button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* Initialize the Power Button */
  //BSP_PowerButton_Init();

  /* Initialize the Battery Charger */
  BSP_BC_Init();

  /* In order to be able to Read Battery Volt */
  BSP_BC_BatMS_Init();

  /* In order to Initialize the GPIO for having the battery Status */
  BSP_BC_ChrgPin_Init();
  
#ifdef ALLMEMS1_ENABLE_PRINTF
  BSP_LED_On(LED1);
  BSP_LED_On(LED2);

  /* Enable USB power on Pwrctrl CR2 register */
  HAL_PWREx_EnableVddUSB();

  /*** USB CDC Configuration ***/
  /* Init Device Library */
  USBD_Init(&USBD_Device, &VCP_Desc, 0);

  /* Add Supported Class */
  USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);

  /* Add Interface callbacks for CDC Class */
  USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);

  /* Start Device Process */
  USBD_Start(&USBD_Device);

  /* Wait 5 seconds for looking the Initialization phases */
  HAL_Delay(5000);

  BSP_LED_Off(LED1);
  BSP_LED_Off(LED2);
#endif /* ALLMEMS1_ENABLE_PRINTF */

  ALLMEMS1_PRINTF("\r\nSTMicroelectronics %s:\r\n"
         "\tVersion %c.%c.%c\r\n"
         "\tSTM32L4R9ZI-SensorTile.box board\r\n\n",
          ALLMEMS1_PACKAGENAME,
          ALLMEMS1_VERSION_MAJOR,ALLMEMS1_VERSION_MINOR,ALLMEMS1_VERSION_PATCH);

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
  ALLMEMS1_PRINTF("\nSensorTile.box board\n\r");

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
  } else {
    ALLMEMS1_PRINTF("Error Temperature and Humidity (Sensor1)\n\r");
  }

  /* Temperarure & Pressure */ 
  if(BSP_ENV_SENSOR_Init(TEMPERATURE_INSTANCE_2,ENV_TEMPERATURE| ENV_PRESSURE)==BSP_ERROR_NONE) {
    ALLMEMS1_PRINTF("\tOK Temperature and Pressure (Sensor2)\n\r");
    TargetBoardFeatures.NumTempSensors++;
  } else {
    ALLMEMS1_PRINTF("\tError Temperature and Pressure (Sensor2)\n\r");
  }
  
  /* Enable interruption LSM6DSM_INT2_PIN  */
  MX_GPIO_Init();

  /*  Enable all the sensors */
  if(BSP_MOTION_SENSOR_Enable(ACCELERO_INSTANCE, MOTION_ACCELERO)==BSP_ERROR_NONE)
    ALLMEMS1_PRINTF("\tEnabled Accelero Sensor\n\r");
  if(BSP_MOTION_SENSOR_Enable(GYRO_INSTANCE, MOTION_GYRO)==BSP_ERROR_NONE)
    ALLMEMS1_PRINTF("\tEnabled Gyroscope Sensor\n\r");
  if(BSP_MOTION_SENSOR_Enable(MAGNETO_INSTANCE, MOTION_MAGNETO)==BSP_ERROR_NONE)
    ALLMEMS1_PRINTF("\tEnabled Magneto Sensor\n\r");
     
  if(BSP_ENV_SENSOR_Enable(TEMPERATURE_INSTANCE_1, ENV_TEMPERATURE)==BSP_ERROR_NONE)
    ALLMEMS1_PRINTF("\tEnabled Temperature\t(Sensor1)\n\r");
  if(BSP_ENV_SENSOR_Enable(HUMIDITY_INSTANCE, ENV_HUMIDITY)==BSP_ERROR_NONE)
    ALLMEMS1_PRINTF("\tEnabled Humidity\t(Sensor1)\n\r");

  if(TargetBoardFeatures.NumTempSensors==2) {
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
  /* Enable interruption LSM6DSOX  */
  GPIO_InitTypeDef GPIO_InitStruct;

  //__HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  /*Configure GPIO pins : PE3 */ //PA2
  //GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  //HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
//  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
//  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  
  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
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
  MicParams.Device = ONBOARD_MIC;
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

/** @brief  This function initiliazes the LED
  * @param  None
  * @retval None
  */
void LedInitTargetPlatform(void)
{
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
}

#ifdef ALLMEMS1_ENABLE_PRINTF
/**
 * @brief  Read from VCOM
 * @param  Pointer to buffer.
 * @param  Data max. length.
 * @retval Number of really read data bytes.
 */
uint32_t VCOM_read(char *buffer, uint32_t len_max)
{
  /* VCOM data receive not completed or no VCOM data received at all. */
  if (VCOM_RxData == 0) {
    return 0;
  }

  /* ERROR: No VCOM data ready. */
  if (VCOM_RxLength == 0 || VCOM_RxBuffer == NULL) {
    Error_Handler();
  }

  /* Read all data */
  if (VCOM_RxLength <= len_max) {
    uint32_t len = VCOM_RxLength;
    memcpy((uint8_t*)buffer, (uint8_t*)VCOM_RxBuffer, len);

    VCOM_RxData   = 0;
    VCOM_RxBuffer = NULL;
    VCOM_RxLength = 0;

    CDC_Next_Packet_Rx();
    return len;
  } else {
    /* Read part of data that fits into buffer. */
    memcpy((uint8_t*)buffer, (uint8_t*)VCOM_RxBuffer, len_max);

    VCOM_RxBuffer += len_max;
    VCOM_RxLength -= len_max;

    return len_max;
  }
}
#endif /* ALLMEMS1_ENABLE_PRINTF */

