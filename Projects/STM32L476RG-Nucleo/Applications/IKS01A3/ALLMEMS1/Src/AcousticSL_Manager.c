/**
  ******************************************************************************
  * @file    AcousticSL_Manager.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   This file includes Source location interface functions
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
#include "TargetFeatures.h"

/* Code for AcousticSL integration - Start Section */

/* Volatile variables and its initialization --------------------------------------*/
volatile int32_t SourceLocationToSend=  ACOUSTIC_SL_NO_AUDIO_DETECTED;

/* Exported variables -------------------------------------------------------------*/
/*Handler and Config structure for Source Localization*/
AcousticSL_Handler_t libSoundSourceLoc_Handler_Instance;
AcousticSL_Config_t  libSoundSourceLoc_Config_Instance;

/* Imported Variables -------------------------------------------------------------*/
extern uint16_t PCM_Buffer[];

/* Private function prototypes -----------------------------------------------*/
static void SW_Task2_Start(void);

/**
* @brief  Initialises AcousticSL algorithm
* @param  None
* @retval None
*/
void AcousticSL_Manager_init(void)
{
  char LibVersion[36];
  
  AcousticSL_GetLibVersion(LibVersion);
  
  /*Setup Source Localization static parameters*/
  libSoundSourceLoc_Handler_Instance.channel_number=            AUDIO_IN_CHANNELS;
  libSoundSourceLoc_Handler_Instance.M12_distance=              SIDE;
  libSoundSourceLoc_Handler_Instance.sampling_frequency=        AUDIO_IN_SAMPLING_FREQUENCY;  
  libSoundSourceLoc_Handler_Instance.algorithm=                 ACOUSTIC_SL_ALGORITHM_GCCP;
  libSoundSourceLoc_Handler_Instance.ptr_M1_channels=           AUDIO_IN_CHANNELS;
  libSoundSourceLoc_Handler_Instance.ptr_M2_channels=           AUDIO_IN_CHANNELS;
  libSoundSourceLoc_Handler_Instance.ptr_M3_channels=           AUDIO_IN_CHANNELS; 
  libSoundSourceLoc_Handler_Instance.ptr_M4_channels=           AUDIO_IN_CHANNELS;
  libSoundSourceLoc_Handler_Instance.samples_to_process=        512;

  AcousticSL_getMemorySize( &libSoundSourceLoc_Handler_Instance);  
  libSoundSourceLoc_Handler_Instance.pInternalMemory=(uint32_t *)malloc(libSoundSourceLoc_Handler_Instance.internal_memory_size);
  
  if(libSoundSourceLoc_Handler_Instance.pInternalMemory == NULL)
  {
    ALLMEMS1_PRINTF("Error AcousticSL Memory allocation\n\r");
    return;
  }
  
  if(AcousticSL_Init( &libSoundSourceLoc_Handler_Instance))
  {
    ALLMEMS1_PRINTF("Error AcousticSL Initialization\n\r");
    return;
  }

  /*Setup Source Localization dynamic parameters*/  
  libSoundSourceLoc_Config_Instance.resolution= ANGLE_RESOLUTION;
  libSoundSourceLoc_Config_Instance.threshold=  SENSITIVITY_SL_HI_THRESHOLD;

  if(AcousticSL_setConfig(&libSoundSourceLoc_Handler_Instance, &libSoundSourceLoc_Config_Instance))
  {
    ALLMEMS1_PRINTF("Error AcousticSL Configuration\n\r");
    return;
  }
     
  /* If everything is ok */
  TargetBoardFeatures.AcousticSLIsInitalized=1;
  ALLMEMS1_PRINTF("Initialized %s (%ld bytes allocated)\r\n", LibVersion, libSoundSourceLoc_Handler_Instance.internal_memory_size);
 
  HAL_NVIC_SetPriority((IRQn_Type)EXTI2_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ((IRQn_Type)EXTI2_IRQn);

  return;
}

/**
* @brief  User function that is called when 1 ms of PDM data is available.
*         In this application only PDM to PCM conversion and USB streaming
*         is performed.
*         User can add his own code here to perform some DSP or audio analysis.
* @param  none
* @retval None
*/
void AudioProcess_SL(void)
{
  if(AcousticSL_Data_Input((int16_t *)&PCM_Buffer[M1_EXT_B],
                           (int16_t *)&PCM_Buffer[M4_EXT_B],
                           NULL,
                           NULL,
                           &libSoundSourceLoc_Handler_Instance))
  {
    /*Localization Processing Task*/
    SW_Task2_Start(); 
  }
}

void SetConfig_SL(uint16_t newThresh)
{
  libSoundSourceLoc_Config_Instance.threshold = newThresh;
  AcousticSL_setConfig(&libSoundSourceLoc_Handler_Instance,&libSoundSourceLoc_Config_Instance);
}

/**
* @brief Throws Highest priority interrupt
* @param  None
* @retval None
*/
void SW_Task2_Start(void)
{
  HAL_NVIC_SetPendingIRQ(EXTI2_IRQn);
}

/**
* @brief Lower priority interrupt handler routine
* @param  None
* @retval None
*/
void  SW_Task2_Callback(void) 
{
  int32_t result=0;
  
  if(AcousticSL_Process((int32_t *)&result, &libSoundSourceLoc_Handler_Instance))
  {
    ALLMEMS1_PRINTF("AcousticSL_Process error\r\n");
    return;
  }
  
  if(result != ACOUSTIC_SL_NO_AUDIO_DETECTED)
  {
    SourceLocationToSend= (int16_t)result;  
    SourceLocationToSend= SourceLocationToSend + 90;
    
    //ALLMEMS1_PRINTF("\r\nSourceLocationToSend = %d\r\n",SourceLocationToSend);
  }
}

/* Code for AcousticSL integration - End Section */

