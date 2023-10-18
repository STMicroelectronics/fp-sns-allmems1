/**
  ******************************************************************************
  * @file    AcousticBF_Manager.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.3.0
  * @date    30-June-2023
  * @brief   This file includes Source location interface functions
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

/* Includes ------------------------------------------------------------------*/
#include "TargetFeatures.h"
#include "sensor_service.h"
#include "AcousticBF_Manager.h"

/* Imported Variables -------------------------------------------------------------*/
extern uint16_t PDM_Buffer[];

/* Private variables -------------------------------------------------------------*/
/*Handler and Config structure for BeamForming*/
AcousticBF_Handler_t libBeamforming_Handler_Instance;
AcousticBF_Config_t lib_Beamforming_Config_Instance;

static uint8_t MicInputBF_1;
static uint8_t MicInputBF_2;

int16_t BFOUT[AUDIO_CHANNELS*AUDIO_SAMPLING_FREQUENCY/1000];

/* Private function prototypes ---------------------------------------------------*/
void SW_Task1_Start(void);

/**
* @brief  Initialize Acoustic Beam Forming library
* @param  None
* @retval None
*/
void AcousticBF_Manager_init(void)
{
  char LibVersion[36];
  
  AcousticBF_GetLibVersion(LibVersion);
  
  /*Setup Beamforming static parameters*/
  libBeamforming_Handler_Instance.algorithm_type_init= ACOUSTIC_BF_TYPE_STRONG;
  libBeamforming_Handler_Instance.ref_mic_enable= ACOUSTIC_BF_REF_ENABLE;
  libBeamforming_Handler_Instance.ptr_out_channels= OUTPUT_STREAM_NUMBER_CHANNELS;
  libBeamforming_Handler_Instance.data_format= ACOUSTIC_BF_DATA_FORMAT_PDM;
  libBeamforming_Handler_Instance.sampling_frequency= AUDIO_SAMPLING_FREQUENCY_BF;  
  libBeamforming_Handler_Instance.ptr_M1_channels= AUDIO_CHANNELS;
  libBeamforming_Handler_Instance.ptr_M2_channels= AUDIO_CHANNELS;
  libBeamforming_Handler_Instance.delay_enable = 1;
  
  AcousticBF_getMemorySize(&libBeamforming_Handler_Instance);
  libBeamforming_Handler_Instance.pInternalMemory =(uint32_t *)malloc(libBeamforming_Handler_Instance.internal_memory_size);
  
  if(libBeamforming_Handler_Instance.pInternalMemory == NULL)
  {
    ALLMEMS1_PRINTF("Error AcousticBF Memory allocation\n\r");
    return;
  }
  
  if(AcousticBF_Init(&libBeamforming_Handler_Instance))
  {
    ALLMEMS1_PRINTF("Error AcousticBF Initialization\n\r");
    return;
  }  
  
  /*Setup Beamforming dynamic parameters*/
  lib_Beamforming_Config_Instance.algorithm_type= ACOUSTIC_BF_TYPE_STRONG;
  lib_Beamforming_Config_Instance.mic_distance= DIAGONAL;
  lib_Beamforming_Config_Instance.M2_gain= GAIN_APPLIED_TO_SECOND_MICROPHONE;  
  lib_Beamforming_Config_Instance.volume = AUDIO_BF_VOLUME_VALUE;

  if(AcousticBF_setConfig(&libBeamforming_Handler_Instance, &lib_Beamforming_Config_Instance))
  {
    ALLMEMS1_PRINTF("Error AcousticBF Configuration\n\r");
    return;
  }

/* Set direction default value (case 3) */ 
  MicInputBF_1= BOTTOM_RIGHT_MIC;
  MicInputBF_2= BOTTOM_LEFT_MIC;
    
  /* If everything is ok */
  TargetBoardFeatures.AcousticBFIsInitalized=1;
  ALLMEMS1_PRINTF("Initialized %s (%ld bytes allocated)\r\n", LibVersion, libBeamforming_Handler_Instance.internal_memory_size);
  
  HAL_NVIC_SetPriority((IRQn_Type)EXTI1_IRQn, 0x09, 0);  
  HAL_NVIC_EnableIRQ((IRQn_Type)EXTI1_IRQn); 	
}

/**
* @brief  Support function needed to setup the correct beamforming microphones depending
*         on the direction choosen by the user.
* @param  Direction: parameter that describes the desired output:
*         For nucleo Board: 1 to 2 for one of the possible beamforming directions* 
*         For BlueCoin:     1 to 8 for one of the possible beamforming directions*                         
* @retval None
*/
void BeamFormingDirectionSetup(int32_t Direction)
{
 switch(Direction)
 {
 case 1:
   MicInputBF_1= TOP_RIGHT_MIC;
   MicInputBF_2= BOTTOM_RIGHT_MIC;
   lib_Beamforming_Config_Instance.mic_distance = SIDE;
   break;
 case 2:
   MicInputBF_1= TOP_RIGHT_MIC;
   MicInputBF_2= BOTTOM_LEFT_MIC;
   lib_Beamforming_Config_Instance.mic_distance = DIAGONAL;
   break;
 case 3:
   MicInputBF_1= BOTTOM_RIGHT_MIC;
   MicInputBF_2= BOTTOM_LEFT_MIC;
   lib_Beamforming_Config_Instance.mic_distance = SIDE;
   break;
 case 4:
   MicInputBF_1= BOTTOM_RIGHT_MIC;
   MicInputBF_2= TOP_LEFT_MIC;
   lib_Beamforming_Config_Instance.mic_distance = DIAGONAL;
   break;
 case 5:
   MicInputBF_1= BOTTOM_RIGHT_MIC;
   MicInputBF_2= TOP_RIGHT_MIC;
   lib_Beamforming_Config_Instance.mic_distance = SIDE;
   break;
 case 6:
   MicInputBF_1= BOTTOM_LEFT_MIC;
   MicInputBF_2= TOP_RIGHT_MIC;
   lib_Beamforming_Config_Instance.mic_distance = DIAGONAL;
   break;
 case 7:
   MicInputBF_1= BOTTOM_LEFT_MIC;
   MicInputBF_2= BOTTOM_RIGHT_MIC;
   lib_Beamforming_Config_Instance.mic_distance = SIDE;
   break;
 case 8:
   MicInputBF_1= TOP_LEFT_MIC;
   MicInputBF_2= BOTTOM_RIGHT_MIC;
   lib_Beamforming_Config_Instance.mic_distance = DIAGONAL;
   break;
 }
  
  AcousticBF_setConfig(&libBeamforming_Handler_Instance, &lib_Beamforming_Config_Instance);
}

void BeamFormingSetType(uint32_t type)
{  
//    BV_ADPCM_Profile_Status status;
//
//  status =BluevoiceADPCM_GetStatus();
  //while(BluevoiceADPCM_GetStatus() != BV_ADPCM_STATUS_READY);

  if(type == 0)
  {
    lib_Beamforming_Config_Instance.algorithm_type= ACOUSTIC_BF_TYPE_ASR_READY;
    if(AcousticBF_setConfig(&libBeamforming_Handler_Instance, &lib_Beamforming_Config_Instance))
    {
      ALLMEMS1_PRINTF("Error AcousticBF Configuration\n\r");
      return;
    } 
    
  }
  else if(type == 1)
  {
    lib_Beamforming_Config_Instance.algorithm_type= ACOUSTIC_BF_TYPE_STRONG;
    if(AcousticBF_setConfig(&libBeamforming_Handler_Instance, &lib_Beamforming_Config_Instance))
    {
      ALLMEMS1_PRINTF("Error AcousticBF Configuration\n\r");
      return;
    } 
    
  }
}

/**
* @brief  User function that is called when 1 ms of PDM data is available.
* @param  none
* @retval None
*/
void AudioProcess_BF(int32_t dir)
{
  if(AcousticBF_FirstStep(&((uint8_t *)(PDM_Buffer))[MicInputBF_1],
                          &((uint8_t *)(PDM_Buffer))[MicInputBF_2], 
                          (int16_t *)BFOUT,
                          &libBeamforming_Handler_Instance))
  {
    SW_Task1_Start();          
  }
}

/**
* @brief Throws Highest priority interrupt
* @param  None
* @retval None
*/
void SW_Task1_Start(void)
{  
  HAL_NVIC_SetPendingIRQ(EXTI1_IRQn);  
}

/**
* @brief  Highest priority interrupt handler routine
* @param  None
* @retval None
*/
void SW_Task1_Callback(void)
{        
  AcousticBF_SecondStep(&libBeamforming_Handler_Instance); 
}

