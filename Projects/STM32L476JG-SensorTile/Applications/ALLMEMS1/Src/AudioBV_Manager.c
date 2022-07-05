/**
  ******************************************************************************
  * @file    AudioBV_Manager.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   This file includes BlueVoice interface functions
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
#include <stdio.h>
#include "TargetFeatures.h"

/* Code for BlueVoice integration - Start Section */

/* Imported Variables -------------------------------------------------------------*/
extern uint16_t PCM_Buffer[];
extern BV_ADPCM_ProfileHandle_t BLUEVOICE_tx_handle;

#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
extern uint8_t IsSdMemsRecording;
extern uint8_t IsSdAudioRecording;
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */

/* Private Variables -------------------------------------------------------------*/
static uint32_t led_toggle_count = 0; /*!< Variable used to handle led toggling.*/
BV_ADPCM_Config_t BLUEVOICE_Config;
BV_ADPCM_Status bvStat;

extern volatile uint16_t BFOUT[];
volatile int32_t BF_dir;

/* Code for BlueVoice integration - Start Section */
static uint16_t num_byte_sent = 0;
/* Code for BlueVoice integration - End Section */

//#if defined(STM32F401xE) || defined(STM32_BLUECOIN)  
void SW_Task3_Start(void);
//#endif

/* Private Defines -------------------------------------------------------------*/
#define LED_TOGGLE_STREAMING  100


/**
* @brief  Initialises BlueVoice manager
* @param  None
* @retval None
*/
void AudioBV_Manager_init(void)
{
  BluevoiceADPCM_Initialize();
  
  BLUEVOICE_Config.sampling_frequency = FR_8000;
  BLUEVOICE_Config.channel_in = 1;
  BLUEVOICE_Config.channel_tot = 1;
  bvStat = BluevoiceADPCM_SetConfig(&BLUEVOICE_Config);
  if (bvStat != BV_ADPCM_SUCCESS) {
    goto fail;
  }
  HAL_NVIC_SetPriority((IRQn_Type)EXTI3_IRQn, 0x06, 0); 
  HAL_NVIC_EnableIRQ((IRQn_Type)EXTI3_IRQn); 
  
  bvStat = BluevoiceADPCM_SetTxHandle(&BLUEVOICE_tx_handle);
  if (bvStat != BV_ADPCM_SUCCESS) {
    goto fail;
  }
  
  /* If everything is ok */
  TargetBoardFeatures.AudioBVIsInitalized=1;
  ALLMEMS1_PRINTF("Initialized ST BlueVoiceADPCM v2.0.0\r\n\r\n");

  return;
  
  fail:
    while(1){}
}

/**
* @brief  User function that is called when the PCM_Buffer is full and ready to send.
* @param  none
* @retval None
*/
void AudioProcess_BV(void)
{
  BV_ADPCM_Status status;
 
  if (BluevoiceADPCM_IsProfileConfigured())
  {
    status = BluevoiceADPCM_AudioIn( (uint16_t*)PCM_Buffer, BV_PCM_AUDIO_IN_SAMPLES); 
    
    if (led_toggle_count++ >= LED_TOGGLE_STREAMING) 
    {
      led_toggle_count = 0;
              
      #ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
      if( (!IsSdMemsRecording) && (!IsSdAudioRecording))
      {
      #endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */
        LedToggleTargetPlatform();
      #ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
      }
      #endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */
    }
    
    if(status==BV_ADPCM_OUT_BUF_READY) {
      SW_Task3_Start();
    }
  }
  
}

#if (defined(STM32F401xE) || defined(STM32F446xx))
/**
* @brief 
* @param None
* @retval None
*/
void BV_SetInForBF(void)
{
  BLUEVOICE_Config.channel_tot = 4;

  while(BluevoiceADPCM_SetConfig(&BLUEVOICE_Config) != BV_ADPCM_SUCCESS);
}

/**
* @brief 
* @param None
* @retval None
*/
void BV_SetOutForBF(void)
{
  BLUEVOICE_Config.channel_tot = 2;
    
  while(BluevoiceADPCM_SetConfig(&BLUEVOICE_Config) != BV_ADPCM_SUCCESS);
}
#endif /* (defined(STM32F401xE) || defined(STM32F446xx)) */



/**
* @brief Throws Highest priority interrupt
* @param None
* @retval None
*/
void SW_Task3_Start(void)
{ 
  HAL_NVIC_SetPendingIRQ(EXTI3_IRQn); 
}

void SW_BV_send_Callback(void)
{
            
  BluevoiceADPCM_SendData(&num_byte_sent);
}

