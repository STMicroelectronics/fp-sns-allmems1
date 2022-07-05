/**
******************************************************************************
* @file    SensorTile_audio.c
* @author  SRA - Central Labs
* @version v2.1.6
* @date    10-Feb-2022
* @brief   This file provides the Audio driver for the SENSORTILE
*          board.
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
#include "SensorTile_audio.h"
#include "SensorTile_conf.h"
#include "audio.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup SENSORTILE
  * @{
  */ 
  
/** @defgroup SENSORTILE_AUDIO SENSORTILE AUDIO
  * @{
  */ 
/** @defgroup SENSORTILE_AUDIO_Private_Defines SENSORTILE_AUDIO Private Defines
  * @{
  */
/**
  * @}
  */

/** @defgroup SENSORTILE_AUDIO_Private_Macros SENSORTILE_AUDIO Private Macros
  * @{
  */

#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))

#define SAI_CLOCK_DIVIDER(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (12U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (2U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (6U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (1U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (3U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (0U) \
         : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (2U) : (1U)

#define DFSDM_OVER_SAMPLING(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (128U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (256U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (128U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (128U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (64U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (64U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (64U)  \
	  : ((__FREQUENCY__) == (AUDIO_FREQUENCY_96K)) ? (32U) : (32U) 
	  
#define DFSDM_CLOCK_DIVIDER(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (17U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (24U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (24U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (16U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_96K)) ? (16U) : (16U)

#define DFSDM_FILTER_ORDER(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (DFSDM_FILTER_SINC4_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (DFSDM_FILTER_SINC5_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (DFSDM_FILTER_SINC4_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (DFSDM_FILTER_SINC4_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (DFSDM_FILTER_SINC5_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (DFSDM_FILTER_SINC5_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (DFSDM_FILTER_SINC5_ORDER) \
	  : ((__FREQUENCY__) == (AUDIO_FREQUENCY_96K)) ? (DFSDM_FILTER_SINC5_ORDER) : (DFSDM_FILTER_SINC5_ORDER)

#define DFSDM_MIC_BIT_SHIFT(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (8U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (5U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (8U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (8U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (10U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (10U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (10U) \
	  : ((__FREQUENCY__) == (AUDIO_FREQUENCY_96K)) ? (5U) : (5U)

/**
  * @}
  */ 

/** @defgroup SENSORTILE_AUDIO_Exported_Variables SENSORTILE_AUDIO Exported Variables
  * @{
  */
/* Recording context */
AUDIO_IN_Ctx_t                         AudioInCtx[AUDIO_IN_INSTANCES_NBR] = {0};
AUDIO_OUT_Ctx_t                        AudioOutCtx[AUDIO_OUT_INSTANCES_NBR] = {0};

/**
  * @}
  */
  
/** @defgroup SENSORTILE_AUDIO_Private_Variables SENSORTILE_AUDIO Private Variables
  * @{
  */
static AUDIO_Drv_t                     *AudioDrv = NULL;
static void                            *CompObj = NULL;

/* Play handle */
SAI_HandleTypeDef               hAudioOutSai;

/* Recording handles */
static DFSDM_Channel_HandleTypeDef      hAudioInDfsdmChannel;
DMA_HandleTypeDef                       hDmaDfsdm; 
static DFSDM_Filter_HandleTypeDef      hAudioInDfsdmFilter;

/* Recording Buffer Trigger */
static  int32_t          MicRecBuff[DEFAULT_AUDIO_IN_BUFFER_SIZE]; 


/**
  * @}
  */ 

/** @defgroup SENSORTILE_AUDIO_Private_Function_Prototypes SENSORTILE_AUDIO Private Function Prototypes
  * @{
  */
/* DFSDM Channel Msp config */
static void DFSDM_ChannelMspInit(DFSDM_Channel_HandleTypeDef *hDfsdmChannel);
static void DFSDM_ChannelMspDeInit(DFSDM_Channel_HandleTypeDef *hDfsdmChannel);

/* DFSDM Filter Msp config */
static void DFSDM_FilterMspInit(DFSDM_Filter_HandleTypeDef *hDfsdmFilter);

/* SAI Msp config */
static void SAI_MspInit(SAI_HandleTypeDef *hsai);
static void SAI_MspDeInit(SAI_HandleTypeDef *hsai);
static void SAI_InitMXConfigStruct(SAI_HandleTypeDef* hsai, MX_SAI_Config *MXConfig);

/*   */
static int32_t PCM1774_Probe(void);
/**
  * @}
  */ 


/** @defgroup STM32H747I_EVAL_AUDIO_OUT_Private_Functions STM32H747I_EVAL_AUDIO_OUT Private Functions
  * @{
  */ 
/**
  * @brief  Configures the audio peripherals.
* @param  Instance   AUDIO OUT Instance. It can only be 0 (SAI)
  * @param  AudioInit  AUDIO OUT init Structure
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_Init(uint32_t Instance, BSP_AUDIO_Init_t* AudioInit)
{
 if(Instance >= AUDIO_OUT_INSTANCES_NBR)
 {
   return BSP_ERROR_WRONG_PARAM; 
 }
 else
 {  
   /* Fill AudioOutCtx structure */
   AudioOutCtx[Instance].Device         = AudioInit->Device;
   AudioOutCtx[Instance].Instance       = Instance; 
   AudioOutCtx[Instance].SampleRate     = AudioInit->SampleRate;
   AudioOutCtx[Instance].BitsPerSample  = AudioInit->BitsPerSample;
   AudioOutCtx[Instance].ChannelsNbr    = AudioInit->ChannelsNbr;
   AudioOutCtx[Instance].Volume         = AudioInit->Volume;
   AudioOutCtx[Instance].State          = AUDIO_OUT_STATE_RESET;
   
   if (AudioOutCtx[Instance].SampleRate != AUDIO_FREQUENCY_16K 
       && AudioOutCtx[Instance].SampleRate != AUDIO_FREQUENCY_32K 
         && AudioOutCtx[Instance].SampleRate != AUDIO_FREQUENCY_48K)
   {
    return BSP_ERROR_WRONG_PARAM;
   }
   
   /* PLL clock SETUP */ 
   if(MX_SAI1_ClockConfig(&hAudioOutSai, (uint32_t) NULL) != HAL_OK)
   {
     return BSP_ERROR_CLOCK_FAILURE;
   }
   /* SAI data transfer preparation:
   Prepare the Media to be used for the audio transfer from memory to SAI peripheral */
   hAudioOutSai.Instance = AUDIO_SAIx;

   SAI_MspInit(&hAudioOutSai);
   
   MX_SAI_Config mx_sai_config;
   
   /* Prepare hAudioOutSai handle */
   mx_sai_config.Mckdiv            = SAI_CLOCK_DIVIDER(AudioInit->SampleRate);
   mx_sai_config.AudioFrequency    = SAI_AUDIO_FREQUENCY_MCKDIV;
   mx_sai_config.AudioMode         = SAI_MODEMASTER_TX;
   mx_sai_config.ClockStrobing     = SAI_CLOCKSTROBING_RISINGEDGE;
   mx_sai_config.MonoStereoMode    = SAI_STEREOMODE;
   mx_sai_config.DataSize          = SAI_DATASIZE_16;
   mx_sai_config.FrameLength       = 32; 
   mx_sai_config.ActiveFrameLength = 16;  
   mx_sai_config.OutputDrive       = SAI_OUTPUTDRIVE_ENABLE;
   mx_sai_config.Synchro           = SAI_ASYNCHRONOUS;
   mx_sai_config.SlotActive         = SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_1;
   
   /* SAI peripheral initialization: this __weak function can be redefined by the application  */
   if(MX_SAI1_Block_A_Init(&hAudioOutSai, &mx_sai_config) != HAL_OK)
   {
     return BSP_ERROR_PERIPH_FAILURE;
   }
      
   if(PCM1774_Probe()!= BSP_ERROR_NONE)
   {
     return BSP_ERROR_COMPONENT_FAILURE;
   } 
   
   /* Update BSP AUDIO OUT state */
   AudioOutCtx[Instance].State = AUDIO_OUT_STATE_STOP;
  }
  
  return BSP_ERROR_NONE; 
}

/**
  * @brief  De-initializes the audio out peripheral.
* @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @retval None
  */
int32_t BSP_AUDIO_OUT_DeInit(uint32_t Instance)
{  
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM; 
  }
  else
  {
    SAI_MspDeInit(&hAudioOutSai);
    /* Initialize the hAudioOutSai Instance parameter */
    hAudioOutSai.Instance = AUDIO_SAIx;
    /* Call the Media layer stop function */
    if(AudioDrv->DeInit(CompObj) != 0)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }   
    else if(HAL_SAI_DeInit(&hAudioOutSai) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }   
    else
    {
      /* Update BSP AUDIO OUT state */    
      AudioOutCtx[Instance].State = AUDIO_OUT_STATE_RESET;    
    } 
  }
  /* Return BSP status */
  return ret;  
}
 
/**
  * @brief  Initializes the Audio Codec audio out instance (SAI).
  * @param  MXConfig SAI configuration structure
  * @note   Being __weak it can be overwritten by the application
  * @retval HAL status
  */
__weak HAL_StatusTypeDef MX_SAI1_Block_A_Init(SAI_HandleTypeDef* hsai, MX_SAI_Config *MXConfig)
{ 
  HAL_StatusTypeDef ret = HAL_OK;
  
  /* Disable SAI peripheral to allow access to SAI internal registers */
  __HAL_SAI_DISABLE(hsai);
  
  /* Configure SAI1_Block_A */
  hsai->Init.MonoStereoMode       = MXConfig->MonoStereoMode;
  hsai->Init.AudioFrequency       = MXConfig->AudioFrequency;
  hsai->Init.AudioMode            = MXConfig->AudioMode;
  hsai->Init.NoDivider            = SAI_MASTERDIVIDER_ENABLE;
  hsai->Init.Protocol             = SAI_FREE_PROTOCOL;
  hsai->Init.DataSize             = MXConfig->DataSize;
  hsai->Init.FirstBit             = SAI_FIRSTBIT_MSB;
  hsai->Init.ClockStrobing        = MXConfig->ClockStrobing;
  hsai->Init.Synchro              = MXConfig->Synchro;
  hsai->Init.OutputDrive          = MXConfig->OutputDrive;
  hsai->Init.FIFOThreshold        = SAI_FIFOTHRESHOLD_1QF;
  hsai->Init.Mckdiv               = MXConfig->Mckdiv;
  
  /* Configure SAI_Block_x Frame */
  hsai->FrameInit.FrameLength       = MXConfig->FrameLength; 
  hsai->FrameInit.ActiveFrameLength = MXConfig->ActiveFrameLength;  
  hsai->FrameInit.FSDefinition      = SAI_FS_CHANNEL_IDENTIFICATION;
  hsai->FrameInit.FSPolarity        = SAI_FS_ACTIVE_LOW;
  hsai->FrameInit.FSOffset          = SAI_FS_BEFOREFIRSTBIT;
  
  /* Configure SAI Block_x Slot */
  hsai->SlotInit.FirstBitOffset     = 0;
  hsai->SlotInit.SlotSize           = SAI_SLOTSIZE_16B;
  hsai->SlotInit.SlotNumber         = 2;  
  hsai->SlotInit.SlotActive         = MXConfig->SlotActive;
  
  if(HAL_SAI_Init(hsai) != HAL_OK)
  {
    ret = HAL_ERROR;
  }
  
  __HAL_SAI_ENABLE(hsai);
  
  return ret;
}

/**
  * @brief  SAI clock Config.
  * @param  hsai SAI handle
  * @param  SampleRate  Audio frequency used to play the audio stream.
  * @note   This API is called by BSP_AUDIO_OUT_Init() and BSP_AUDIO_OUT_SetFrequency()
  *         Being __weak it can be overwritten by the application     
  * @retval HAL status
  */
__weak HAL_StatusTypeDef MX_SAI1_ClockConfig(SAI_HandleTypeDef *hsai, uint32_t SampleRate)
{
  HAL_StatusTypeDef ret = HAL_OK;
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsai);
  UNUSED(SampleRate);
  
  RCC_PeriphCLKInitTypeDef RCC_ExCLKInitStruct;  
  HAL_RCCEx_GetPeriphCLKConfig(&RCC_ExCLKInitStruct);
  
  /* SAI clock config 
  PLLSAI1_VCO= 8 Mhz * PLLSAI1N = 8 * 43 = VCO_344M 
  SAI_CK_x = PLLSAI1_VCO/PLLSAI1P = 344/7 = 49.142 Mhz */  
  RCC_ExCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
  RCC_ExCLKInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI1;
  RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N = 43;
  RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1CFGR_PLLSAI1PEN;
  
  if(HAL_RCCEx_PeriphCLKConfig(&RCC_ExCLKInitStruct) != HAL_OK)
  {
    ret = HAL_ERROR;
  }
  
  return ret;
}


/**
  * @brief  Starts playing audio stream from a data buffer for a determined size.
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @param  pData         pointer on data address 
  * @param  NbrOfBytes   Size of total samples in bytes
  *                      BitsPerSample: 16 or 32
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_Play(uint32_t Instance, uint8_t* pData, uint32_t NbrOfBytes)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if((Instance >= AUDIO_OUT_INSTANCES_NBR) || (((NbrOfBytes / (AudioOutCtx[Instance].BitsPerSample/8U)) > 0xFFFFU)))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if((AudioOutCtx[Instance].State == AUDIO_OUT_STATE_STOP) || (AudioOutCtx[Instance].State == AUDIO_OUT_STATE_RESET))
  {
    if(HAL_SAI_Transmit_DMA(&hAudioOutSai, pData, DMA_MAX(NbrOfBytes)) != HAL_OK)
    {  
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    if(ret == BSP_ERROR_NONE)
    {
      /* Update BSP AUDIO OUT state */    
      AudioOutCtx[Instance].State = AUDIO_OUT_STATE_PLAYING;
    }
  }
  else
  {
    ret = BSP_ERROR_BUSY;
  }
  
  /* Return BSP status */
  return ret;  
}

/**
  * @brief  This function Pauses the audio file stream. In case
  *         of using DMA, the DMA Pause feature is used.
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @note   When calling BSP_AUDIO_OUT_Pause() function for pause, only
  *          BSP_AUDIO_OUT_Resume() function should be called for resume (use of BSP_AUDIO_OUT_Play() 
  *          function for resume could lead to unexpected behavior).
  * @retval BSP status
  */  
int32_t BSP_AUDIO_OUT_Pause(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Call the Media layer pause function */   
    if(HAL_SAI_DMAPause(&hAudioOutSai) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }    
    
    if(ret == BSP_ERROR_NONE)
    {
      /* Update BSP AUDIO OUT state */    
      AudioOutCtx[Instance].State = AUDIO_OUT_STATE_PAUSE;   
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief   Resumes the audio file stream.
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @note    When calling BSP_AUDIO_OUT_Pause() function for pause, only
  *          BSP_AUDIO_OUT_Resume() function should be called for resume (use of BSP_AUDIO_OUT_Play() 
  *          function for resume could lead to unexpected behavior).
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_Resume(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Call the Media layer pause/resume function */
    if(HAL_SAI_DMAResume(&hAudioOutSai) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    
    if(ret == BSP_ERROR_NONE)
    {
      /* Update BSP AUDIO OUT state */    
      AudioOutCtx[Instance].State = AUDIO_OUT_STATE_PLAYING;
    }
  }
  
  /* Return BSP status */
  return ret;  
}

/**
  * @brief  Stops audio playing and Power down the Audio Codec.
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_Stop(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (AudioOutCtx[Instance].State == AUDIO_OUT_STATE_PLAYING)
  {
    /* Call the Media layer stop function */
    if(AudioDrv->Stop(CompObj, CODEC_PDWN_SW) != BSP_ERROR_NONE)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      if(HAL_SAI_DMAStop(&hAudioOutSai)!= HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }

      if( ret==BSP_ERROR_NONE)
      {
        /* Update BSP AUDIO OUT state */    
        AudioOutCtx[Instance].State = AUDIO_OUT_STATE_STOP;  
      }
    }    
  }  
  else
  {
    ret = BSP_ERROR_BUSY;
  }
  
  /* Return BSP status */
  return ret;
}

/**
  * @brief  Controls the current audio volume level.
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @param  Volume    Volume level to be set in percentage from 0% to 100% (0 for 
  *         Mute and 100 for Max volume level).
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_SetVolume(uint32_t Instance, uint32_t Volume)
{
  int32_t ret = BSP_ERROR_NONE;

  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {      
    /* Call the codec volume control function with converted volume value */
    if(AudioDrv->SetVolume(CompObj, 1, VOLUME_OUT_CONVERT(Volume)) != BSP_ERROR_NONE)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else if(Volume == 0U)
    {
      /* Update Mute State */
      AudioOutCtx[Instance].IsMute = BSP_AUDIO_MUTE_ENABLED;
    }
    else
    {
      /* Update Mute State */
      AudioOutCtx[Instance].IsMute = BSP_AUDIO_MUTE_DISABLED;      
    }
   AudioOutCtx[Instance].Volume = Volume;
  }

  /* Return BSP status */
  return ret;  
}

/**
  * @brief  Get the current audio volume level.
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @param  Volume    pointer to volume to be returned
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_GetVolume(uint32_t Instance, uint32_t *Volume)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    *Volume = AudioOutCtx[Instance].Volume;
  }
  /* Return BSP status */
  return ret;  
}

/**
  * @brief  Enables the MUTE
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_Mute(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR) 
  { 
    ret = BSP_ERROR_WRONG_PARAM;    
  }
  else
  {
    /* Call the Codec Mute function */
    if(AudioDrv->SetMute(CompObj, CODEC_MUTE_ON) != BSP_ERROR_NONE)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      /* Update Mute State */
      AudioOutCtx[Instance].IsMute = BSP_AUDIO_MUTE_ENABLED;
    }
  }
  /* Return BSP status */
  return ret; 
}

/**
  * @brief  Disables the MUTE mode
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_UnMute(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;    
  }
  else
  {    
    /* Call the Codec Mute function */
    if(AudioDrv->SetMute(CompObj, CODEC_MUTE_OFF) != BSP_ERROR_NONE)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      /* Update Mute State */
      AudioOutCtx[Instance].IsMute = BSP_AUDIO_MUTE_DISABLED;
    }
  }  
  /* Return BSP status */
  return ret; 
}

/**
  * @brief  Check whether the MUTE mode is enabled or not
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @param  IsMute    pointer to mute state
  * @retval Mute status
  */
int32_t BSP_AUDIO_OUT_IsMute(uint32_t Instance, uint32_t *IsMute)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    *IsMute = AudioOutCtx[Instance].IsMute; 
  }
  /* Return BSP status */
  return ret; 
}

/**
  * @brief  Switch dynamically (while audio file is played) the output target 
  *         (speaker or headphone).
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @param  Device  The audio output device
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_SetDevice(uint32_t Instance, uint32_t Device)
{
  int32_t ret;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioOutCtx[Instance].State != AUDIO_OUT_STATE_PLAYING)
  { 
    /* Call the Codec output device function */
    if(AudioDrv->SetOutputMode(CompObj, Device) != BSP_ERROR_NONE)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      MX_SAI_Config mx_out_config;
      /* Get SAI MX configuration */
      SAI_InitMXConfigStruct(&hAudioOutSai, &mx_out_config);
      if(MX_SAI1_Block_A_Init(&hAudioOutSai, &mx_out_config) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
      else
      {
        /* Update AudioOutCtx structure */
        AudioOutCtx[Instance].Device = Device;
      }
    }
  }
  else
  {
    ret = BSP_ERROR_BUSY;
  }
  
  /* Return BSP status */
  return ret; 
}

/**
  * @brief  Get the Output Device 
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @param  Device    The audio output device
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_GetDevice(uint32_t Instance, uint32_t *Device)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  { 
    /* Get AudioOutCtx Device */
    *Device = AudioOutCtx[Instance].Device;
  } 
  /* Return BSP status */
  return ret;   
}

/**
  * @brief  Updates the audio frequency.
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @param  SampleRate Audio frequency used to play the audio stream.
  * @note   This API should be called after the BSP_AUDIO_OUT_Init() to adjust the
  *         audio frequency.
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_SetSampleRate(uint32_t Instance, uint32_t SampleRate)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioOutCtx[Instance].State != AUDIO_OUT_STATE_PLAYING)
  {
    /* Call the Codec output device function */
    if(AudioDrv->SetFrequency(CompObj, SampleRate) != 0)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }    
    else
    {
      MX_SAI_Config mx_sai_config;
      
      /* Get SAI MX configuration */
      SAI_InitMXConfigStruct(&hAudioOutSai, &mx_sai_config);
      /* Update MX config structure */
      mx_sai_config.Mckdiv = SAI_CLOCK_DIVIDER(SampleRate);
      /* Update the SAI audio frequency configuration */
      hAudioOutSai.Init.Mckdiv = SAI_CLOCK_DIVIDER(SampleRate);
      /* PLL clock setup */ 
      if(MX_SAI1_ClockConfig(&hAudioOutSai, SampleRate) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
      else if(MX_SAI1_Block_A_Init(&hAudioOutSai, &mx_sai_config) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
      else
      {        
        /* Store new sample rate */
        AudioOutCtx[Instance].SampleRate = SampleRate;     
      }
    }
  }
  else
  {
    ret = BSP_ERROR_BUSY; 
  }
  
  /* Return BSP status */
  return ret;   
}

/**
  * @brief  Get the audio frequency.
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @param  SampleRate  Audio frequency used to play the audio stream.
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_GetSampleRate(uint32_t Instance, uint32_t *SampleRate)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    *SampleRate = AudioOutCtx[Instance].SampleRate;
  }   
  /* Return BSP status */
  return ret; 
}

/**
  * @brief  Get the audio Resolution.
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @param  BitsPerSample  Audio Resolution used to play the audio stream.
  * @retval BSP status
  */

int32_t BSP_AUDIO_OUT_SetBitsPerSample(uint32_t Instance, uint32_t BitsPerSample)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioOutCtx[Instance].State != AUDIO_OUT_STATE_PLAYING)
  {
    if (BitsPerSample != AUDIO_RESOLUTION_16b)      
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }    
  }
  else
  {
    ret = BSP_ERROR_BUSY;
  }
  
  /* Return BSP status */
  return ret; 
}    

/**
  * @brief  Get the audio Resolution.
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @param  BitsPerSample  Audio Resolution used to play the audio stream.
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_GetBitsPerSample(uint32_t Instance, uint32_t *BitsPerSample)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {  
    /* Get audio Out resolution */
    *BitsPerSample = AudioOutCtx[Instance].BitsPerSample;    
  }
  
  /* Return BSP status */
  return ret; 
}

/**
  * @brief  Set the audio Channels number.
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @param  ChannelNbr  Audio Channels number used to play the audio stream.
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_SetChannelsNbr(uint32_t Instance, uint32_t ChannelNbr)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if((Instance >= AUDIO_OUT_INSTANCES_NBR) || (ChannelNbr > 2U))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioOutCtx[Instance].State != AUDIO_OUT_STATE_PLAYING)
  {
    /* Set the audio Channels number */
    if(ChannelNbr != 2U)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else
    {    
    /* Store new Channel number */
    AudioOutCtx[Instance].ChannelsNbr = ChannelNbr;         
    }
  }
  else
  {
    ret = BSP_ERROR_BUSY;
  }
  
  /* Return BSP status */
  return ret; 
}

/**
  * @brief  Get the audio Channels number.
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @param  ChannelNbr     Audio Channels number used to play the audio stream.
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_GetChannelsNbr(uint32_t Instance, uint32_t *ChannelNbr)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {  
    /* Get the audio Channels number */
    *ChannelNbr = AudioOutCtx[Instance].ChannelsNbr;
  }
  
  /* Return BSP status */
  return ret; 
}

/**
  * @brief  Get Audio Out state
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @param  State     Audio Out state
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_GetState(uint32_t Instance, uint32_t *State)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {  
  /* Return audio Output State */
  *State = AudioOutCtx[Instance].State;
  }
  
  /* Return BSP status */  
  return ret;
}


/**
  * @brief  Tx Transfer completed callbacks.
  * @param  hsai SAI handle
  * @retval None
  */
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsai);
  
  /* Manage the remaining file size and new address offset: This function 
     should be coded by user (its prototype is already declared in stm32h747i_eval_audio.h) */
  BSP_AUDIO_OUT_TransferComplete_CallBack(0);
}

/**
  * @brief  Tx Half Transfer completed callbacks.
  * @param  hsai  SAI handle
  * @retval None
  */
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsai);
  
  /* Manage the remaining file size and new address offset: This function 
     should be coded by user (its prototype is already declared in stm32h747i_eval_audio.h) */
  BSP_AUDIO_OUT_HalfTransfer_CallBack(0);
}

/**
  * @brief  SAI error callbacks.
  * @param  hsai  SAI handle
  * @retval None
  */
void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsai);
  BSP_AUDIO_OUT_Error_CallBack(0);
 
}


/**
  * @brief  Manages the DMA full Transfer complete event
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @retval None
  */
__weak void BSP_AUDIO_OUT_TransferComplete_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);  
}

/**
  * @brief  Manages the DMA Half Transfer complete event
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @retval None
  */
__weak void BSP_AUDIO_OUT_HalfTransfer_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);  
}

/**
  * @brief  Manages the DMA FIFO error event
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @retval None
  */
__weak void BSP_AUDIO_OUT_Error_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);  
}

/** @defgroup SENSORTILE_AUDIO_IN_Private_Functions SENSORTILE_AUDIO_IN Private Functions
  * @{
  */ 
  
  
/**
* @brief  Initialize wave recording.
* @param  Instance  AUDIO IN Instance. It can be:
*       - 0 when I2S is used 
*       - 1 if DFSDM is used
*       - 2 if PDM is used
* @param  AudioInit Init structure
* @retval BSP status
*/
__weak int32_t BSP_AUDIO_IN_Init(uint32_t Instance, BSP_AUDIO_Init_t* AudioInit)
{
  int32_t ret =  BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;  
  }
  else
  {
    /* Store the audio record context */
    AudioInCtx[Instance].Device          = AudioInit->Device;
    AudioInCtx[Instance].ChannelsNbr     = AudioInit->ChannelsNbr;  
    AudioInCtx[Instance].SampleRate      = AudioInit->SampleRate; 
    AudioInCtx[Instance].BitsPerSample   = AudioInit->BitsPerSample;
    AudioInCtx[Instance].Volume          = AudioInit->Volume;
    AudioInCtx[Instance].State           = AUDIO_IN_STATE_RESET;
    
    if(Instance == 0U)
    { 
      ret = BSP_ERROR_WRONG_PARAM;
    }
    else if(Instance == 1U)
    {
      DFSDM_Filter_TypeDef* FilterInstnace = AUDIO_DFSDMx_MIC1_FILTER;  
      DFSDM_Channel_TypeDef* ChannelInstance = AUDIO_DFSDMx_MIC1_CHANNEL;
      uint32_t DigitalMicPins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
      uint32_t DigitalMicType = DFSDM_CHANNEL_SPI_RISING;
      uint32_t Channel4Filter = AUDIO_DFSDMx_MIC1_CHANNEL_FOR_FILTER;
      MX_DFSDM_Config dfsdm_config;
      
      /* PLL clock is set depending on the AudioFreq */
      if(MX_DFSDM1_ClockConfig(&hAudioInDfsdmChannel, AudioInit->SampleRate) != HAL_OK)
      {
        ret = BSP_ERROR_CLOCK_FAILURE;
      }
      
      DFSDM_FilterMspInit(&hAudioInDfsdmFilter);      
      DFSDM_ChannelMspInit(&hAudioInDfsdmChannel);
      
      dfsdm_config.FilterInstance  = FilterInstnace;
      dfsdm_config.ChannelInstance = ChannelInstance;
      dfsdm_config.DigitalMicPins  = DigitalMicPins;
      dfsdm_config.DigitalMicType  = DigitalMicType;
      dfsdm_config.Channel4Filter  = Channel4Filter;
      if((AudioInCtx[Instance].Device == AUDIO_IN_DIGITAL_MIC))
      {
        dfsdm_config.RegularTrigger = DFSDM_FILTER_SW_TRIGGER;    
      } 
      else
      {
        dfsdm_config.RegularTrigger = DFSDM_FILTER_SYNC_TRIGGER;
      }
      dfsdm_config.SincOrder       = DFSDM_FILTER_ORDER(AudioInCtx[Instance].SampleRate);
      dfsdm_config.Oversampling    = DFSDM_OVER_SAMPLING(AudioInCtx[Instance].SampleRate);
      dfsdm_config.ClockDivider    = DFSDM_CLOCK_DIVIDER(AudioInCtx[Instance].SampleRate);
      dfsdm_config.RightBitShift   = DFSDM_MIC_BIT_SHIFT(AudioInCtx[Instance].SampleRate);
      
      /* Default configuration of DFSDM filters and channels */
      if(MX_DFSDM1_Init(&hAudioInDfsdmFilter, &hAudioInDfsdmChannel, &dfsdm_config) != HAL_OK)
      {
        /* Return BSP_ERROR_PERIPH_FAILURE when operations are not correctly done */
        ret =  BSP_ERROR_PERIPH_FAILURE;
      }      
    }    
    
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_STOP; 
    /* Return BSP status */
  }
  return ret;
}

/**
* @brief  Deinit the audio IN peripherals.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @retval BSP status
*/

__weak int32_t BSP_AUDIO_IN_DeInit(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if(Instance == 1U)
    {
      /* De-initializes DFSDM Filter handle */
      if(hAudioInDfsdmFilter.Instance != NULL)
      {
        if(HAL_OK != HAL_DFSDM_FilterDeInit(&hAudioInDfsdmFilter))
        {
          ret = BSP_ERROR_PERIPH_FAILURE;
        }
        hAudioInDfsdmFilter.Instance = NULL;
      }
      
      /* De-initializes DFSDM Channel handle */
      if(hAudioInDfsdmChannel.Instance != NULL)
      {
        DFSDM_ChannelMspDeInit(&hAudioInDfsdmChannel);      
        if(HAL_OK != HAL_DFSDM_ChannelDeInit(&hAudioInDfsdmChannel))
        {
          ret = BSP_ERROR_PERIPH_FAILURE;
        }
        hAudioInDfsdmChannel.Instance = NULL;
      }
      
      
      /* Reset AudioInCtx[1].IsMultiBuff if any */
      AudioInCtx[1].IsMultiBuff = 0;
      
    }
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_RESET;   
  }
  /* Return BSP status */
  return ret;
}


/**
* @brief  Clock Config.
* @param  hDfsdmChannel  DFSDM Channel Handle
* @param  SampleRate     Audio frequency to be configured for the DFSDM Channel.
* @note   This API is called by BSP_AUDIO_IN_Init()
*         Being __weak it can be overwritten by the application     
* @retval HAL_status
*/
__weak HAL_StatusTypeDef MX_DFSDM1_ClockConfig(DFSDM_Channel_HandleTypeDef *hDfsdmChannel, uint32_t SampleRate)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hDfsdmChannel);
  
  HAL_StatusTypeDef ret = HAL_OK;
  RCC_PeriphCLKInitTypeDef RCC_ExCLKInitStruct;
  HAL_RCCEx_GetPeriphCLKConfig(&RCC_ExCLKInitStruct);
  
  switch (SampleRate) {
  case AUDIO_FREQUENCY_8K: {
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N = 37;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P = 17;
    break;
  }
  case AUDIO_FREQUENCY_16K: {
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N = 43;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P = 7;
    break;
  }
  case AUDIO_FREQUENCY_32K: {
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N = 43;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P = 7;
    break;
  }
  case AUDIO_FREQUENCY_48K: {
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N = 43;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P = 7;
    break;
  }
  case AUDIO_FREQUENCY_96K: {
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N = 43;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P = 7;
    break;
  }
  default: {
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N = 43;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P = 7;
    break;
  }
  }
  
  /* Configure PLLSAI prescalers */
  /* Please note that some of these parameters must be consistent with 
  the parameters of the main PLL */
  RCC_ExCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
  RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
  RCC_ExCLKInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;   
  RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1M = 6;  
  
  if(HAL_RCCEx_PeriphCLKConfig(&RCC_ExCLKInitStruct) != HAL_OK)
  {
    ret = HAL_ERROR;
  }
  return ret;
}


/**
* @brief  Initializes the Audio instance (DFSDM).
* @param  hDfsdmFilter  DFSDM Filter Handle
* @param  hDfsdmChannel DFSDM Channel Handle
* @param  SampleRate    Audio frequency to be configured for the DFSDM Channel.
* @note   Being __weak it can be overwritten by the application
* @note   Channel output Clock Divider and Filter Oversampling are calculated as follow: 
*         - Clock_Divider = CLK(input DFSDM)/CLK(micro) with
*           1MHZ < CLK(micro) < 3.2MHZ (TYP 2.4MHZ for MP34DT01TR)
*         - Oversampling = CLK(input DFSDM)/(Clock_Divider * AudioFreq)
* @retval HAL_status
*/
__weak HAL_StatusTypeDef MX_DFSDM1_Init(DFSDM_Filter_HandleTypeDef *hDfsdmFilter, DFSDM_Channel_HandleTypeDef *hDfsdmChannel, MX_DFSDM_Config *MXConfig)
{  
  HAL_StatusTypeDef ret = HAL_OK;
  /* MIC filters  initialization */
  __HAL_DFSDM_FILTER_RESET_HANDLE_STATE(hDfsdmFilter); 
  hDfsdmFilter->Instance                          = MXConfig->FilterInstance; 
  hDfsdmFilter->Init.RegularParam.Trigger         = MXConfig->RegularTrigger;
  hDfsdmFilter->Init.RegularParam.FastMode        = ENABLE;
  hDfsdmFilter->Init.RegularParam.DmaMode         = ENABLE;
  hDfsdmFilter->Init.InjectedParam.Trigger        = DFSDM_FILTER_SW_TRIGGER;
  hDfsdmFilter->Init.InjectedParam.ScanMode       = DISABLE;
  hDfsdmFilter->Init.InjectedParam.DmaMode        = DISABLE;
  hDfsdmFilter->Init.InjectedParam.ExtTrigger     = DFSDM_FILTER_EXT_TRIG_TIM8_TRGO;
  hDfsdmFilter->Init.InjectedParam.ExtTriggerEdge = DFSDM_FILTER_EXT_TRIG_BOTH_EDGES;
  hDfsdmFilter->Init.FilterParam.SincOrder        = MXConfig->SincOrder;
  hDfsdmFilter->Init.FilterParam.Oversampling     = MXConfig->Oversampling;   
  hDfsdmFilter->Init.FilterParam.IntOversampling  = 1;
  
  if(HAL_DFSDM_FilterInit(hDfsdmFilter) != HAL_OK)
  {
    ret =  HAL_ERROR;
  }  
  
  /* MIC channels initialization */
  __HAL_DFSDM_CHANNEL_RESET_HANDLE_STATE(hDfsdmChannel);
  hDfsdmChannel->Instance                      = MXConfig->ChannelInstance;  
  hDfsdmChannel->Init.OutputClock.Activation   = ENABLE;
  hDfsdmChannel->Init.OutputClock.Selection    = DFSDM_CHANNEL_OUTPUT_CLOCK_AUDIO; 
  hDfsdmChannel->Init.OutputClock.Divider      = MXConfig->ClockDivider; 
  hDfsdmChannel->Init.Input.Multiplexer        = DFSDM_CHANNEL_EXTERNAL_INPUTS;  
  hDfsdmChannel->Init.Input.DataPacking        = DFSDM_CHANNEL_STANDARD_MODE;
  hDfsdmChannel->Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL; 
  hDfsdmChannel->Init.Awd.FilterOrder          = DFSDM_CHANNEL_SINC1_ORDER;
  hDfsdmChannel->Init.Awd.Oversampling         = 10; 
  hDfsdmChannel->Init.Offset                   = 0;
  hDfsdmChannel->Init.RightBitShift            = MXConfig->RightBitShift;
  hDfsdmChannel->Init.Input.Pins               = MXConfig->DigitalMicPins; 
  hDfsdmChannel->Init.SerialInterface.Type     = MXConfig->DigitalMicType;
  
  if(HAL_OK != HAL_DFSDM_ChannelInit(hDfsdmChannel))
  {
    ret =  HAL_ERROR;
  }
  
  /* Configure injected channel */
  if(HAL_DFSDM_FilterConfigRegChannel(hDfsdmFilter, MXConfig->Channel4Filter, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    ret =  HAL_ERROR;
  } 
  
  return ret;
}


/**
* @brief  Initialize the PDM library.
* @param Instance    AUDIO IN Instance
* @param  AudioFreq  Audio sampling frequency
* @param  ChnlNbrIn  Number of input audio channels in the PDM buffer
* @param  ChnlNbrOut Number of desired output audio channels in the  resulting PCM buffer
* @retval BSP status
*/
__weak int32_t BSP_AUDIO_IN_PDMToPCM_Init(uint32_t Instance, uint32_t AudioFreq, uint32_t ChnlNbrIn, uint32_t ChnlNbrOut)
{
  
  if(Instance != 0U)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else 
  {
    return BSP_ERROR_WRONG_PARAM;
  }  
}

        

/**
* @brief  Converts audio format from PDM to PCM.
* @param  Instance  AUDIO IN Instance  
* @param  PDMBuf    Pointer to PDM buffer data
* @param  PCMBuf    Pointer to PCM buffer data
* @retval BSP status
*/
__weak int32_t BSP_AUDIO_IN_PDMToPCM(uint32_t Instance, uint16_t *PDMBuf, uint16_t *PCMBuf)
{  
  if(Instance != 0U)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else 
  {   
    return BSP_ERROR_WRONG_PARAM;
  }  
}

/**
* @brief  Start audio recording.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  pbuf     Main buffer pointer for the recorded data storing  
* @param  Size     Size of the record buffer
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_Record(uint32_t Instance, uint8_t* pBuf, uint32_t NbrOfBytes)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else 
  {
    AudioInCtx[Instance].pBuff = (uint16_t*)pBuf;
    
    if(Instance == 0U)
    {
      ret = BSP_ERROR_WRONG_PARAM;      
    }
    else
    { 
        if(HAL_DFSDM_FilterRegularStart_DMA(&hAudioInDfsdmFilter, MicRecBuff, NbrOfBytes) != HAL_OK)
        {
          ret = BSP_ERROR_PERIPH_FAILURE;
        }
      /* Update BSP AUDIO IN state */     
      AudioInCtx[Instance].State = AUDIO_IN_STATE_RECORDING;
    }
  }
  /* Return BSP status */
  return ret;
}

/**
* @brief  Stop audio recording.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_Stop(uint32_t Instance)
{
  int32_t ret;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;  
  }
  else
  {
    if(Instance == 0U)
    {
      ret = BSP_ERROR_WRONG_PARAM;      
    }
    else /*(Instance == 1U) */
    {
      /* Call the Media layer stop function */
      if(HAL_DFSDM_FilterRegularStop_DMA(&hAudioInDfsdmFilter) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
           
    }
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_STOP;
  } 
  /* Return BSP status */
  return ret;  
}

/**
* @brief  Pause the audio file stream.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_Pause(uint32_t Instance)
{
  int32_t ret;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;  
  }
  else
  {  
    if(Instance == 0U)
    { 
      ret = BSP_ERROR_WRONG_PARAM;
    }
    else /* (Instance == 1U) */
    {
      /* Call the Media layer stop function */
      if(HAL_DFSDM_FilterRegularStop_DMA(&hAudioInDfsdmFilter) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }

    }
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_PAUSE;    
  }
  /* Return BSP status */
  return ret;
}

/**
* @brief  Resume the audio file stream.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_Resume(uint32_t Instance)
{
  int32_t ret;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;  
  }
  else 
  {
    if(Instance == 0U)
    {   
      ret = BSP_ERROR_WRONG_PARAM;
    }
    else /* (Instance == 1U) */
    {
      /* Call the Media layer start function*/
      if(HAL_DFSDM_FilterRegularStart_DMA(&hAudioInDfsdmFilter, MicRecBuff, DEFAULT_AUDIO_IN_BUFFER_SIZE) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }

    }
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_RECORDING;
  }
  /* Return BSP status */
  return ret;
}

/**
* @brief  Starts audio recording.
* @param  Instance  AUDIO IN Instance. It can be 1(DFSDM used)
* @param  pBuf      Main buffer pointer for the recorded data storing
* @param  size      Size of the recorded buffer
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_RecordChannels(uint32_t Instance, uint8_t **pBuf, uint32_t NbrOfBytes)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance != 1U)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {    
    uint32_t mic_init = 0;
    uint32_t audio_in_digital_mic = AUDIO_IN_DIGITAL_MIC1, pbuf_index = 0;
    uint32_t enabled_mic = 0;
    
    /* Get the number of activated microphones */
    
    if((AudioInCtx[Instance].Device & audio_in_digital_mic) == audio_in_digital_mic)
    {
      enabled_mic++;
    }
    audio_in_digital_mic = audio_in_digital_mic << 1;
    
    
    AudioInCtx[Instance].pMultiBuff = pBuf;
    AudioInCtx[Instance].Size  = NbrOfBytes;
    AudioInCtx[Instance].IsMultiBuff = 1; 
    
    audio_in_digital_mic = AUDIO_IN_DIGITAL_MIC_LAST;
    
    if(((AudioInCtx[Instance].Device & audio_in_digital_mic) == audio_in_digital_mic) && (mic_init != 1U))
    {
      /* Call the Media layer start function for MICx channel */
      if(HAL_DFSDM_FilterRegularMsbStart_DMA(&hAudioInDfsdmFilter, (int16_t*)pBuf[enabled_mic - 1U - pbuf_index], NbrOfBytes) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
      else
      {
        mic_init = 1;
        pbuf_index++;
        ret = BSP_ERROR_NONE;   
      }
    }
    audio_in_digital_mic = audio_in_digital_mic >> 1;
    
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_RECORDING;    
    
  }
  
  /* Return BSP status */
  return ret;
}

/**
* @brief  Stop audio recording.
* @param  Instance  AUDIO IN Instance. It can be 1(DFSDM used)
* @param  Device    Digital input device to be stopped
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_StopChannels(uint32_t Instance, uint32_t Device)
{
  int32_t ret;
  
  /* Stop selected devices */
  ret = BSP_AUDIO_IN_PauseChannels(Instance, Device);
  
  if(ret == BSP_ERROR_NONE)
  {    
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_STOP;
  }
  
  /* Return BSP status */
  return ret; 
}

/**
* @brief  Pause the audio file stream.
* @param  Instance  AUDIO IN Instance. It can be 1(DFSDM used)
* @param  Device    Digital mic to be paused
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_PauseChannels(uint32_t Instance, uint32_t Device)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if((Instance != 1U) || ((Device < AUDIO_IN_DIGITAL_MIC1) && (Device > AUDIO_IN_DIGITAL_MIC_LAST)))
  {
    ret = BSP_ERROR_WRONG_PARAM;  
  }
  else
  {    
    uint32_t audio_in_digital_mic = AUDIO_IN_DIGITAL_MIC1; 
    
    if((Device & audio_in_digital_mic) == audio_in_digital_mic)
    { 
      /* Call the Media layer stop function */
      if(HAL_DFSDM_FilterRegularStop_DMA(&hAudioInDfsdmFilter) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
    }
    audio_in_digital_mic = audio_in_digital_mic << 1;
    
    
    if(ret == BSP_ERROR_NONE)
    {
      /* Update BSP AUDIO IN state */     
      AudioInCtx[Instance].State = AUDIO_IN_STATE_PAUSE;
    }
    
  }      
  
  /* Return BSP status */
  return ret;
}

/**
* @brief  Resume the audio file stream
* @param  Instance  AUDIO IN Instance. It can be 1(DFSDM used)
* @param  Device    Digital mic to be resumed
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_ResumeChannels(uint32_t Instance, uint32_t Device)
{
  int32_t ret = BSP_ERROR_NONE;
  if((Instance != 1U) || ((Device < AUDIO_IN_DIGITAL_MIC1) && (Device > AUDIO_IN_DIGITAL_MIC_LAST)))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    
    uint32_t audio_in_digital_mic = AUDIO_IN_DIGITAL_MIC_LAST;  
    
    if((Device & audio_in_digital_mic) == audio_in_digital_mic)
    { 
      /* Start selected device channel */
      if(HAL_DFSDM_FilterRegularMsbStart_DMA(&hAudioInDfsdmFilter,\
        (int16_t*)AudioInCtx[Instance].pMultiBuff[POS_VAL(audio_in_digital_mic)], AudioInCtx[Instance].Size) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
    }
    audio_in_digital_mic = audio_in_digital_mic >> 1;
    
    
    if(ret == BSP_ERROR_NONE)
    {
      /* Update BSP AUDIO IN state */     
      AudioInCtx[Instance].State = AUDIO_IN_STATE_RECORDING;
    }
    
  }
  
  /* Return BSP status */
  return ret;
}

/**
* @brief  Start audio recording.
* @param  Instance  AUDIO IN SAI PDM Instance. It can be only 2
* @param  pbuf     Main buffer pointer for the recorded data storing  
* @param  Size     Size of the record buffer
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_RecordPDM(uint32_t Instance, uint8_t* pBuf, uint32_t NbrOfBytes)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance != 2U)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  
  /* Return BSP status */
  return ret;
}


/**
* @brief  Set Audio In device
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  Device    The audio input device to be used
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_SetDevice(uint32_t Instance, uint32_t Device)
{
  int32_t ret = BSP_ERROR_NONE;
  BSP_AUDIO_Init_t audio_init;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioInCtx[Instance].State == AUDIO_IN_STATE_STOP)
  {  
    if(Instance == 1U)
    {      
      if(HAL_DFSDM_ChannelDeInit(&hAudioInDfsdmChannel) != HAL_OK)
      {
        return BSP_ERROR_PERIPH_FAILURE;
      }
    }
    audio_init.Device = Device;
    audio_init.ChannelsNbr   = AudioInCtx[Instance].ChannelsNbr;  
    audio_init.SampleRate    = AudioInCtx[Instance].SampleRate;   
    audio_init.BitsPerSample = AudioInCtx[Instance].BitsPerSample;
    audio_init.Volume        = AudioInCtx[Instance].Volume;
    
    if(BSP_AUDIO_IN_Init(Instance, &audio_init) != BSP_ERROR_NONE)
    {
      ret = BSP_ERROR_NO_INIT;
    }
  }
  else
  {
    ret = BSP_ERROR_BUSY;
  }
  
  /* Return BSP status */  
  return ret;
}

/**
* @brief  Get Audio In device
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  Device    The audio input device used
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_GetDevice(uint32_t Instance, uint32_t *Device)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {  
    /* Return audio Input Device */
    *Device = AudioInCtx[Instance].Device;
  }
  return ret;
}

/**
* @brief  Set Audio In frequency
* @param  Instance     Audio IN instance
* @param  SampleRate  Input frequency to be set
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_SetSampleRate(uint32_t Instance, uint32_t  SampleRate)
{
  int32_t ret = BSP_ERROR_NONE;
  BSP_AUDIO_Init_t audio_init;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioInCtx[Instance].State == AUDIO_IN_STATE_STOP)
  {
    if(Instance == 1U)
    {
      if(HAL_DFSDM_ChannelDeInit(&hAudioInDfsdmChannel) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }  
      if(HAL_DFSDM_FilterDeInit(&hAudioInDfsdmFilter) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
    }
    audio_init.Device        = AudioInCtx[Instance].Device;
    audio_init.ChannelsNbr   = AudioInCtx[Instance].ChannelsNbr;  
    audio_init.SampleRate    = SampleRate;   
    audio_init.BitsPerSample = AudioInCtx[Instance].BitsPerSample;
    audio_init.Volume        = AudioInCtx[Instance].Volume; 
    if(BSP_AUDIO_IN_Init(Instance, &audio_init) != BSP_ERROR_NONE)
    {
      ret = BSP_ERROR_NO_INIT;
    }   
  }
  else
  {
    ret = BSP_ERROR_BUSY;
  }
  
  /* Return BSP status */
  return ret;  
}

/**
* @brief  Get Audio In frequency
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  SampleRate  Audio Input frequency to be returned
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_GetSampleRate(uint32_t Instance, uint32_t *SampleRate)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Return audio in frequency */
    *SampleRate = AudioInCtx[Instance].SampleRate;
  }
  
  /* Return BSP status */  
  return ret;
}

/**
* @brief  Set Audio In Resolution
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  BitsPerSample  Input resolution to be set
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_SetBitsPerSample(uint32_t Instance, uint32_t BitsPerSample)
{
  int32_t ret = BSP_ERROR_NONE;
  BSP_AUDIO_Init_t audio_init;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioInCtx[Instance].State == AUDIO_IN_STATE_STOP)
  {
    if(Instance == 1U)
    {   
      if(HAL_DFSDM_ChannelDeInit(&hAudioInDfsdmChannel) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
    }
    audio_init.Device        = AudioInCtx[Instance].Device;
    audio_init.ChannelsNbr   = AudioInCtx[Instance].ChannelsNbr;  
    audio_init.SampleRate    = AudioInCtx[Instance].SampleRate;   
    audio_init.BitsPerSample = BitsPerSample;
    audio_init.Volume        = AudioInCtx[Instance].Volume; 
    if(BSP_AUDIO_IN_Init(Instance, &audio_init) != BSP_ERROR_NONE)
    {
      ret = BSP_ERROR_NO_INIT;
    }
  }
  else
  {
    ret = BSP_ERROR_BUSY;
  }
  
  /* Return BSP status */  
  return ret;
}

/**
* @brief  Get Audio In Resolution
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  BitsPerSample  Input resolution to be returned
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_GetBitsPerSample(uint32_t Instance, uint32_t *BitsPerSample)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {  
    /* Return audio in resolution */
    *BitsPerSample = AudioInCtx[Instance].BitsPerSample;
  }
  return ret;
}

/**
* @brief  Set Audio In Channel number
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  ChannelNbr  Channel number to be used
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_SetChannelsNbr(uint32_t Instance, uint32_t ChannelNbr)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if((Instance >= AUDIO_IN_INSTANCES_NBR) || (ChannelNbr > 1U))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Update AudioIn Context */
    AudioInCtx[Instance].ChannelsNbr = ChannelNbr;
  }
  /* Return BSP status */
  return ret;
}

/**
* @brief  Get Audio In Channel number
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  ChannelNbr  Channel number to be used
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_GetChannelsNbr(uint32_t Instance, uint32_t *ChannelNbr)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Channel number to be returned */
    *ChannelNbr = AudioInCtx[Instance].ChannelsNbr;
  }
  return ret;
}

/**
* @brief  Set the current audio in volume level.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  Volume    Volume level to be returnd
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_SetVolume(uint32_t Instance, uint32_t Volume)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (Instance == 0U)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Update AudioIn Context */
    AudioInCtx[Instance].Volume = Volume;
  }
  /* Return BSP status */
  return ret;  
}

/**
* @brief  Get the current audio in volume level.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  Volume    Volume level to be returnd
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_GetVolume(uint32_t Instance, uint32_t *Volume)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }  
  else
  {
    /* Input Volume to be returned */
    *Volume = AudioInCtx[Instance].Volume;
  }
  /* Return BSP status */
  return ret;  
}

/**
* @brief  Get Audio In device
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  State     Audio Out state
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_GetState(uint32_t Instance, uint32_t *State)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {  
    /* Input State to be returned */
    *State = AudioInCtx[Instance].State;
  }
  return ret;
}

/**
* @brief  This function handles DMA_REQUEST_DFSDM1_FLT0 for $DFSDM_MIC1_DMAx_REQUST$ interrupt request.
* @retval None
*/
void BSP_AUDIO_IN_DMA1_Stream4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hAudioInDfsdmFilter.hdmaReg);
}


/**
* @brief  Regular conversion complete callback. 
* @note   In interrupt mode, user has to read conversion value in this function
using HAL_DFSDM_FilterGetRegularValue.
* @param  hdfsdm_filter   DFSDM filter handle.
* @retval None
*/
void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  uint32_t i;
  
  if(AudioInCtx[1].IsMultiBuff == 1U)
  {
    /* Call the record update function to get the second half */
    BSP_AUDIO_IN_TransferComplete_CallBack(1);
  }
  else
  {   
    if(hdfsdm_filter == &hAudioInDfsdmFilter)
    {

        for (i = 0; i < (AudioInCtx[1].SampleRate / (uint32_t)1000); i++)
        {
          AudioInCtx[1].HP_Filters.Z = ((MicRecBuff[i + (AudioInCtx[1].SampleRate / (uint32_t)1000) ] /256) * (int32_t)(AudioInCtx[1].Volume)) /128;
          AudioInCtx[1].HP_Filters.oldOut = (0xFC * (AudioInCtx[1].HP_Filters.oldOut + AudioInCtx[1].HP_Filters.Z - AudioInCtx[1].HP_Filters.oldIn)) / 256;
          AudioInCtx[1].HP_Filters.oldIn = AudioInCtx[1].HP_Filters.Z;
          AudioInCtx[1].pBuff[(i * AudioInCtx[1].ChannelsNbr)] = (uint16_t) (SaturaLH(AudioInCtx[1].HP_Filters.oldOut, -32760, 32760));
        }		
      
      BSP_AUDIO_IN_TransferComplete_CallBack(1);
    }
  }
}

/**
* @brief  Half regular conversion complete callback. 
* @param  hdfsdm_filter   DFSDM filter handle.
* @retval None
*/
void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  uint32_t i;
  
  if(AudioInCtx[1].IsMultiBuff == 1U)
  {
    /* Call the record update function to get the first half */
    BSP_AUDIO_IN_HalfTransfer_CallBack(1);
  }
  else
  {
    if(hdfsdm_filter == &hAudioInDfsdmFilter)
    {
        for (i = 0; i < (AudioInCtx[1].SampleRate / (uint32_t)1000); i++)
        {
          AudioInCtx[1].HP_Filters.Z = ((MicRecBuff[i] /256) * (int32_t)(AudioInCtx[1].Volume)) /128;
          AudioInCtx[1].HP_Filters.oldOut = (0xFC * (AudioInCtx[1].HP_Filters.oldOut + AudioInCtx[1].HP_Filters.Z - AudioInCtx[1].HP_Filters.oldIn)) / 256;
          AudioInCtx[1].HP_Filters.oldIn = AudioInCtx[1].HP_Filters.Z;
          AudioInCtx[1].pBuff[(i * AudioInCtx[1].ChannelsNbr)] = (uint16_t) (SaturaLH(AudioInCtx[1].HP_Filters.oldOut, -32760, 32760));
        }	
      
      BSP_AUDIO_IN_HalfTransfer_CallBack(1);
    }
  }
}


/**
* @brief  User callback when record buffer is filled.
* @retval None
*/
__weak void BSP_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);
  
  /* This function should be implemented by the user application.
  It is called into this driver when the current buffer is filled
  to prepare the next buffer pointer and its size. */
}

/**
* @brief  Manages the DMA Half Transfer complete event.
* @retval None
*/
__weak void BSP_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);
  
  /* This function should be implemented by the user application.
  It is called into this driver when the current buffer is filled
  to prepare the next buffer pointer and its size. */
}

/**
* @brief  Audio IN Error callback function.
* @retval None
*/
__weak void BSP_AUDIO_IN_Error_CallBack(uint32_t Instance)
{ 
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);
  
  /* This function is called when an Interrupt due to transfer error on or peripheral
  error occurs. */
}

/*******************************************************************************
Static Functions
*******************************************************************************/
/**
  * @brief  Register Bus IOs if component ID is OK
  * @retval error status
  */
static int32_t PCM1774_Probe(void)
{
  int32_t ret = BSP_ERROR_NONE;
  PCM1774_IO_t              IOCtx;
  static PCM1774_Object_t   PCM1774Obj;
  
  /* Configure the audio driver */
  IOCtx.BusType     = PCM1774_I2C_BUS;
  IOCtx.Address     = PCM1774_CODEC_I2C_ADDRESS_LOW;
  IOCtx.Init        = BSP_I2C3_Init;
  IOCtx.DeInit      = BSP_I2C3_DeInit;  
  IOCtx.ReadReg     = BSP_I2C3_ReadReg;
  IOCtx.WriteReg    = BSP_I2C3_WriteReg; 
  IOCtx.GetTick     = BSP_GetTick;  
  
  if(PCM1774_RegisterBusIO (&PCM1774Obj, &IOCtx) != PCM1774_OK)
  {
    ret = BSP_ERROR_BUS_FAILURE;   
  }
  else
  {
    AudioDrv = (AUDIO_Drv_t *) (void *) &PCM1774_AUDIO_Driver;
    CompObj = &PCM1774Obj;    
  }
  
  if (AudioDrv->Init(CompObj, (void *)&AudioOutCtx[0]) != PCM1774_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }
  return ret;
} 


/**
  * @brief  Get SAI MX Init configuration
  * @param  hsai SAI handle
  * @param  MXConfig SAI configuration structure
  * @retval None
  */
static void SAI_InitMXConfigStruct(SAI_HandleTypeDef* hsai, MX_SAI_Config *MXConfig)
{
  MXConfig->AudioFrequency    = hsai->Init.AudioFrequency;
  MXConfig->ActiveFrameLength = hsai->FrameInit.ActiveFrameLength;
  MXConfig->AudioMode         = hsai->Init.AudioMode;
  MXConfig->ClockStrobing     = hsai->Init.ClockStrobing;
  MXConfig->DataSize          = hsai->Init.DataSize;
  MXConfig->FrameLength       = hsai->FrameInit.FrameLength;
  MXConfig->MonoStereoMode    = hsai->Init.MonoStereoMode;
  MXConfig->OutputDrive       = hsai->Init.OutputDrive;
  MXConfig->SlotActive        = hsai->SlotInit.SlotActive;
  MXConfig->Synchro           = hsai->Init.Synchro;
  MXConfig->SynchroExt        = hsai->Init.SynchroExt;
  MXConfig->Mckdiv            = hsai->Init.Mckdiv;
}

/**
  * @brief  Initialize BSP_AUDIO_OUT MSP.
  * @param  hsai  SAI handle 
  * @retval None
  */
static void SAI_MspInit(SAI_HandleTypeDef *hsai)
{
  static DMA_HandleTypeDef hdma_saiTx;
  GPIO_InitTypeDef  GPIO_InitStruct;  
  
  if(hsai->Instance == AUDIO_SAIx)
  {
    /* enable USB power for GPIOG */
    HAL_PWREx_EnableVddIO2();
    /* Enable SAI clock */
    AUDIO_SAIx_CLK_ENABLE();
    /* Enable GPIO clock */
    AUDIO_SAIx_MCLK_SCK_SD_FS_ENABLE();
    
    /* CODEC_SAI pins configuration: FS, SCK, MCK and SD pins ------------------*/
    GPIO_InitStruct.Pin = AUDIO_SAIx_FS_PIN | AUDIO_SAIx_SCK_PIN | AUDIO_SAIx_SD_PIN | AUDIO_SAIx_MCLK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = AUDIO_SAIx_MCLK_SCK_SD_FS_AF;
    HAL_GPIO_Init(AUDIO_SAIx_MCLK_SCK_SD_FS_GPIO_PORT, &GPIO_InitStruct);
    
    /* Enable the DMA clock */
    AUDIO_SAIx_DMAx_CLK_ENABLE();
    
    /* Configure the hdma_saiTx handle parameters */   
    /* Configure the hdma_saiTx handle parameters */   
    hdma_saiTx.Init.Request             = DMA_REQUEST_1;
    hdma_saiTx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_saiTx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_saiTx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_saiTx.Init.PeriphDataAlignment = AUDIO_SAIx_DMAx_PERIPH_DATA_SIZE;
    hdma_saiTx.Init.MemDataAlignment    = AUDIO_SAIx_DMAx_MEM_DATA_SIZE;
    hdma_saiTx.Init.Mode                = DMA_CIRCULAR;
    hdma_saiTx.Init.Priority            = DMA_PRIORITY_HIGH;
    
    hdma_saiTx.Instance = AUDIO_SAIx_DMAx_CHANNEL;
    
    /* Associate the DMA handle */
    __HAL_LINKDMA(hsai, hdmatx, hdma_saiTx);
    
    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(&hdma_saiTx);
    
    /* Configure the DMA Stream */
    HAL_DMA_Init(&hdma_saiTx);      
    
    /* SAI DMA IRQ Channel configuration */
    HAL_NVIC_SetPriority(AUDIO_SAIx_DMAx_IRQ, BSP_AUDIO_OUT_IT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(AUDIO_SAIx_DMAx_IRQ);
  }
}

/**
  * @brief  Deinitializes SAI MSP.
  * @param  hsai  SAI handle 
  * @retval HAL status
  */
static void SAI_MspDeInit(SAI_HandleTypeDef *hsai)
{
  GPIO_InitTypeDef  gpio_init_structure;
  if(hsai->Instance == AUDIO_SAIx)
  {  
    /* SAI DMA IRQ Channel deactivation */
    HAL_NVIC_DisableIRQ(AUDIO_SAIx_DMAx_IRQ);
    
    /* Deinitialize the DMA stream */
    (void)HAL_DMA_DeInit(hsai->hdmatx);
    
    /* Disable SAI peripheral */
    __HAL_SAI_DISABLE(hsai);  
    
    /* Deactivates CODEC_SAI pins FS, SCK, MCK and SD by putting them in input mode */
    gpio_init_structure.Pin = AUDIO_SAIx_FS_PIN;
    HAL_GPIO_DeInit(AUDIO_SAIx_MCLK_SCK_SD_FS_GPIO_PORT, gpio_init_structure.Pin);
    
    gpio_init_structure.Pin = AUDIO_SAIx_SCK_PIN;
    HAL_GPIO_DeInit(AUDIO_SAIx_MCLK_SCK_SD_FS_GPIO_PORT, gpio_init_structure.Pin);
    
    gpio_init_structure.Pin =  AUDIO_SAIx_SD_PIN;
    HAL_GPIO_DeInit(AUDIO_SAIx_MCLK_SCK_SD_FS_GPIO_PORT, gpio_init_structure.Pin);
    
    gpio_init_structure.Pin = AUDIO_SAIx_MCLK_PIN;
    HAL_GPIO_DeInit(AUDIO_SAIx_MCLK_SCK_SD_FS_GPIO_PORT, gpio_init_structure.Pin);
    
    /* Disable SAI clock */
    AUDIO_SAIx_CLK_DISABLE();
  }
}

/**
* @brief  Initialize the DFSDM channel MSP.
* @param  hDfsdmChannel DFSDM Channel handle
* @retval None
*/
static void DFSDM_ChannelMspInit(DFSDM_Channel_HandleTypeDef *hDfsdmChannel)
{
  GPIO_InitTypeDef  GPIO_InitStruct;  
  
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hDfsdmChannel);
  
  /* Enable DFSDM clock */                                  
  AUDIO_DFSDMx_CLK_ENABLE();                
  /* Enable GPIO clock */ 
  AUDIO_DFSDMx_CKOUT_GPIO_CLK_ENABLE(); 
  AUDIO_DFSDMx_DATIN_MIC1_GPIO_CLK_ENABLE();
  
  /* DFSDM pins configuration: DFSDM_CKOUT, DMIC_DATIN pins ------------------*/
  
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  
  GPIO_InitStruct.Pin = AUDIO_DFSDMx_CKOUT_PIN;
  GPIO_InitStruct.Alternate = AUDIO_DFSDMx_CKOUT_AF;
  HAL_GPIO_Init(AUDIO_DFSDMx_CKOUT_GPIO_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = AUDIO_DFSDMx_DATIN_MIC1_PIN;
  GPIO_InitStruct.Alternate = AUDIO_DFSDMx_DATIN_MIC1_AF;
  HAL_GPIO_Init(AUDIO_DFSDMx_DATIN_MIC1_GPIO_PORT, &GPIO_InitStruct); 
}

/**
* @brief  DeInitialize the DFSDM channel MSP.
* @param  hDfsdmChannel DFSDM Channel handle
* @retval None
*/
static void DFSDM_ChannelMspDeInit(DFSDM_Channel_HandleTypeDef *hDfsdmChannel)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hDfsdmChannel);
  
  /* DFSDM pins configuration: DFSDM_CKOUT, DMIC_DATIN pins ------------------*/
  GPIO_InitStruct.Pin = AUDIO_DFSDMx_CKOUT_PIN;
  HAL_GPIO_DeInit(AUDIO_DFSDMx_CKOUT_GPIO_PORT, GPIO_InitStruct.Pin);    
  
  GPIO_InitStruct.Pin = AUDIO_DFSDMx_DATIN_MIC1_PIN;
  HAL_GPIO_DeInit(AUDIO_DFSDMx_DATIN_MIC1_GPIO_PORT, GPIO_InitStruct.Pin);
}

/**
* @brief  Initialize the DFSDM filter MSP.
* @param  hDfsdmFilter DFSDM Filter handle
* @retval None
*/
static void DFSDM_FilterMspInit(DFSDM_Filter_HandleTypeDef *hDfsdmFilter)
{  
  uint32_t mic_init = 0;
  DMA_Channel_TypeDef* AUDIO_DFSDMx_DMAx_MIC_STREAM = AUDIO_DFSDMx_DMAx_MIC1_STREAM;
  
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hDfsdmFilter);
  
  /* Enable DFSDM clock */
  AUDIO_DFSDMx_CLK_ENABLE();                
  /* Enable the DMA clock */ 
  AUDIO_DFSDMx_DMAx_CLK_ENABLE();
  
  if(((AudioInCtx[1].Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1) && (mic_init != 1U))
  {
    mic_init = 1;
  }
  else
  {
  }  
  /* Configure the hDmaDfsdm[i] handle parameters */
  hDmaDfsdm.Init.Request             = DMA_REQUEST_0; 
  hDmaDfsdm.Instance                 = AUDIO_DFSDMx_DMAx_MIC_STREAM;
  hDmaDfsdm.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hDmaDfsdm.Init.PeriphInc           = DMA_PINC_DISABLE;
  hDmaDfsdm.Init.MemInc              = DMA_MINC_ENABLE;
  hDmaDfsdm.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hDmaDfsdm.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  hDmaDfsdm.Init.Mode                = DMA_CIRCULAR;
  hDmaDfsdm.Init.Priority            = DMA_PRIORITY_HIGH;  
  hDmaDfsdm.State                    = HAL_DMA_STATE_RESET;
  
  /* Associate the DMA handle */
  __HAL_LINKDMA(&hAudioInDfsdmFilter, hdmaReg, hDmaDfsdm);
  
  /* Reset DMA handle state */
  __HAL_DMA_RESET_HANDLE_STATE(&hDmaDfsdm);
  
  /* Configure the DMA Channel */
  (void)HAL_DMA_Init(&hDmaDfsdm);
  
  if (hDmaDfsdm.Instance == AUDIO_DFSDMx_DMAx_MIC1_STREAM)
  {
    /* DMA IRQ Channel configuration */
    HAL_NVIC_SetPriority(AUDIO_DFSDMx_DMAx_MIC1_IRQ, BSP_AUDIO_IN_IT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(AUDIO_DFSDMx_DMAx_MIC1_IRQ);
  }
  
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
