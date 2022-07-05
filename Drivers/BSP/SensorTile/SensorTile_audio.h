/**
******************************************************************************
* @file    SensorTile_audio.h
* @author  SRA - Central Labs
* @version v2.1.6
* @date    10-Feb-2022
* @brief   This file contains the common defines and functions prototypes for
*          the SensorTile_audio.c driver.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SENSORTILE_AUDIO_H
#define SENSORTILE_AUDIO_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "SensorTile_conf.h"
#include "audio.h"
#include "PCM1774.h"
#include <stdlib.h>
#include <stdio.h>


/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup SENSORTILE
  * @{
  */
    
/** @addtogroup SENSORTILE_AUDIO 
  * @{
  */
   
/** @defgroup X-NUCLEO-CCA02M1_AUDIO_Exported_Variables
 * @{
 */
extern DMA_HandleTypeDef               hDmaDfsdm;


/**
 * @}
 */   

/** @defgroup SENSORTILE_AUDIO_Exported_Types SENSORTILE_AUDIO Exported Types
  * @{
  */

typedef enum
{
  CODEX_SENSORS_AUTO = -1,    /* Always first element and equal to -1 */
  PCM1774_0 = 0                   /* . */  
} CODEX_ID_t;

typedef struct {
	int32_t Z; 
	int32_t oldOut; 
	int32_t oldIn; 
}HP_FilterState_TypeDef;  
  
typedef struct
{                                   
  uint32_t                    Device;                                           
  uint32_t                    SampleRate;                                         
  uint32_t                    BitsPerSample;                                          
  uint32_t                    ChannelsNbr;                                         
  uint32_t                    Volume;
}BSP_AUDIO_Init_t;

typedef struct
{
  uint32_t                    Instance;            /* Audio IN instance              */  
  uint32_t                    Device;              /* Audio IN device to be used     */ 
  uint32_t                    SampleRate;          /* Audio IN Sample rate           */
  uint32_t                    BitsPerSample;       /* Audio IN Sample resolution     */
  uint32_t                    ChannelsNbr;         /* Audio IN number of channel     */
  uint16_t                    *pBuff;              /* Audio IN record buffer         */
  uint8_t                     **pMultiBuff;        /* Audio IN multi-buffer          */
  uint32_t                    Size;                /* Audio IN record buffer size    */
  uint32_t                    Volume;              /* Audio IN volume                */
  uint32_t                    State;               /* Audio IN State                 */
  uint32_t                    IsMultiBuff;         /* Audio IN multi-buffer usage    */
  uint32_t                    IsMspCallbacksValid; /* Is Msp Callbacks registred     */
  HP_FilterState_TypeDef 	  HP_Filters;       /*!< HP filter state for each channel*/
  uint32_t DecimationFactor;
}AUDIO_IN_Ctx_t;
  
typedef struct
{
  uint32_t                    Instance;            /* Audio OUT instance              */  
  uint32_t                    Device;              /* Audio OUT device to be used     */ 
  uint32_t                    SampleRate;          /* Audio OUT Sample rate           */
  uint32_t                    BitsPerSample;       /* Audio OUT Sample Bit Per Sample */
  uint32_t                    Volume;              /* Audio OUT volume                */
  uint32_t                    ChannelsNbr;         /* Audio OUT number of channel     */
  uint32_t                    IsMute;              /* Mute state                      */   
  uint32_t                    State;               /* Audio OUT State                 */
  uint32_t                    IsMspCallbacksValid; /* Is Msp Callbacks registred      */ 
}AUDIO_OUT_Ctx_t;

typedef struct
{
  /* Filter parameters */
  DFSDM_Filter_TypeDef   *FilterInstance;
  uint32_t               RegularTrigger;
  uint32_t               SincOrder;   
  uint32_t               Oversampling;
  /* Channel parameters */
  DFSDM_Channel_TypeDef *ChannelInstance;
  uint32_t              DigitalMicPins;
  uint32_t              DigitalMicType;
  uint32_t              Channel4Filter;
  uint32_t              ClockDivider;
  uint32_t              RightBitShift; 
}MX_DFSDM_Config;  

typedef struct
{
  uint32_t AudioFrequency;
  uint32_t AudioMode;
  uint32_t DataSize;
  uint32_t Mckdiv;
  uint32_t MonoStereoMode;
  uint32_t ClockStrobing;
  uint32_t Synchro;
  uint32_t OutputDrive;
  uint32_t SynchroExt;
  uint32_t FrameLength;
  uint32_t ActiveFrameLength;
  uint32_t SlotActive; 
}MX_SAI_Config;

/**
  * @}
  */ 

/** @defgroup SENSORTILE_AUDIO_Exported_Constants SENSORTILE_AUDIO Exported Constants
  * @{
  */


/* AUDIO FREQUENCY */
#ifndef AUDIO_FREQUENCY_192K
#define AUDIO_FREQUENCY_192K     (uint32_t)192000U
#endif
#ifndef AUDIO_FREQUENCY_176K  
#define AUDIO_FREQUENCY_176K     (uint32_t)176400U
#endif
#ifndef AUDIO_FREQUENCY_96K
#define AUDIO_FREQUENCY_96K       (uint32_t)96000U
#endif
#ifndef AUDIO_FREQUENCY_88K
#define AUDIO_FREQUENCY_88K       (uint32_t)88200U
#endif
#ifndef AUDIO_FREQUENCY_48K
#define AUDIO_FREQUENCY_48K       (uint32_t)48000U
#endif
#ifndef AUDIO_FREQUENCY_44K  
#define AUDIO_FREQUENCY_44K       (uint32_t)44100U
#endif
#ifndef AUDIO_FREQUENCY_32K
#define AUDIO_FREQUENCY_32K       (uint32_t)32000U
#endif
#ifndef AUDIO_FREQUENCY_22K
#define AUDIO_FREQUENCY_22K       (uint32_t)22050U
#endif
#ifndef AUDIO_FREQUENCY_16K
#define AUDIO_FREQUENCY_16K       (uint32_t)16000U
#endif
#ifndef AUDIO_FREQUENCY_11K
#define AUDIO_FREQUENCY_11K       (uint32_t)11025U
#endif
#ifndef AUDIO_FREQUENCY_8K
#define AUDIO_FREQUENCY_8K         (uint32_t)8000U 
#endif
   
/* AUDIO RESOLUTION */   
#ifndef AUDIO_RESOLUTION_16b
#define AUDIO_RESOLUTION_16b                16U
#endif
#ifndef AUDIO_RESOLUTION_24b
#define AUDIO_RESOLUTION_24b                24U
#endif
#ifndef AUDIO_RESOLUTION_32b
#define AUDIO_RESOLUTION_32b                32U
#endif


#define AUDIO_OUT_INSTANCES_NBR            1U

/* Audio Mute state */
#define BSP_AUDIO_MUTE_DISABLED             0U
#define BSP_AUDIO_MUTE_ENABLED              1U

/* Audio Out states */
#define AUDIO_OUT_STATE_RESET               0U
#define AUDIO_OUT_STATE_PLAYING             1U
#define AUDIO_OUT_STATE_STOP                2U
#define AUDIO_OUT_STATE_PAUSE               3U

/* Codec commands */
#define CODEC_PDWN_SW                       1U
#define CODEC_MUTE_ON                       1U
#define CODEC_MUTE_OFF                      0U
    
/*------------------------------------------------------------------------------
                        AUDIO OUT defines parameters
------------------------------------------------------------------------------*/    
  /* SAI peripheral configuration defines */
#define AUDIO_SAIx                            SAI2_Block_A
#define AUDIO_SAIx_CLK_ENABLE()               __HAL_RCC_SAI2_CLK_ENABLE()
#define AUDIO_SAIx_CLK_DISABLE()              __HAL_RCC_SAI2_CLK_DISABLE()
#define AUDIO_SAIx_MCLK_SCK_SD_FS_AF       GPIO_AF13_SAI2
  
#define AUDIO_SAIx_MCLK_SCK_SD_FS_ENABLE()   __HAL_RCC_GPIOG_CLK_ENABLE()
#define AUDIO_SAIx_FS_PIN                      GPIO_PIN_10
#define AUDIO_SAIx_SCK_PIN                     GPIO_PIN_9
#define AUDIO_SAIx_SD_PIN                      GPIO_PIN_12
#define AUDIO_SAIx_MCLK_PIN                     GPIO_PIN_11
#define AUDIO_SAIx_MCLK_SCK_SD_FS_GPIO_PORT  GPIOG
  
  /* SAI DMA Channel definitions */
#define AUDIO_SAIx_DMAx_CLK_ENABLE()         __HAL_RCC_DMA2_CLK_ENABLE()
#define AUDIO_SAIx_DMAx_CHANNEL              DMA2_Channel3
#define AUDIO_SAIx_DMAx_IRQ                  DMA2_Channel3_IRQn
#define AUDIO_SAIx_DMAx_PERIPH_DATA_SIZE     DMA_PDATAALIGN_HALFWORD
#define AUDIO_SAIx_DMAx_MEM_DATA_SIZE        DMA_MDATAALIGN_HALFWORD
#define DMA_MAX_SZE                          (uint32_t)0xFFFF
  
#define AUDIO_SAIx_DMAx_IRQHandler           DMA2_Channel3_IRQHandler
  
  /* Select the interrupt preemption priority for the DMA interrupt */
#define AUDIO_OUT_IRQ_PREPRIO           7   /* Select the preemption priority level(0 is the highest) */   

#define DMA_MAX(_X_)                (((_X_) <= DMA_MAX_SZE)? (_X_):DMA_MAX_SZE)


/*------------------------------------------------------------------------------
                        AUDIO IN defines parameters
------------------------------------------------------------------------------*/ 

/* DFSDM Configuration defines */
#define AUDIO_DFSDMx_MIC1_CHANNEL                    DFSDM_Channel5  
#define AUDIO_DFSDMx_MIC1_CHANNEL_FOR_FILTER         DFSDM_CHANNEL_5  
#define AUDIO_DFSDMx_MIC1_FILTER                     DFSDM_Filter0
#define AUDIO_DFSDMx_CLK_ENABLE()                    __HAL_RCC_DFSDM_CLK_ENABLE()

/* DATIN for MIC1 */
#define AUDIO_DFSDMx_DATIN_MIC1_PIN                  GPIO_PIN_6
#define AUDIO_DFSDMx_DATIN_MIC1_AF                   GPIO_AF6_DFSDM
#define AUDIO_DFSDMx_DATIN_MIC1_GPIO_PORT            GPIOB
#define AUDIO_DFSDMx_DATIN_MIC1_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE() 

/* CKOUT for all mics */                                                           
#define AUDIO_DFSDMx_CKOUT_PIN                       GPIO_PIN_2
#define AUDIO_DFSDMx_CKOUT_AF                        GPIO_AF6_DFSDM
#define AUDIO_DFSDMx_CKOUT_GPIO_PORT                 GPIOC
#define AUDIO_DFSDMx_CKOUT_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOC_CLK_ENABLE()

/* DFSDM DMA MIC1 and MIC2 channels definitions */
#define AUDIO_DFSDMx_DMAx_MIC1_STREAM                DMA1_Channel4
#define AUDIO_DFSDMx_DMAx_MIC1_IRQ                   DMA1_Channel4_IRQn
#define AUDIO_DFSDM_DMAx_MIC1_IRQHandler             DMA1_Channel4_IRQHandler
#define AUDIO_DFSDMx_DMAx_PERIPH_DATA_SIZE           DMA_PDATAALIGN_WORD
#define AUDIO_DFSDMx_DMAx_MEM_DATA_SIZE              DMA_MDATAALIGN_WORD
#define AUDIO_DFSDMx_DMAx_CLK_ENABLE()               __HAL_RCC_DMA1_CLK_ENABLE()                                                     

/* Audio In devices */ 

/* MP34DT01TR digital microphone on PCB top side */
#define AUDIO_IN_DIGITAL_MIC1      0x10U
#define AUDIO_IN_DIGITAL_MIC_LAST  AUDIO_IN_DIGITAL_MIC1
#define AUDIO_IN_DIGITAL_MIC       (AUDIO_IN_DIGITAL_MIC1)
#define DFSDM_MIC_NUMBER           AUDIO_CHANNELS

/* Default Audio IN internal buffer size */   
#define DEFAULT_AUDIO_IN_BUFFER_SIZE        (AUDIO_SAMPLING_FREQUENCY/1000)*2

/*Number of millisecond of audio at each DMA interrupt*/
#define N_MS_PER_INTERRUPT               (1)

/* Audio In states */
#define AUDIO_IN_STATE_RESET               0U
#define AUDIO_IN_STATE_RECORDING           1U
#define AUDIO_IN_STATE_STOP                2U
#define AUDIO_IN_STATE_PAUSE               3U

/* Audio In instances number:
   Instance 0 is I2S / SPI path
   Instance 1 is DFSDM path
   Instance 2 is PDM path
 */
#define AUDIO_IN_INSTANCES_NBR             2U
/**
  * @}
  */
   
/** @defgroup SENSORTILE_AUDIO_Exported_Macros SENSORTILE_AUDIO Exported Macros
  * @{
  */
#define POS_VAL(VAL)                  (POSITION_VAL(VAL) - 4)
#define VOLUME_OUT_CONVERT(Volume)    (((Volume) > 100)? 63:((uint8_t)(((Volume) * 63) / 100)))
#define VOLUME_IN_CONVERT(Volume)     (((Volume) >= 100)? 239:((uint8_t)(((Volume) * 239) / 100)))
    
/**
  * @}
  */ 
/** @addtogroup SENSORTILE_AUDIO_Exported_Variables
  * @{
  */
/* Recording context */
extern AUDIO_IN_Ctx_t                         AudioInCtx[];
/**
  * @}
  */

/** @defgroup SENSORTILE_AUDIO_Exported_Functions SENSORTILE_AUDIO_IN Exported Functions
  * @{
  */
int32_t BSP_AUDIO_OUT_Init(uint32_t Instance, BSP_AUDIO_Init_t* AudioInit);    
int32_t BSP_AUDIO_OUT_DeInit(uint32_t Instance);

int32_t BSP_AUDIO_OUT_Play(uint32_t Instance, uint8_t* pData, uint32_t NbrOfBytes);
int32_t BSP_AUDIO_OUT_Pause(uint32_t Instance);
int32_t BSP_AUDIO_OUT_Resume(uint32_t Instance);
int32_t BSP_AUDIO_OUT_Stop(uint32_t Instance);
int32_t BSP_AUDIO_OUT_Mute(uint32_t Instance);
int32_t BSP_AUDIO_OUT_UnMute(uint32_t Instance);
int32_t BSP_AUDIO_OUT_IsMute(uint32_t Instance, uint32_t *IsMute);
int32_t BSP_AUDIO_OUT_SetDevice(uint32_t Instance, uint32_t Device);
int32_t BSP_AUDIO_OUT_GetDevice(uint32_t Instance, uint32_t *Device);
int32_t BSP_AUDIO_OUT_SetSampleRate(uint32_t Instance, uint32_t SampleRate);
int32_t BSP_AUDIO_OUT_GetSampleRate(uint32_t Instance, uint32_t *SampleRate);
int32_t BSP_AUDIO_OUT_SetBitsPerSample(uint32_t Instance, uint32_t BitsPerSample);
int32_t BSP_AUDIO_OUT_GetBitsPerSample(uint32_t Instance, uint32_t *BitsPerSample);
int32_t BSP_AUDIO_OUT_SetChannelsNbr(uint32_t Instance, uint32_t ChannelNbr);
int32_t BSP_AUDIO_OUT_GetChannelsNbr(uint32_t Instance, uint32_t *ChannelNbr);
int32_t BSP_AUDIO_OUT_SetVolume(uint32_t Instance, uint32_t Volume);
int32_t BSP_AUDIO_OUT_GetVolume(uint32_t Instance, uint32_t *Volume);
int32_t BSP_AUDIO_OUT_GetState(uint32_t Instance, uint32_t *State);

/* User Callbacks: user has to implement these functions in his code if they are needed. */
/* This function is called when the requested data has been completely transferred.*/
void    BSP_AUDIO_OUT_TransferComplete_CallBack(uint32_t Instance);

/* This function is called when half of the requested buffer has been transferred. */
void    BSP_AUDIO_OUT_HalfTransfer_CallBack(uint32_t Instance);

/* This function is called when an Interrupt due to transfer error on or peripheral
   error occurs. */
void    BSP_AUDIO_OUT_Error_CallBack(uint32_t Instance);

/* These function can be modified in case the current settings need to be changed 
   for specific application needs */
HAL_StatusTypeDef MX_SAI1_ClockConfig(SAI_HandleTypeDef *hsai, uint32_t SampleRate);
HAL_StatusTypeDef MX_SAI1_Block_A_Init(SAI_HandleTypeDef* hsai, MX_SAI_Config *MXConfig);


int32_t BSP_AUDIO_IN_Init(uint32_t Instance, BSP_AUDIO_Init_t* AudioInit);    
int32_t BSP_AUDIO_IN_DeInit(uint32_t Instance);
int32_t BSP_AUDIO_IN_Record(uint32_t Instance, uint8_t* pBuf, uint32_t NbrOfBytes);
int32_t BSP_AUDIO_IN_Stop(uint32_t Instance);
int32_t BSP_AUDIO_IN_Pause(uint32_t Instance);
int32_t BSP_AUDIO_IN_Resume(uint32_t Instance);

int32_t BSP_AUDIO_IN_RecordChannels(uint32_t Instance, uint8_t **pBuf, uint32_t NbrOfBytes);
int32_t BSP_AUDIO_IN_StopChannels(uint32_t Instance, uint32_t Device);
int32_t BSP_AUDIO_IN_PauseChannels(uint32_t Instance, uint32_t Device);
int32_t BSP_AUDIO_IN_ResumeChannels(uint32_t Instance, uint32_t Device);

int32_t BSP_AUDIO_IN_SetDevice(uint32_t Instance, uint32_t Device);
int32_t BSP_AUDIO_IN_GetDevice(uint32_t Instance, uint32_t *Device);
int32_t BSP_AUDIO_IN_SetSampleRate(uint32_t Instance, uint32_t SampleRate);
int32_t BSP_AUDIO_IN_GetSampleRate(uint32_t Instance, uint32_t *SampleRate);                 
int32_t BSP_AUDIO_IN_SetBitsPerSample(uint32_t Instance, uint32_t BitsPerSample);
int32_t BSP_AUDIO_IN_GetBitsPerSample(uint32_t Instance, uint32_t *BitsPerSample);                
int32_t BSP_AUDIO_IN_SetChannelsNbr(uint32_t Instance, uint32_t ChannelNbr);
int32_t BSP_AUDIO_IN_GetChannelsNbr(uint32_t Instance, uint32_t *ChannelNbr);
int32_t BSP_AUDIO_IN_SetVolume(uint32_t Instance, uint32_t Volume);
int32_t BSP_AUDIO_IN_GetVolume(uint32_t Instance, uint32_t *Volume);
int32_t BSP_AUDIO_IN_GetState(uint32_t Instance, uint32_t *State);

/* Specific PDM recodr APIs */
int32_t BSP_AUDIO_IN_PDMToPCM_Init(uint32_t Instance, uint32_t AudioFreq, uint32_t ChnlNbrIn, uint32_t ChnlNbrOut);
int32_t BSP_AUDIO_IN_PDMToPCM(uint32_t Instance, uint16_t *PDMBuf, uint16_t *PCMBuf);
int32_t BSP_AUDIO_IN_RecordPDM(uint32_t Instance, uint8_t* pBuf, uint32_t NbrOfBytes);

void BSP_AUDIO_IN_DMA1_Stream4_IRQHandler(void);
void BSP_AUDIO_IN_DMA1_Stream5_IRQHandler(void);
void BSP_AUDIO_IN_DMA1_Stream6_IRQHandler(void);
void BSP_AUDIO_IN_DMA1_Stream7_IRQHandler(void);

/* User Callbacks: user has to implement these functions in his code if they are needed. */
/* This function should be implemented by the user application.
   It is called into this driver when the current buffer is filled to prepare the next
   buffer pointer and its size. */
void BSP_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance);
void BSP_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance);

/* This function is called when an Interrupt due to transfer error on or peripheral
   error occurs. */
void BSP_AUDIO_IN_Error_CallBack(uint32_t Instance);

/* These function can be modified in case the current settings (e.g. DMA stream)
   need to be changed for specific application needs */
HAL_StatusTypeDef MX_DFSDM1_ClockConfig(DFSDM_Channel_HandleTypeDef *hDfsdmChannel, uint32_t SampleRate);
HAL_StatusTypeDef MX_DFSDM1_Init(DFSDM_Filter_HandleTypeDef *hDfsdmFilter, DFSDM_Channel_HandleTypeDef *hDfsdmChannel, MX_DFSDM_Config *MXConfig);

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

#ifdef __cplusplus
}
#endif

#endif /* SENSORTILE_AUDIO_H */
