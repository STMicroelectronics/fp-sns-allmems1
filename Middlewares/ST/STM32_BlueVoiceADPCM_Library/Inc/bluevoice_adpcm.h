/**
  ******************************************************************************
  * @file    bluevoice_adpcm.h
  * @author  Central Labs
  * @version V2.0.0
  * @date    18-April-2017
  * @brief   This file contains definitions for BlueVoice ADPCM profile.
  *
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
#ifndef __BLUEVOICE_ADPCM_H
#define __BLUEVOICE_ADPCM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "bluenrg_aci_const.h"
#include "bluenrg_gatt_aci.h" 

/** @addtogroup MIDDLEWARES
  * @{
  */

/** @defgroup BV_ADPCM BV_ADPCM
* @{
*/ 

/** @defgroup BV_ADPCM_Exported_Enums BV_ADPCM_Exported_Enums
  * @{
 */

/** @addtogroup BV_ADPCM_Frequencies
  * @{
  */

/** 
* @brief Accepted audio in sampling frequencies.
*/
typedef enum
{
  FR_8000 = 8000,                               /*!< Sampling frequency 8000 kHz.*/
  FR_16000 = 16000                              /*!< Sampling frequency 16000 kHz.*/
} Sampling_fr_t;
/**
  * @}
  */ 

/** @addtogroup BV_ADPCM_Return_States
  * @{
  */

/** 
* @brief Return states values.
*/
typedef enum 
{
  BV_ADPCM_SUCCESS = 0x00,                        /*!< Success.*/
  BV_ADPCM_LOCKER_ERROR = 0x11,                   /*!< The library locker authenticate failed CRC must be enabled.*/
  BV_ADPCM_ERROR = 0x01,                          /*!< Error.*/
  BV_ADPCM_DISCONNETED = 0x10,                    /*!< BLE Disconnected Error.*/
  BV_ADPCM_NOT_CONFIG = 0x20,                     /*!< BV_ADPCM library not configured.*/
  BV_ADPCM_RECEIVER_DISABLE = 0x30,               /*!< Receiver mode disabled.*/
  BV_ADPCM_TRANSMITTER_DISABLE = 0x31,            /*!< Transmitter mode disabled.*/
  BV_ADPCM_RX_HANDLE_NOT_AVAILABLE = 0x32,        /*!< The handle isn't recognized.*/
  BV_ADPCM_OUT_BUF_READY = 0x40,                  /*!< The audio out buffer is ready to be sent.*/
  BV_ADPCM_OUT_BUF_NOT_READY = 0x41,              /*!< The audio out buffer is not ready to be sent.*/
  BV_ADPCM_PCM_SAMPLES_ERR = 0x50,                /*!< The number of PCM samples given as audio input is not correct.*/
  BV_ADPCM_NOTIF_DISABLE = 0x60,                  /*!< The notifications are disabled.*/
  BV_ADPCM_TIMEOUT = 0x70                         /*!< A BLE timeout occurred.*/
} BV_ADPCM_Status;
/**
  * @}
  */ 

/** @addtogroup BV_ADPCM_Mode
  * @{
  */

/** 
* @brief BlueVoice ADPCM working modalities.
*/
typedef enum
{
  NOT_READY = 0x00,                                /*!< Device not ready.*/
  TRANSMITTER = 0x01,                              /*!< Device in Transmitter mode.*/
  RECEIVER = 0x02,                                 /*!< Device in Receiver mode.*/
  HALF_DUPLEX = 0x03                               /*!< Device in HalfDuplex mode.*/
} BV_ADPCM_Mode;
/**
  * @}
  */ 

/** @addtogroup BV_ADPCM_Profile_States
  * @{
  */

/** 
* @brief BlueVoice ADPCM profile status.
*/
typedef enum
{
  BV_ADPCM_STATUS_UNITIALIZED = 0x00,      /*!< BlueVoice Profile is not initialized.*/
  BV_ADPCM_STATUS_INITIALIZED = 0x10,      /*!< BlueVoice Profile is initialized.*/
  BV_ADPCM_STATUS_READY = 0x20,            /*!< BlueVoice channel is ready and idle.*/
  BV_ADPCM_STATUS_STREAMING = 0x30,        /*!< BlueVoice device is streaming data.*/
  BV_ADPCM_STATUS_RECEIVING = 0x40         /*!< BlueVoice device is receiving data.*/
} BV_ADPCM_Profile_Status;

/**
  * @}
  */

/**
  * @}
  */ 

/** @defgroup BV_ADPCM_Exported_Types BV_ADPCM_Exported_Types
  * @{
 */

/** @addtogroup BV_ADPCM_Configuration
  * @{
  */

/** 
* @brief BlueVoice ADPCM profile configuration parameters.
*/
typedef struct
{   
  Sampling_fr_t sampling_frequency;     /*!< Specifies the sampling frequency in kHz - can be 16 kHz or 8 kHz.*/
  
  uint8_t channel_tot;                  /*!< Number of channels contained in the buffer given as Audio Input.*/
  
  uint8_t channel_in;                   /*!< The chosen channel among the available in the input buffer.*/

} BV_ADPCM_Config_t;
 /**
  * @}
  */ 

/** @addtogroup BV_ADPCM_UUIDs
  * @{
  */

/** 
* @brief BlueVoice ADPCM profile configuration parameters.
*/
typedef struct
{
  uint8_t ServiceUUID[16];              /*!< Service UUID.*/
 
  uint8_t CharAudioUUID[16];            /*!< Audio characteristic UUID.*/
  
  uint8_t CharAudioSyncUUID[16];        /*!< Audio sync characteristic UUID.*/

} BV_ADPCM_uuid_t;

 /**
  * @}
  */ 

/** @addtogroup BV_ADPCM_Handlers
  * @{
  */

/** 
* @brief BlueVoice ADPCM profile configuration parameters.
*/
typedef struct
{
  uint16_t ServiceHandle;               /*!< Service handle.*/
 
  uint16_t CharAudioHandle;             /*!< Audio characteristic handle.*/
  
  uint16_t CharAudioSyncHandle;         /*!< Audio Sync characteristic handle.*/

} BV_ADPCM_ProfileHandle_t;

/**
* @}
*/ 

/**
  * @}
  */ 

/** @defgroup BV_ADPCM_Exported_Constants BV_ADPCM_Exported_Constants
  * @{
  */ 

/** @addtogroup BV_ADPCM_Timeouts
  * @{
  */
#define BV_ADPCM_TIMEOUT_STATUS                 ((uint16_t)100)      /*!< status timeout (in ms), the function "BluevoiceADPCM_IncTick" must be 
                                                                               called every 1 ms. If timeout expires, the device goes from receiving/streaming 
                                                                               to ready mode.*/
                                                                            
/**
  * @}
  */

/** @addtogroup BV_ADPCM_PCM_samples_packet
* @{
*/
#define BV_ADPCM_PCM_SAMPLES_PER_PACKET            (40)                 /*!< Number of PCM samples contained in a single BlueVoice audio packet .*/

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup BV_ADPCM_Exported_Functions BV_ADPCM_Exported_Functions
  * @{
  */

/**
  * @brief  This function is called to initialize the bluevoice library.
  * @param  None.
  * @retval BV_ADPCM_Status: BV_ADPCM_SUCCESS if the configuration is ok, BV_ADPCM_ERROR otherwise.
  */
BV_ADPCM_Status BluevoiceADPCM_Initialize(void);
    
/**
  * @brief  This function is called to set the configuration parameters 
  * @param  BV_ADPCM_Config: It contains the configuration parameters.
  * @retval BV_ADPCM_Status: BV_ADPCM_SUCCESS if the configuration is ok, BV_ADPCM_ERROR otherwise.
  */
BV_ADPCM_Status BluevoiceADPCM_SetConfig(BV_ADPCM_Config_t *BV_ADPCM_Config);

/**
  * @brief  This function returns if the BlueVoice Profile is configured.
  * @param  None.
  * @retval uint8_t: 1 if the profile is configured 0 otherwise.
  */
uint8_t BluevoiceADPCM_IsProfileConfigured(void);
  
/**
  * @brief  This function is called to add BlueVoice Service.
  * @param  service_uuid: Service uuid value.
  * @param  service_handle: Pointer to a variable in which the service handle will be saved.
  * @retval BV_ADPCM_Status: Value indicating success or error code.
  */
BV_ADPCM_Status BluevoiceADPCM_AddService(uint8_t *service_uuid, uint16_t *service_handle);

/**
  * @brief  This function is called to add BlueVoice characteristics.
  * @param  uuid: It contains the uuid values of the Audio and audioSync characteristics.
  * @param  service_handle: Handle of the service to which the characteristic must be added.
  * @param  handle: Pointer to a BV_ADPCM_ProfileHandle_t struct in which the handles will be saved.
  * @retval BV_ADPCM_Status: BV_ADPCM_SUCCESS in case of success, BV_ADPCM_ERROR otherwise.
  */
BV_ADPCM_Status BluevoiceADPCM_AddChar(BV_ADPCM_uuid_t uuid, uint16_t service_handle, BV_ADPCM_ProfileHandle_t *handle);

/**
  * @brief  This function is called to set the handles if the BlueVoice characteristics are added out of the library.
  * @param  tx_handle: Pointer to a BV_ADPCM_ProfileHandle_t struct in which the handles are stored.
  * @retval BV_Status: BV_ADPCM_SUCCESS in case of success, BV_ADPCM_ERROR otherwise.
  */
BV_ADPCM_Status BluevoiceADPCM_SetTxHandle(BV_ADPCM_ProfileHandle_t *tx_handle); 
  
/**
  * @brief  This function is called to set the handles discovered, if an other BlueVoice module is available.
  * @param  rx_handle: Pointer to a BV_ADPCM_ProfileHandle_t struct in which the handles are stored.
  * @retval BV_Status: BV_ADPCM_SUCCESS in case of success, BV_ADPCM_ERROR otherwise.
  */
BV_ADPCM_Status BluevoiceADPCM_SetRxHandle(BV_ADPCM_ProfileHandle_t *rx_handle);

/**
  * @brief  This function returns the BlueVoice ADPCM Profile State Machine status.
  * @param  None.
  * @retval BV_ADPCM_Profile_Status: BlueVoice ADPCM Profile Status.
  */
BV_ADPCM_Profile_Status BluevoiceADPCM_GetStatus(void);

/**
  * @brief  This function returns the current modality.
  * @param  None.
  * @retval BV_ADPCM_Mode: Current working modality: NOT_READY, RECEIVER, TRANSMITTER or HALF_DUPLEX.
  */
BV_ADPCM_Mode BluevoiceADPCM_GetMode(void);
   
/**
  * @brief  This function increases the the internal counter, used to switch from Receiving/Streaming to Ready status.
  * @param  None.
  * @retval BV_ADPCM_Status: Value indicating success or error code.
  */
BV_ADPCM_Status BluevoiceADPCM_IncTick(void);
  
/**
  * @brief  This function is called to enable notifications mechanism.
  * @param  None.
  * @retval BV_ADPCM_Status: Value indicating success or error code.
  */
BV_ADPCM_Status BluevoiceADPCM_EnableNotification(void);
  
/**
  * @brief  This function is called to fill audio buffer.
  * @param  buffer: Audio in PCM buffer.
  * @param  Nsamples: Number of PCM 16 bit audio samples for each channel.
  * @retval BV_ADPCM_Status: Value indicating success or error code.
  */
BV_ADPCM_Status BluevoiceADPCM_AudioIn(uint16_t* buffer, uint8_t Nsamples);
  
/**
  * @brief  This function must be called when the compressed audio data are ready, 
  *         (when the function BluevoiceADPCM_AudioIn returns BV_ADPCM_OUT_BUF_READY)
  * @param  NbyteSent: Number of bytes sent.
  * @retval BV_ADPCM_Status: Value indicating success or error code.
  */
BV_ADPCM_Status BluevoiceADPCM_SendData(uint16_t *NbyteSent);
  
/**
  * @brief  This function is called to parse received data.
  * @param  buffer_in: 8-bit packed ADPCM samples source buffer.
  * @param  Len: Dimension in Bytes.
  * @param  attr_handle: Handle of the updated characteristic.
  * @param  buffer_out: 16-bit PCM samples destination buffer.
  * @param  samples: Number of 16-bit PCM samples in the destination buffer.
  * @retval BV_ADPCM_Status: Value indicating success or error code.
  */
BV_ADPCM_Status BluevoiceADPCM_ParseData(uint8_t* buffer_in, uint32_t Len, uint16_t attr_handle, uint8_t* buffer_out, uint8_t *samples);

/**
  * @}
  */ 
  
/** @defgroup BV_ADPCM_Exported_Callbacks BV_ADPCM_Exported_Callbacks
* @{
*/
  
/**
  * @brief  This function must be called when there is a LE Connection Complete event.
  * @param  handle: Connection handle.
  * @retval None.
  */
void BluevoiceADPCM_ConnectionComplete_CB(uint16_t handle);

/**
  * @brief  This function must be called when there is a LE disconnection Complete event. 
  * @param  None.
  * @retval None.
  */
void BluevoiceADPCM_DisconnectionComplete_CB(void);

/**
  * @brief  This function must be called when there is a LE attribut modified event. 
  * @param  attr_handle: Attribute handle.
  * @param  attr_len: Attribute length.
  * @param  attr_value: Attribute value.
  * @retval None.
  */
void BluevoiceADPCM_AttributeModified_CB(uint16_t attr_handle, uint8_t attr_len, uint8_t *attr_value);

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

#endif /* __BLUEVOICE_ADPCM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
