/**
  ******************************************************************************
  * @file    usbd_cdc_interface.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   Source file for USBD CDC interface
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

#ifdef ALLMEMS1_ENABLE_PRINTF

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define APP_RX_DATA_SIZE  128
#define APP_TX_DATA_SIZE  512

/* Private macro -------------------------------------------------------------*/
/* Shared variables ----------------------------------------------------------*/
extern volatile uint8_t VCOM_RxData;
extern volatile uint8_t *VCOM_RxBuffer;
extern volatile uint32_t VCOM_RxLength;

USBD_CDC_LineCodingTypeDef LineCoding =
{
  115200, /* baud rate */
  0x00,   /* stop bits - 1 */
  0x00,   /* parity - none */
  0x08    /* nb. of bits 8 */
};

uint8_t UserRxBuffer[APP_RX_DATA_SIZE];
uint8_t UserTxBuffer[APP_TX_DATA_SIZE]; /* Data to be sent via USB - circular
                                           buffer: can hold at most
                                           (APP_TX_DATA_SIZE - 1) data elements */

volatile uint32_t UserTxBufPtrOut = 0; /* Points to beginning of data
                                          to be sent via USB */
                                       /* NOTE: Changed by callback function */

uint32_t UserTxBufPtrIn           = 0; /* Points to end of data
                                          to be sent via USB */

/* USB handler declaration */
extern USBD_HandleTypeDef  USBD_Device;

/* Shared function prototypes ------------------------------------------------*/
extern void Error_Handler(void);


/* Private function prototypes -----------------------------------------------*/
static int8_t CDC_Itf_Init(void);
static int8_t CDC_Itf_DeInit(void);
static int8_t CDC_Itf_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Itf_Receive(uint8_t *pbuf, uint32_t *Len);

USBD_CDC_ItfTypeDef USBD_CDC_fops =
{
  CDC_Itf_Init,
  CDC_Itf_DeInit,
  CDC_Itf_Control,
  CDC_Itf_Receive
};


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  CDC_Itf_Init
  *         Initializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Init(void)
{
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&USBD_Device, UserTxBuffer, 0);
  USBD_CDC_SetRxBuffer(&USBD_Device, UserRxBuffer); 
  return (USBD_OK);
}



/**
  * @brief  CDC_Itf_DeInit
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_DeInit(void)
{
  return (USBD_OK);
}



/**
  * @brief  CDC_Itf_Control
  *         Manage the CDC class requests
  * @param  Cmd: Command code
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
  switch (cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:
    /* Add your code here */
    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:
    /* Add your code here */
    break;

  case CDC_SET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_GET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_CLEAR_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_SET_LINE_CODING:
    LineCoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |\
                            (pbuf[2] << 16) | (pbuf[3] << 24));
    LineCoding.format     = pbuf[4];
    LineCoding.paritytype = pbuf[5];
    LineCoding.datatype   = pbuf[6];
    break;

  case CDC_GET_LINE_CODING:
    pbuf[0] = (uint8_t)(LineCoding.bitrate);
    pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
    pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
    pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
    pbuf[4] = LineCoding.format;
    pbuf[5] = LineCoding.paritytype;
    pbuf[6] = LineCoding.datatype;
    break;

  case CDC_SET_CONTROL_LINE_STATE:
    /* Add your code here */
    break;

  case CDC_SEND_BREAK:
     /* Add your code here */
    break;

  default:
    break;
  }

  return (USBD_OK);
}

/**
  * @brief  Fill the USB TX buffer
  * @param  Buf: pointer to the TX buffer
  * @param  TotalLen: number of bytes to be sent
  * @retval Result of the operation: USBD_OK if all operations are OK,
  *         else USBD_FAIL
  */
uint8_t CDC_Fill_Buffer(uint8_t *Buf, uint32_t TotalLen)
{
  uint32_t i;

  for (i = 0; i < TotalLen; i++)
  {
    /* ERROR: Buffer overrun.
       NOTE: Write pointer 'UserTxBufPtrIn' points to last free position, which
       is -1 position from read pointer 'UserTxBufPtrOut' and MUST NEVER be written.
    */
    if ((UserTxBufPtrIn + 1) % APP_TX_DATA_SIZE == UserTxBufPtrOut)
    {
      return (USBD_FAIL);
    }

    UserTxBuffer[UserTxBufPtrIn] = Buf[i];
    UserTxBufPtrIn = (UserTxBufPtrIn + 1) % APP_TX_DATA_SIZE;
  }
  CDC_PeriodElapsedCallback();

  return (USBD_OK);
}



/**
  * @brief  Initiate next USB packet transfer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK,
  *         else USBD_FAIL
  */
uint8_t CDC_Next_Packet_Rx(void)
{

  /* VCOM data overrun - current VCOM data received are not complete read by application. */
  if (VCOM_RxData == 1)
  {
    Error_Handler();
  }

  return USBD_CDC_ReceivePacket(&USBD_Device);
}



/**
  * @brief  CDC_period elapsed callback
  * @param  htim: CDC_TIM handle
  * @retval None
  */
void CDC_PeriodElapsedCallback(void)
{
  uint32_t buffptr;
  uint32_t buffsize;

  if(UserTxBufPtrOut != UserTxBufPtrIn)
  {
    if(UserTxBufPtrOut > UserTxBufPtrIn) /* Rollback */
    {
      buffsize = APP_TX_DATA_SIZE - UserTxBufPtrOut;
    }
    else
    {
      buffsize = UserTxBufPtrIn - UserTxBufPtrOut;
    }

    buffptr = UserTxBufPtrOut;

    USBD_CDC_SetTxBuffer(&USBD_Device, &UserTxBuffer[buffptr], buffsize);

    if (USBD_CDC_TransmitPacket(&USBD_Device) == USBD_OK)
    {
      UserTxBufPtrOut += buffsize;
      if (UserTxBufPtrOut >= APP_TX_DATA_SIZE)
      {
        UserTxBufPtrOut = 0;
      }
    }
  }
}



/**
  * @brief  CDC_Itf_DataRx
  *         Data received over USB OUT endpoint - callback.
  * @param  Buf: Buffer of data received via USB
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Receive(uint8_t *Buf, uint32_t *Len)
{
  VCOM_RxData   = 1;
  VCOM_RxBuffer = Buf;
  VCOM_RxLength = *Len;
  /* Make Echo */
  CDC_Fill_Buffer(Buf, *Len);
  return (USBD_OK);
}

#endif /* ALLMEMS1_ENABLE_PRINTF */
