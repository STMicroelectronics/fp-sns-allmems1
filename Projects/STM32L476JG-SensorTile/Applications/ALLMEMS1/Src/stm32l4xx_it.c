/**
  ******************************************************************************
  * @file    stm32l4xx_it.c 
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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
#include "stm32l4xx_it.h"
//#include "DataLog_Manager.h"

/* Imported variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef    TimEnvHandle;
extern TIM_HandleTypeDef    TimCCHandle;
extern TIM_HandleTypeDef    TimAudioDataHandle;

#ifdef ALLMEMS1_MOTIONFA
extern TIM_HandleTypeDef    TimFitnessActivitiesHandle;
#endif /* ALLMEMS1_MOTIONFA */

#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
extern SPI_HandleTypeDef SPI_SD_Handle;
extern TIM_HandleTypeDef    TimSdRecordingHandle;

/* RTC handler declared in "main.c" file */
extern RTC_HandleTypeDef RtcHandle;
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */
  
#ifdef ALLMEMS1_ENABLE_PRINTF
  extern PCD_HandleTypeDef hpcd;
  extern TIM_HandleTypeDef  TimHandle;
#endif /* ALLMEMS1_ENABLE_PRINTF */

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/*                 STM32L4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l4xxxx.s).                                             */
/******************************************************************************/

#ifdef ALLMEMS1_MOTIONFA
/**
  * @brief  This function handles TIM3 interrupt request.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimFitnessActivitiesHandle);
}
#endif /* ALLMEMS1_MOTIONFA */

/**
  * @brief  This function handles TIM4 interrupt request.
  * @param  None
  * @retval None
  */
void TIM4_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimEnvHandle);
}

/**
  * @brief  This function handles TIM5 interrupt request.
  * @param  None
  * @retval None
  */
void TIM5_IRQHandler(void)
{  
  HAL_TIM_IRQHandler(&TimAudioDataHandle);
}

/**
  * @brief  This function handles TIM1 Interrupt request
  * @param  None
  * @retval None
  */
void TIM1_CC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimCCHandle);
}

#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
/**
  * @brief  This function handles TIM interrupt request.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimSdRecordingHandle);
}

/**
  * @brief  This function handles DMA Tx interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Channel2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(SPI_SD_Handle.hdmatx);
}

/**
  * @brief  This function handles RTC Alarm interrupt request.
  * @param  None
  * @retval None
  */
void RTC_Alarm_IRQHandler(void)
{
  HAL_RTC_AlarmIRQHandler(&RtcHandle);
}
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */

/**
  * @brief This function handles DFSDM Left DMAinterrupt request.
  * @param None
  * @retval None
  */
void AUDIO_DFSDM_DMAx_MIC1_IRQHandler(void)
{ 
  //HAL_DMA_IRQHandler(&hdma_dfsdmReg_FLT[0]);
  HAL_DMA_IRQHandler(&hDmaDfsdm);
}

/**
* @brief  This function handles External line 2 interrupt request.
* @param  None
* @retval None
*/
void EXTI3_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  SW_BV_send_Callback();

}

/**
  * @brief  EXTI9_5_IRQHandler This function handles External line
  *         interrupt request for BlueNRG.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
  //HAL_GPIO_EXTI_IRQHandler(BNRG_SPI_EXTI_PIN);
  HAL_GPIO_EXTI_IRQHandler(HCI_TL_SPI_EXTI_PIN);
}

/**
  * @brief  This function handles External line 2 interrupt request
  * @param  None
  * @retval None
  */
void EXTI2_IRQHandler( void )
{
  HAL_GPIO_EXTI_IRQHandler(BSP_LSM6DSM_INT2);
}

#ifdef ALLMEMS1_ENABLE_PRINTF
/**
  * @brief  This function handles USB-On-The-Go FS global interrupt request.
  * @param  None
  * @retval None
  */
void OTG_FS_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd);
}

/**
  * @brief  This function handles TIM interrupt request.
  * @param  None
  * @retval None
  */
void TIM8_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimHandle);
}
#endif /* ALLMEMS1_ENABLE_PRINTF */

