/**
  ******************************************************************************
  * @file    hci_tl_interface_template.c
  * @author  SRA Application Team
  * @brief   Function implementation for the STM32 BlueNRG HCI Transport Layer 
  *          interface
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "hci_tl_interface.h"
 
void hci_tl_lowlevel_init(void)
{
  
  /* USER CODE BEGIN hci_tl_lowlevel_init 1 */
  /* Register IO bus services */
  
  /* USER CODE END hci_tl_lowlevel_init 1 */
   
  /* USER CODE BEGIN hci_tl_lowlevel_init 2 */
  /* Register event irq handler */
  
  /* USER CODE END hci_tl_lowlevel_init 2 */

  /* USER CODE BEGIN hci_tl_lowlevel_init 3 */
  
  /* USER CODE END hci_tl_lowlevel_init 3 */
}

void hci_tl_lowlevel_isr(void)
{
  /* USER CODE BEGIN hci_tl_lowlevel_isr */
  /* Call hci_notify_asynch_evt() */
  
  /* USER CODE END hci_tl_lowlevel_isr */
}
