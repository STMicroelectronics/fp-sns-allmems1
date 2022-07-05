 /**
  ******************************************************************************
  * @file    DataLog_Manager.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   Header for DataLog_Manager.c
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
#ifndef _DATA_LOG_MANAGER_H_
#define _DATA_LOG_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

//#define WRITE_EACH 8 //ORIGiNAL
#define WRITE_EACH 32
#define AUDIO_BUFF_SIZE (AUDIO_SAMPLING_FREQUENCY / 1000 * 1 * (WRITE_EACH*2))
  
#define ARRAYSIZE 10816//9152//8192

#define BUFF_SIZE 104
#define FREQUENCY BUFF_SIZE
  
#define CREATE_FILE_FOR_WRITE 0
#define OPEN_FILE_FOR_APP     1

/* Exported Functions Prototypes ---------------------------------------------*/
extern void AudioProcess_SD_Recording(void);
extern void openFileAudio(void);
extern void closeFileAudio(void);

extern void SD_CardLoggingMemsData(void);
extern void openFile(uint8_t AccessControl);
extern void closeFile(void);

extern void SD_CardLoggingMemsStart(void);
extern void SD_CardLoggingMemsStop(void);

#ifdef __cplusplus
}
#endif

#endif /* _DATA_LOG_MANAGER_H_ */



