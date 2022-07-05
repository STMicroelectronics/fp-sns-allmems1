/**
  ******************************************************************************
  * @file    datalog_application.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   Header for datalog_application.c module.
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
#ifndef __DATALOG_APPLICATION_H
#define __DATALOG_APPLICATION_H

#ifdef __cplusplus
extern "C" {
#endif
  
#define MAX_TRIALS_OPENS_SD 10

extern void DATALOG_SD_Init(void);
extern void DATALOG_SD_DeInit(void);

extern uint8_t DATALOG_SD_Log_Enable(uint8_t AccessControl);
extern void DATALOG_SD_Log_Disable(void);

extern uint8_t DATALOG_SD_LogAudio_Enable(void);
extern void DATALOG_SD_LogAudio_Disable(void);

extern void writeAudio_on_sd(void);

void DATALOG_SD_NewLine(void);
void uSdWriteSpeedTest(char* myData); // AST

extern void saveData(char *myData, float *Quaternion, BSP_MOTION_SENSOR_Axes_t acc, BSP_MOTION_SENSOR_Axes_t gyro, BSP_MOTION_SENSOR_Axes_t magn, float press, float temp1, float temp2, float humi, uint16_t linesNum);





#ifdef __cplusplus
}
#endif

#endif /* __DATALOG_APPLICATION_H */


