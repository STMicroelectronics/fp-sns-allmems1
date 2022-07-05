/**
  ******************************************************************************
  * @file    datalog_application.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   This file provides a set of functions to handle the datalog
  *          application.
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
#include "datalog_application.h"
#include "sensor_service.h"
#include "DataLog_Manager.h"

#include <stdio.h>
#include <math.h>
    
/* FatFs includes component */
#include "ff_gen_drv.h"
#include "sd_diskio_SensorTile.h"

#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING

/* Private variables -------------------------------------------------------------*/
   
/* FatFs function common result code */
FRESULT res;

/* File write/read counts */
uint32_t byteswritten, bytesread;

/* File system object for SD card logical drive */
FATFS SDFatFs;

/* File object */
FIL MyFile;

/* SD card logical drive path */
char SDPath[4];

volatile uint8_t pHeader[169];
volatile uint8_t pAudioHeader[44];

/* Imported Variables -------------------------------------------------------------*/
extern uint16_t Audio_OUT_Buff[];
extern  volatile int  index_buff;
   
/* RTC handler declared in "main.c" file */
//extern RTC_HandleTypeDef RtcHandle;
extern RTC_DateTypeDef CurrentDate;
extern RTC_TimeTypeDef CurrentTime;

extern char MemsDataFileName[];

/* Exported Variables -------------------------------------------------------------*/
volatile uint8_t SD_Log_Enabled = 0;
volatile uint8_t SD_CardNotPresent = 0;

uint16_t sdcard_file_counter = 0;

char newLine[] = "\r\n";

/* Private function prototypes ---------------------------------------------------*/
static uint32_t csvProcess_HeaderInit(void);
static void floatToInt( float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec );

static uint32_t WavProcess_HeaderInit(void);
static uint32_t WavProcess_HeaderUpdate(uint32_t len);

static void CreateFileName(char *FileName, uint16_t Counter);


/**
  * @brief  Start SD-Card demo
  * @param  None
  * @retval None
  */
void DATALOG_SD_Init(void)
{
  //char SDPath[4];
    
  if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
  {
    /* Register the file system object to the FatFs module */
    if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK)
    {
      /* FatFs Initialization Error */
      while(1)
      {
        BSP_LED_On(LED1);
        HAL_Delay(500);
        BSP_LED_Off(LED1);
        HAL_Delay(100);
      }
    }
  }
}
  
/**
  * @brief  Start SD-Card demo
  * @param  None
  * @retval None
  */
uint8_t DATALOG_SD_Log_Enable(uint8_t AccessControl)
{
  //static uint16_t sdcard_file_counter = 0;
  
  static uint32_t delay,d=0;
  
  /* written byte count */
  uint32_t byteswritten;
  
  static char file_name[40] = {0};
  
  csvProcess_HeaderInit();
  
  /* SD SPI CS Config */
  SD_IO_CS_Init();
  
  switch(AccessControl)
  {
		int i;
  /*Create the file */
  case CREATE_FILE_FOR_WRITE:
    
    CreateFileName(file_name,sdcard_file_counter);
    
    for(i=0; i<40; i++)
      MemsDataFileName[i+1]= file_name[i];
    
    sdcard_file_counter++;

    delay = HAL_GetTick();  
    d=HAL_GetTick();
    do
    {
      d=HAL_GetTick();
    }while(d - delay < 100 );
    //HAL_Delay(100);
    
    if(f_open(&MyFile, (char const*)file_name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
    {
      if(sdcard_file_counter > MAX_TRIALS_OPENS_SD)
      {
        sdcard_file_counter = 0;
        SD_CardNotPresent= 1;
      }
      
      return 0;
    }
    
    MDM_SaveGMD(GMD_MEMS_DATA_FILENAME,(void *)&MemsDataFileName);
    NecessityToSaveMetaDataManager=1;
    
    //BytesToWrite =sprintf((char *)BufferToWrite,"Open File OK\r\n");
    //Term_Update(BufferToWrite,BytesToWrite);
    //HAL_Delay(100);

    if(f_write(&MyFile, (const void*)pHeader, sizeof(pHeader), (void *)&byteswritten) != FR_OK)
    {
      //BytesToWrite =sprintf((char *)BufferToWrite,"Write Failed\r\n");
      //Term_Update(BufferToWrite,BytesToWrite);
      return 0;
    }
    break;
    
  /* Open the file to append data */  
  case OPEN_FILE_FOR_APP:
    
    for(int i=0; i<40; i++)
     file_name[i]=  MemsDataFileName[i+1];
    
    if(f_open(&MyFile, (char const*)file_name, FA_OPEN_ALWAYS | FA_WRITE) != FR_OK)
    {
      BytesToWrite =sprintf((char *)BufferToWrite,"Open Failed\r\n");
      Term_Update(BufferToWrite,BytesToWrite);
      
      return 0;
    }
    
    /* Move to end of the file to append data */
    f_lseek(&MyFile, f_size(&MyFile));
    break;
  }
  
  return 1;
}

/**
  * @brief  Start SD-Card demo
  * @param  None
  * @retval None
  */
uint8_t DATALOG_SD_LogAudio_Enable(void)
{
  static uint16_t SdCard_FileAudio_Counter = 0;
  
  /* written byte count */
  uint32_t byteswritten; 
  
  char file_name[30] = {0};
  static uint32_t delay,d=0;
  WavProcess_HeaderInit();

  /* SD SPI CS Config */
  SD_IO_CS_Init();
  
  /* SensorTile Audio Logging File Name */
  sprintf(file_name, "%s%.3d%s", "ST_AudioLog_N", SdCard_FileAudio_Counter, ".wav");
  
  SdCard_FileAudio_Counter++;
  
  delay = HAL_GetTick();  
  d=HAL_GetTick();
  do
  {
    d=HAL_GetTick();
  }while(d - delay < 100 );
  //HAL_Delay(100);

  if(f_open(&MyFile, (char const*)file_name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
  {
    if(SdCard_FileAudio_Counter > MAX_TRIALS_OPENS_SD)
    {
      SdCard_FileAudio_Counter = 0;
      SD_CardNotPresent= 1;
    }
    
    return 0;
  }
 
  if(f_write(&MyFile, (uint8_t*) pAudioHeader, sizeof(pAudioHeader), (void *)&byteswritten) != FR_OK)
  {
    return 0;
  }
  
  return 1;
}

/**
  * @brief  .
  * @param  None
  * @retval None
  */
void DATALOG_SD_DeInit(void)
{
  FATFS_UnLinkDriver(SDPath);
}

/**
  * @brief  Disable SDCard Log
  * @param  None
  * @retval None
  */
void DATALOG_SD_Log_Disable(void)
{
  f_close(&MyFile);
  
  /* SD SPI Config */
  SD_IO_CS_DeInit();
}

/**
  * @brief  Disable SDCard Log
  * @param  None
  * @retval None
  */
void DATALOG_SD_LogAudio_Disable(void)
{
  uint32_t len;
  uint32_t byteswritten;                     /* File write/read counts */

  
  len = f_size(&MyFile);
  WavProcess_HeaderUpdate(len);

    /* Update the data length in the header of the recorded Wave */    
  f_lseek(&MyFile, 0);
  
  /* Parse the wav file header and extract required information */
  
  f_write(&MyFile, (uint8_t*)pAudioHeader,  sizeof(pAudioHeader), (void*)&byteswritten);
  
  /* Close file and unmount MyFilesystem */
    //f_sync(&MyFile);
  f_close(&MyFile);
  
  /* SD SPI Config */
  SD_IO_CS_DeInit();
}

/**
* @brief Highest priority interrupt handler routine
* @param None
* @retval None
*/
void writeAudio_on_sd(void)
{ 
  volatile FRESULT s;
  uint32_t byteswritten; /* File write/read counts */
  
  s=f_write(&MyFile,  &(((uint8_t *)Audio_OUT_Buff)[index_buff]), AUDIO_BUFF_SIZE, (void *)&byteswritten);
  if(s != FR_OK)
  {
    BSP_LED_On(LED1);
    while(1);
  }
}

/**
  * @brief  Write New Line to file
  * @param  None
  * @retval None
  */
void DATALOG_SD_NewLine(void)
{
  uint32_t byteswritten; /* written byte count */
  f_write(&MyFile, (const void*)&newLine, 2, (void *)&byteswritten);
}

void saveData(char *myData, float *Quaternion, BSP_MOTION_SENSOR_Axes_t acc, BSP_MOTION_SENSOR_Axes_t gyro, BSP_MOTION_SENSOR_Axes_t magn, float press, float temp1, float temp2, float humi, uint16_t linesNum)
{
  uint8_t v_mode;
  
  uint32_t soc= 0;
  
  uint32_t size = 0;
  
  int32_t qi1, qi2;
  int32_t qj1, qj2;
  int32_t qk1, qk2;
  int32_t qs1, qs2;
  
  char SignQuat[4];
  
  int32_t p1, p2;
  int32_t t11, t12;
  int32_t t21, t22;
  int32_t h1, h2;

  /* Update Gas Gouge Status */
  BSP_GG_Task(TargetBoardFeatures.HandleGGComponent,&v_mode);
  BSP_GG_GetSOC(TargetBoardFeatures.HandleGGComponent, &soc);
  
  /* Write data to the file on the SDCard */
  if(SD_Log_Enabled) 
  {
    myData[0] = '\0';
    
    if(Quaternion[3] < 0)
    {
      Quaternion[0]= (-1) * Quaternion[0];
      Quaternion[1]= (-1) * Quaternion[0];
      Quaternion[2]= (-1) * Quaternion[0];
    }
    
    for(int i=0;i<4;i++)
    {
      if(Quaternion[i] >= 0)
        SignQuat[i]=' ';
      else
        SignQuat[i]='-';
    }
    
    floatToInt( Quaternion[0], &qi1, &qi2, 3 );
    floatToInt( Quaternion[1], &qj1, &qj2, 3 );
    floatToInt( Quaternion[2], &qk1, &qk2, 3 );
    floatToInt( Quaternion[3], &qs1, &qs2, 3 );
    
    floatToInt( press, &p1, &p2, 2 );
    floatToInt( temp1, &t11, &t12, 2 );
    floatToInt( temp2, &t21, &t22, 2 );
    floatToInt( humi, &h1, &h2, 2 );
    size = size + sprintf(myData + strlen(myData), "%02d:%02d:%02d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d.%02d, %d.%02d, %d.%02d, %d.%02d, %c%d.%03d, %c%d.%03d, %c%d.%03d, %c%d.%03d, %d\n",
                          CurrentTime.Hours,
                          CurrentTime.Minutes,
                          CurrentTime.Seconds,
                          (int)acc.x,
                          (int)acc.y,
                          (int)acc.z,
                          (int)gyro.x,
                          (int)gyro.y,
                          (int)gyro.z,
                          (int)magn.x,
                          (int)magn.y,
                          (int)magn.z,
                          (int)p1,
                          (int)p2,
                          (int)t11,
                          (int)t12,
                          (int)t21,
                          (int)t22,
                          (int)h1,
                          (int)h2,
                          SignQuat[0],
                          (int)qi1,
                          (int)qi2,
                          SignQuat[1],
                          (int)qj1,
                          (int)qj2,
                          SignQuat[2],
                          (int)qk1,
                          (int)qk2,
                          SignQuat[3],
                          (int)qs1,
                          (int)qs2,
                          (int)soc);
    
    //size = size + sprintf(myData + strlen(myData), "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d.%02d, %d.%02d, %d.%02d\n", (int)msTicks, (int)acc.x, (int)acc.y, (int)acc.z, (int)gyro.x, (int)gyro.y, (int)gyro.z, (int)magn.x, (int)magn.y, (int)magn.z, (int)p1, (int)p2, (int)t1, (int)t2, (int)h1, (int)h2); 
   
    res = f_write(&MyFile, myData, size, (void *)&byteswritten);
  }  
}

/**
* @brief  write on uSD some data
* @param  None
* @retval None
*/
void uSdWriteSpeedTest(char* myData)
{
  /* Write data to the file on the SDCard */
  if(SD_Log_Enabled) 
  {
    uint32_t size; 
    size = 8192; 
    res = f_write(&MyFile, myData, size, (void *)&byteswritten);
  }
}

/**
  * @brief  Initialize the wave header file
  * @param  pHeader: Header Buffer to be filled
  * @param  pWaveFormatStruct: Pointer to the wave structure to be filled.
  * @retval 0 if passed, !0 if failed.
  */
static uint32_t csvProcess_HeaderInit(void)
{
  pHeader[0]=  'h';
  pHeader[1]=  'h';
  pHeader[2]=  ':';
  pHeader[3]=  'm';
  pHeader[4]=  'm';
  pHeader[5]=  ':';
  pHeader[6]=  's';
  pHeader[7]=  's';
  pHeader[8]=  ' ';
  pHeader[9]=  ',';
  
  pHeader[10]= 'A';
  pHeader[11]= 'c';
  pHeader[12]= 'c';
  pHeader[13]= 'X';
  pHeader[14]= ' ';
  pHeader[15]= '[';
  pHeader[16]= 'm';
  pHeader[17]= 'g';
  pHeader[18]= ']';
  pHeader[19]= ',';
  
  pHeader[20]= 'A';
  pHeader[21]= 'c';
  pHeader[22]= 'c';
  pHeader[23]= 'Y';
  pHeader[24]= ' ';
  pHeader[25]= '[';
  pHeader[26]= 'm';
  pHeader[27]= 'g';
  pHeader[28]= ']';
  pHeader[29]= ',';
  
  pHeader[30]= 'A';
  pHeader[31]= 'c';
  pHeader[32]= 'c';
  pHeader[33]= 'Z';
  pHeader[34]= ' ';
  pHeader[35]= '[';
  pHeader[36]= 'm';
  pHeader[37]= 'g';
  pHeader[38]= ']';
  pHeader[39]= ',';
  
  pHeader[40]= 'G';
  pHeader[41]= 'y';
  pHeader[42]= 'r';
  pHeader[43]= 'o';
  pHeader[44]= 'X';
  pHeader[45]= ' ';
  pHeader[46]= '[';
  pHeader[47]= 'm';
  pHeader[48]= 'd';
  pHeader[49]= 'p';
  pHeader[50]= 's';
  pHeader[51]= ']';
  pHeader[52]= ',';
  
  pHeader[53]= 'G';
  pHeader[54]= 'y';
  pHeader[55]= 'r';
  pHeader[56]= 'o';
  pHeader[57]= 'Y';
  pHeader[58]= ' ';
  pHeader[59]= '[';
  pHeader[60]= 'm';
  pHeader[61]= 'd';
  pHeader[62]= 'p';
  pHeader[63]= 's';
  pHeader[64]= ']';
  pHeader[65]= ',';

  pHeader[66]= 'G';
  pHeader[67]= 'y';
  pHeader[68]= 'r';
  pHeader[69]= 'o';
  pHeader[70]= 'Z';
  pHeader[71]= ' ';
  pHeader[72]= '[';
  pHeader[73]= 'm';
  pHeader[74]= 'd';
  pHeader[75]= 'p';
  pHeader[76]= 's';
  pHeader[77]= ']';
  pHeader[78]= ',';
  
  pHeader[79]= 'M';
  pHeader[80]= 'a';
  pHeader[81]= 'g';
  pHeader[82]= 'X';
  pHeader[83]= ' ';
  pHeader[84]= '[';
  pHeader[85]= 'm';
  pHeader[86]= 'g';
  pHeader[87]= 'a';
  pHeader[88]= 'u';
  pHeader[89]= 's';
  pHeader[90]= 's';
  pHeader[91]= ']';
  pHeader[92]= ',';
  
  pHeader[93]= 'M';
  pHeader[94]= 'a';
  pHeader[95]= 'g';
  pHeader[96]= 'Y';
  pHeader[97]= ' ';
  pHeader[98]= '[';
  pHeader[99]= 'm';
  pHeader[100]= 'g';
  pHeader[101]= 'a';
  pHeader[102]= 'u';
  pHeader[103]= 's';
  pHeader[104]= 's';
  pHeader[105]= ']';
  pHeader[106]= ',';
  
  pHeader[107]= 'M';
  pHeader[108]= 'a';
  pHeader[109]= 'g';
  pHeader[110]= 'Z';
  pHeader[111]= ' ';
  pHeader[112]= '[';
  pHeader[113]= 'm';
  pHeader[114]= 'g';
  pHeader[115]= 'a';
  pHeader[116]= 'u';
  pHeader[117]= 's';
  pHeader[118]= 's';
  pHeader[119]= ']';
  pHeader[120]= ',';
  
  pHeader[121]= 'P';
  pHeader[122]= ' ';
  pHeader[123]= '[';
  pHeader[124]= 'm';
  pHeader[125]= 'B';
  pHeader[126]= ']';
  pHeader[127]= ',';
  
  pHeader[128]= 'T';
  pHeader[129]= '1';
  pHeader[130]= ' ';
  pHeader[131]= '[';
  pHeader[132]= '°';
  pHeader[133]= 'C';
  pHeader[134]= ']';
  pHeader[135]= ',';
  
  pHeader[136]= 'T';
  pHeader[137]= '2';
  pHeader[138]= ' ';
  pHeader[139]= '[';
  pHeader[140]= '°';
  pHeader[141]= 'C';
  pHeader[142]= ']';
  pHeader[143]= ',';
  
  pHeader[144]= 'H';
  pHeader[145]= ' ';
  pHeader[146]= '[';
  pHeader[147]= '%';
  pHeader[148]= ']';
  pHeader[149]= ',';
  
  pHeader[150]= 'q';
  pHeader[151]= 'i';
  pHeader[152]= ',';
  pHeader[153]= 'q';
  pHeader[154]= 'j';
  pHeader[155]= ',';
  pHeader[156]= 'q';
  pHeader[157]= 'k';
  pHeader[158]= ',';
  pHeader[159]= 'q';
  pHeader[160]= 's';
  pHeader[161]= ',';
  
  pHeader[162]= 'B';
  pHeader[163]= 'a';
  pHeader[164]= 't';
  pHeader[165]= 't';
  pHeader[166]= '%';
  
  pHeader[167]= '\r';
  pHeader[168]= '\n';
  
  /* Return 0 if all operations are OK */
  return 0;
}

/**
  * @brief  Initialize the wave header file
  * @param  pHeader: Header Buffer to be filled
  * @param  pWaveFormatStruct: Pointer to the wave structure to be filled.
  * @retval 0 if passed, !0 if failed.
  */
static uint32_t WavProcess_HeaderInit(void)
{
  uint16_t   BitPerSample=16;
  uint16_t   NbrChannels=1;//AUDIO_CHANNELS;
  uint32_t   ByteRate=AUDIO_SAMPLING_FREQUENCY*(BitPerSample/8);
   
  uint32_t   SampleRate=AUDIO_SAMPLING_FREQUENCY;  
  uint16_t   BlockAlign= NbrChannels * (BitPerSample/8);
  
  /* Write chunkID, must be 'RIFF'  ------------------------------------------*/
  pAudioHeader[0] = 'R';
  pAudioHeader[1] = 'I';
  pAudioHeader[2] = 'F';
  pAudioHeader[3] = 'F';
  
  /* Write the file length ----------------------------------------------------*/
  /* The sampling time: this value will be be written back at the end of the 
     recording opearation.  Example: 661500 Btyes = 0x000A17FC, byte[7]=0x00, byte[4]=0xFC */
  pAudioHeader[4] = 0x00;
  pAudioHeader[5] = 0x4C;
  pAudioHeader[6] = 0x1D;
  pAudioHeader[7] = 0x00;
  
  /* Write the file format, must be 'WAVE' -----------------------------------*/
  pAudioHeader[8]  = 'W';
  pAudioHeader[9]  = 'A';
  pAudioHeader[10] = 'V';
  pAudioHeader[11] = 'E';
  
  /* Write the format chunk, must be'fmt ' -----------------------------------*/
  pAudioHeader[12]  = 'f';
  pAudioHeader[13]  = 'm';
  pAudioHeader[14]  = 't';
  pAudioHeader[15]  = ' ';
  
  /* Write the length of the 'fmt' data, must be 0x10 ------------------------*/
  pAudioHeader[16]  = 0x10;
  pAudioHeader[17]  = 0x00;
  pAudioHeader[18]  = 0x00;
  pAudioHeader[19]  = 0x00;
  
  /* Write the audio format, must be 0x01 (PCM) ------------------------------*/
  pAudioHeader[20]  = 0x01;
  pAudioHeader[21]  = 0x00;
  
  /* Write the number of channels, ie. 0x01 (Mono) ---------------------------*/
  pAudioHeader[22]  = NbrChannels;
  pAudioHeader[23]  = 0x00;
  
  /* Write the Sample Rate in Hz ---------------------------------------------*/
  /* Write Little Endian ie. 8000 = 0x00001F40 => byte[24]=0x40, byte[27]=0x00*/
  pAudioHeader[24]  = (uint8_t)((SampleRate & 0xFF));
  pAudioHeader[25]  = (uint8_t)((SampleRate >> 8) & 0xFF);
  pAudioHeader[26]  = (uint8_t)((SampleRate >> 16) & 0xFF);
  pAudioHeader[27]  = (uint8_t)((SampleRate >> 24) & 0xFF);
  
  /* Write the Byte Rate -----------------------------------------------------*/
  pAudioHeader[28]  = (uint8_t)(( ByteRate & 0xFF));                   
  pAudioHeader[29]  = (uint8_t)(( ByteRate >> 8) & 0xFF);
  pAudioHeader[30]  = (uint8_t)(( ByteRate >> 16) & 0xFF);
  pAudioHeader[31]  = (uint8_t)(( ByteRate >> 24) & 0xFF);
  
  /* Write the block alignment -----------------------------------------------*/
  pAudioHeader[32]  = BlockAlign;
  pAudioHeader[33]  = 0x00;
  
  /* Write the number of bits per sample -------------------------------------*/
  pAudioHeader[34]  = BitPerSample;
  pAudioHeader[35]  = 0x00;
  
  /* Write the Data chunk, must be 'data' ------------------------------------*/
  pAudioHeader[36]  = 'd';
  pAudioHeader[37]  = 'a';
  pAudioHeader[38]  = 't';
  pAudioHeader[39]  = 'a';
  
  /* Write the number of sample data -----------------------------------------*/
  /* This variable will be written back at the end of the recording operation */
  pAudioHeader[40]  = 0x00;
  pAudioHeader[41]  = 0x4C;
  pAudioHeader[42]  = 0x1D;
  pAudioHeader[43]  = 0x00;
  
  /* Return 0 if all operations are OK */
  return 0;
}

/**
  * @brief  Initialize the wave header file
  * @param  pHeader: Header Buffer to be filled
  * @param  pWaveFormatStruct: Pointer to the wave structure to be filled.
  * @retval 0 if passed, !0 if failed.
  */
static uint32_t WavProcess_HeaderUpdate(uint32_t len)
{
  /* Write the file length ----------------------------------------------------*/
  /* The sampling time: this value will be be written back at the end of the 
     recording opearation.  Example: 661500 Btyes = 0x000A17FC, byte[7]=0x00, byte[4]=0xFC */
  pAudioHeader[4] = (uint8_t)(len);
  pAudioHeader[5] = (uint8_t)(len >> 8);
  pAudioHeader[6] = (uint8_t)(len >> 16);
  pAudioHeader[7] = (uint8_t)(len >> 24);
  /* Write the number of sample data -----------------------------------------*/
  /* This variable will be written back at the end of the recording operation */
  len -=44;
  pAudioHeader[40] = (uint8_t)(len); 
  pAudioHeader[41] = (uint8_t)(len >> 8);
  pAudioHeader[42] = (uint8_t)(len >> 16);
  pAudioHeader[43] = (uint8_t)(len >> 24); 
  /* Return 0 if all operations are OK */
  return 0;
}

/**
* @brief  Splits a float into two integer values.
* @param  in the float value as input
* @param  out_int the pointer to the integer part as output
* @param  out_dec the pointer to the decimal part as output
* @param  dec_prec the decimal precision to be used
* @retval None
*/
static void floatToInt( float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec )
{
  *out_int = (int32_t)in;
  
  if(in >= 0.0f)
  {
    in = in - (float)(*out_int);
  }
  else
  {
    in = (float)(*out_int) - in;
  }
  *out_dec = (int32_t)trunc(in * pow(10, dec_prec));
}

/**
* @brief  Create file name for mems data loggimg.
* @param  FileName Name of the create file
* @retval None
*/
static void CreateFileName(char *FileName, uint16_t Counter)
{
  uint8_t Month;
  
  Month= RTC_ByteToBcd2(CurrentDate.Month);
  
  switch(Month)
  {
  case RTC_MONTH_JANUARY:
    sprintf(FileName, "MemsData_%02d_January_%02d_%.3d.csv", CurrentDate.Date, CurrentDate.Year, Counter);
    break;
  case RTC_MONTH_FEBRUARY:
    sprintf(FileName, "MemsData_%02d_February_%02d_%.3d.csv", CurrentDate.Date, CurrentDate.Year, Counter);
    break;
  case RTC_MONTH_MARCH:
    sprintf(FileName, "MemsData_%02d_March_%02d_%.3d.csv", CurrentDate.Date, CurrentDate.Year, Counter);
    break;
  case RTC_MONTH_APRIL:
    sprintf(FileName, "MemsData_%02d_April_%02d_%.3d.csv", CurrentDate.Date, CurrentDate.Year, Counter);
    break;
  case RTC_MONTH_MAY:
    sprintf(FileName, "MemsData_%02d_May_%02d_%.3d.csv", CurrentDate.Date, CurrentDate.Year, Counter);
    break;
  case RTC_MONTH_JUNE:
    sprintf(FileName, "MemsData_%02d_June_%02d_%.3d.csv", CurrentDate.Date, CurrentDate.Year, Counter);
    break;
  case RTC_MONTH_JULY:
    sprintf(FileName, "MemsData_%02d_July_%02d_%.3d.csv", CurrentDate.Date, CurrentDate.Year, Counter);
    break;
  case RTC_MONTH_AUGUST:
    sprintf(FileName, "MemsData_%02d_August_%02d_%.3d.csv", CurrentDate.Date, CurrentDate.Year, Counter);
    break;
  case RTC_MONTH_SEPTEMBER:
    sprintf(FileName, "MemsData_%02d_September_%02d_%.3d.csv", CurrentDate.Date, CurrentDate.Year, Counter);
    break;
  case RTC_MONTH_OCTOBER:
    sprintf(FileName, "MemsData_%02d_October_%02d_%.3d.csv", CurrentDate.Date, CurrentDate.Year, Counter);
    break;
  case RTC_MONTH_NOVEMBER:
    sprintf(FileName, "MemsData_%02d_November_%02d_%.3d.csv", CurrentDate.Date, CurrentDate.Year, Counter);
    break;
  case RTC_MONTH_DECEMBER:
    sprintf(FileName, "MemsData_%02d_December_%02d_%.3d.csv", CurrentDate.Date, CurrentDate.Year, Counter);
    break;
  }
}

#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */

