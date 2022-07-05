/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include <math.h>
#include <limits.h>
#include "TargetFeatures.h"
#include "OTA.h"
#include "MetaDataManager.h"
#include "bluenrg_utils.h"
#include "HWAdvanceFeatures.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define N_BUTTON_PRESS 3
#define CHECK_CALIBRATION ((uint32_t)0x12345678)

/* Imported Variables -------------------------------------------------------------*/
extern volatile float RMS_Ch[];
extern float DBNOISE_Value_Old_Ch[];
extern uint16_t PCM_Buffer[];
extern uint16_t PDM_Buffer[];
extern uint32_t NumSample;

/* Exported Variables -------------------------------------------------------------*/

float sensitivity;
/* Acc sensitivity multiply by FROM_MG_TO_G constant */
float sensitivity_Mul;

MFX_MagCal_output_t magOffset;
MOTION_SENSOR_Axes_t MAG_Offset;

uint32_t ForceReCalibration    =0;

uint8_t bdaddr[6];

uint8_t EnvironmentalTimerEnabled= 0;
uint8_t InertialTimerEnabled= 0;
uint8_t AccEventEnabled= 0;
uint8_t AudioLevelTimerEnabled= 0;

uint8_t ActivityRecognitionEnabled= 0;
uint8_t CarryPositionEnabled= 0;
uint8_t FitnessActivitiesEnabled= 0;
uint8_t GestureRecognitionEnabled= 0;
uint8_t MotionIntensityEnabled= 0;
uint8_t SensorFusionEnabled= 0;
uint8_t ECompassEnabled= 0;
uint8_t PoseEstimationEnabled= 0;
uint8_t Standing_VS_SittingDeskEnabled= 0;
uint8_t TiltSensingEnabled= 0;
uint8_t VerticalContextEnabled= 0;
uint8_t AudioSourceLocalizationTimerEnabled= 0;

uint8_t TIM1_CHANNEL_1_Enabled= 0;
uint8_t TIM1_CHANNEL_2_Enabled= 0;
uint8_t TIM1_CHANNEL_3_Enabled= 0;
uint8_t TIM1_CHANNEL_4_Enabled= 0;

uint32_t uhCCR1_Val = DEFAULT_uhCCR1_Val;
uint32_t uhCCR2_Val = DEFAULT_uhCCR2_Val;
uint32_t uhCCR3_Val = DEFAULT_uhCCR3_Val;
uint32_t uhCCR4_Val = DEFAULT_uhCCR4_Val;

uint8_t  NodeName[8];

volatile uint32_t HCI_ProcessEvent=0;

unsigned char AccelleroCalibrationDone = 0;
unsigned char MagnetoCalibrationDone = 0;

uint16_t PedometerStepCount= 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

DFSDM_Channel_HandleTypeDef hdfsdm1_channel0;

SAI_HandleTypeDef hsai_BlockA2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t MagCalibrationData[6];
uint32_t AccCalibrationData[8];

/* Table with All the known Meta Data */
MDM_knownGMD_t known_MetaData[]={
  {GMD_MAG_CALIBRATION,(sizeof(MagCalibrationData))},
  {GMD_ACC_CALIBRATION,(sizeof(AccCalibrationData))},
  {GMD_NODE_NAME,      (sizeof(NodeName))},
  {GMD_END    ,0}/* THIS MUST BE THE LAST ONE */
};

static volatile int ButtonPressed       = 0;
static volatile int MEMSInterrupt       = 0;
static volatile uint32_t SendEnv        = 0;
static volatile uint32_t SendInertial	= 0;
static volatile uint32_t SendAudioLevel  =0;

static volatile uint32_t TimeStamp		= 0;

static float UsedGyroscopeDataRate;
static float UsedAccelerometerDataRate;
static float UsedPressureDataRate;

static volatile uint32_t Quaternion      = 0;
static volatile uint32_t UpdateMotionAR  = 0;

#ifdef ALLMEMS1_MOTIONCP
static volatile uint32_t UpdateMotionCP  =0;
#endif /*  ALLMEMS1_MOTIONCP */

#ifdef ALLMEMS1_MOTIONFA
static volatile uint32_t UpdateMotionFA  =0;
#endif /* ALLMEMS1_MOTIONFA */

static volatile uint32_t UpdateMotionGR  = 0;
static volatile uint32_t UpdateMotionID  = 0;

/* Code for MotionPE integration - Start Section */
static volatile uint32_t UpdateMotionPE  =0;
/* Code for MotionPE integration - End Section */

/* Code for MotionSD integration - Start Section */
static volatile uint32_t UpdateMotionSD  =0;
/* Code for MotionSD integration - End Section */

/* Code for MotionTL integration - Start Section */
static volatile uint32_t UpdateMotionTL  =0;
MTL_acc_cal_t AccelerometerCalibrationValue;
/* Code for MotionTL integration - End Section */

/* Code for MotionVC integration - Start Section */
static volatile uint32_t UpdateMotionVC  =0;
/* Code for MotionVC integration - End Section */

static volatile uint32_t SendAudioSourceLocalization=0;

extern volatile int32_t SourceLocationToSend;

static uint32_t mag_time_stamp = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM1_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_SAI2_Init(void);
/* USER CODE BEGIN PFP */
static void Set2GAccelerometerFullScale(void);
static void Set4GAccelerometerFullScale(void);
static void InitMotionLibraries(void);
static void EnableDisableFeatures(void);
  
static unsigned char ResetCalibrationInMemory(void);
static unsigned char ReCallNodeNameFromMemory(void);

static void SendEnvironmentalData(void);
static void MEMSCallback(void);

static void AccCalibTest(void);

static void MagCalibTest(void);
static void ReCalibration(uint8_t ReCalibrationDev);

static void SendMotionData(void);

static void SendAudioLevelData(void);

static void ButtonCallback(void);

static void Inertial_StartStopTimer(void);
static void Environmental_StartStopTimer(void);
static void AccEnv_StartStop(void);
static void AudioLevel_StartStopTimer(void);
static void AudioSourceLocalization_StartStopTimer(void);

static void TIM1_CHANNEL_1_StartStop(void);
static void TIM1_CHANNEL_2_StartStop(void);
static void TIM1_CHANNEL_3_StartStop(void);
static void TIM1_CHANNEL_4_StartStop(void);

void AudioProcess_DB_Noise(void);

static void ComputeMotionAR(void);

#ifdef ALLMEMS1_MOTIONCP
static void ComputeMotionCP(void);
#endif /*  ALLMEMS1_MOTIONCP */

#ifdef ALLMEMS1_MOTIONFA
static void ComputeMotionFA(void);
#endif /* ALLMEMS1_MOTIONFA */

static void ComputeQuaternions(void);
static void ComputeMotionGR(void);
static void ComputeMotionID(void);

/* Code for MotionPE integration - Start Section */
static void ComputeMotionPE(void);
/* Code for MotionPE integration - End Section */

/* Code for MotionSD integration - Start Section */
static void ComputeMotionSD(void);
/* Code for MotionSD integration - End Section */

/* Code for MotionTL integration - Start Section */
static void ComputeMotionTL(void);
/* Code for MotionTL integration - End Section */

/* Code for MotionVC integration - Start Section */
static void ComputeMotionVC(void);
/* Code for MotionVC integration - End Section */

static void SendAudioSourceLocalizationData(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_CRC_Init();
  MX_TIM1_Init();
  MX_DFSDM1_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_SAI2_Init();
  /* USER CODE BEGIN 2 */
  HAL_SAI_DeInit(&hsai_BlockA2);
  HAL_DFSDM_ChannelDeInit(&hdfsdm1_channel0);
  
  InitTargetPlatform();
  
  /* Check the MetaDataManager */
  InitMetaDataManager((void *)&known_MetaData,MDM_DATA_TYPE_GMD,NULL);
  
  ALLMEMS1_PRINTF("\n\t(HAL %ld.%ld.%ld_%ld)\r\n"
        "\tCompiled %s %s"

#if defined (__IAR_SYSTEMS_ICC__)
        " (IAR)\r\n"
#elif defined (__CC_ARM)
        " (KEIL)\r\n"
#elif defined (__GNUC__)
        " (STM32CubeIDE)\r\n"
#endif
         "\tSend Every %4dmS %d Short precision Quaternions\r\n"
         "\tSend Every %4dmS Temperature/Humidity/Pressure\r\n"
         "\tSend Every %4dmS Acc/Gyro/Magneto\r\n"
         "\tSend Every %4dmS dB noise\r\n\n",
           HAL_GetHalVersion() >>24,
          (HAL_GetHalVersion() >>16)&0xFF,
          (HAL_GetHalVersion() >> 8)&0xFF,
           HAL_GetHalVersion()      &0xFF,
         __DATE__,__TIME__,
         QUAT_UPDATE_MUL_10MS*10,SEND_N_QUATERNIONS,
         ALGO_PERIOD_ENV,
         ALGO_PERIOD_INERTIAL,
         ALGO_PERIOD_AUDIO_LEVEL);

#ifdef ALLMEMS1_DEBUG_CONNECTION
  ALLMEMS1_PRINTF("Debug Connection         Enabled\r\n");
#endif /* ALLMEMS1_DEBUG_CONNECTION */

#ifdef ALLMEMS1_DEBUG_NOTIFY_TRAMISSION
  ALLMEMS1_PRINTF("Debug Notify Trasmission Enabled\r\n\n");
#endif /* ALLMEMS1_DEBUG_NOTIFY_TRAMISSION */

  /* Set Node Name */
  ReCallNodeNameFromMemory();
  
  /* Initialize the BlueNRG stack and services */
  BluetoothInit();
  
  /* Check the BootLoader Compliance */
  ALLMEMS1_PRINTF("\r\n");
  if(CheckBootLoaderCompliance()) {
    ALLMEMS1_PRINTF("BootLoader Compliant with FOTA procedure\r\n\n");
  } else {
    ALLMEMS1_PRINTF("ERROR: BootLoader NOT Compliant with FOTA procedure\r\n\n");
  }

  /* Set Accelerometer Full Scale to 2G */
  Set2GAccelerometerFullScale();

  /* Read the Acc Sensitivity */
  MOTION_SENSOR_GetSensitivity(ACCELERO_INSTANCE,MOTION_ACCELERO,&sensitivity);
  sensitivity_Mul = sensitivity * ((float) FROM_MG_TO_G);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Led Blinking when there is not a client connected */
    if(!connected)
    {
      if(!TargetBoardFeatures.LedStatus) {
        if(!(HAL_GetTick()&0x3FF)) {
          LedOnTargetPlatform();
        }
      } else {
        if(!(HAL_GetTick()&0x3F)) {
          LedOffTargetPlatform();
        }
      }
    }

    if(set_connectable){
      InitMotionLibraries();

      if(NecessityToSaveMetaDataManager) {
        uint32_t Success = EraseMetaDataManager();
        if(Success) {
          SaveMetaDataManager();
        }
      }

      /* Now update the BLE advertize data and make the Board connectable */
      setConnectable();
      set_connectable = FALSE;
    }
    
    /* Enable/Disable BLE features and related timer */
    EnableDisableFeatures();

    /* Handle user button */
    if(ButtonPressed) {
      ButtonCallback();
      ButtonPressed=0;       
    }
    
    /* Handle Interrupt from MEMS */
    if(MEMSInterrupt) {
      MEMSCallback();
      MEMSInterrupt=0;
    }

    /* Handle Re-Calibration */
    if(ForceReCalibration > 0) {
      ReCalibration((uint8_t) ForceReCalibration);
      ForceReCalibration=0;
    }

    /* handle BLE event */
    if(HCI_ProcessEvent) {
      HCI_ProcessEvent=0;
      hci_user_evt_proc();
    }

    /* Environmental Data */
    if(SendEnv) {
      SendEnv=0;
      SendEnvironmentalData();
    }
    
    /* Mic Data */
    if (SendAudioLevel) {
      SendAudioLevel = 0;
      SendAudioLevelData();
    }

    /* Motion Data */
    if(SendInertial) {
      SendInertial=0;
      SendMotionData();
    }

    /* Code for MotionFX integration - Start Section */
    if(Quaternion) {
      Quaternion=0;
      ComputeQuaternions();
    }
    /* Code for MotionFX integration - End Section */

    /* Code for MotionAR integration - Start Section */
    if(UpdateMotionAR) {
      UpdateMotionAR=0;
      ComputeMotionAR();
    }
    /* Code for MotionAR integration - End Section */
    
    #ifdef ALLMEMS1_MOTIONCP
    /* MotionCP */
    if(UpdateMotionCP) {
      UpdateMotionCP=0;
      ComputeMotionCP();
    }
    #endif /*  ALLMEMS1_MOTIONCP */
    
    #ifdef ALLMEMS1_MOTIONFA
    if(UpdateMotionFA) {
      UpdateMotionFA=0;
      ComputeMotionFA();
    }
    #endif /* ALLMEMS1_MOTIONFA */

    /* MotionGR */
    if(UpdateMotionGR) {
      UpdateMotionGR=0;
      ComputeMotionGR();
    }
    
    /* MotionID */
    if(UpdateMotionID) {
      UpdateMotionID=0;
      ComputeMotionID();
    }
    
    /* Code for MotionPE integration - Start Section */
    if(UpdateMotionPE) {
      UpdateMotionPE=0;
      ComputeMotionPE();
    }
    /* Code for MotionPE integration - End Section */
    
    /* Code for MotionSD integration - Start Section */
    if(UpdateMotionSD) {
      UpdateMotionSD=0;
      ComputeMotionSD();
    }
    /* Code for MotionSD integration - End Section */
    
    /* Code for MotionTL integration - Start Section */
    if(UpdateMotionTL) {
      UpdateMotionTL=0;
      ComputeMotionTL();
    }
    /* Code for MotionTL integration - End Section */
    
    /* Code for MotionVC integration - Start Section */
    if(UpdateMotionVC) {
      UpdateMotionVC=0;
      ComputeMotionVC();
    }
    /* Code for MotionVC integration - End Section */
    
    /* Code for AcousticSL integration - Start Section */
    if (SendAudioSourceLocalization)
    {
      SendAudioSourceLocalization = 0;
      SendAudioSourceLocalizationData();
    }
    /* Code for AcousticSL integration - End Section */

    /* Wait for Event */
    __WFI();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */
//////
  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */
//////
  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel0.Instance = DFSDM1_Channel0;
  hdfsdm1_channel0.Init.OutputClock.Activation = DISABLE;
  hdfsdm1_channel0.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel0.Init.OutputClock.Divider = 2;
  hdfsdm1_channel0.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel0.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel0.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel0.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel0.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_EXTERNAL;
  hdfsdm1_channel0.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel0.Init.Awd.Oversampling = 1;
  hdfsdm1_channel0.Init.Offset = 0;
  hdfsdm1_channel0.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */
//////
  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief SAI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI2_Init(void)
{

  /* USER CODE BEGIN SAI2_Init 0 */

  /* USER CODE END SAI2_Init 0 */

  /* USER CODE BEGIN SAI2_Init 1 */

  /* USER CODE END SAI2_Init 1 */
  hsai_BlockA2.Instance = SAI2_Block_A;
  hsai_BlockA2.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA2.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA2.Init.DataSize = SAI_DATASIZE_8;
  hsai_BlockA2.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA2.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA2.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA2.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA2.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_192K;
  hsai_BlockA2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA2.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA2.FrameInit.FrameLength = 8;
  hsai_BlockA2.FrameInit.ActiveFrameLength = 1;
  hsai_BlockA2.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockA2.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA2.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockA2.SlotInit.FirstBitOffset = 0;
  hsai_BlockA2.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA2.SlotInit.SlotNumber = 1;
  hsai_BlockA2.SlotInit.SlotActive = 0x00000000;
  if (HAL_SAI_Init(&hsai_BlockA2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI2_Init 2 */

  /* USER CODE END SAI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = ((SystemCoreClock / 10000) - 1);
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = DEFAULT_uhCCR1_Val;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = DEFAULT_uhCCR2_Val;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = DEFAULT_uhCCR3_Val;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = DEFAULT_uhCCR4_Val;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = ((SystemCoreClock / TIM_CLOCK_INERTIAL) - 1);
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = ((TIM_CLOCK_INERTIAL / ALGO_FREQ_INERTIAL) -1);
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = ((SystemCoreClock / TIM_CLOCK_ENV) - 1);
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = ((TIM_CLOCK_ENV / ALGO_FREQ_ENV) -1);
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = ((SystemCoreClock / TIM_CLOCK_AUDIO_LEVEL) - 1);
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = ((TIM_CLOCK_AUDIO_LEVEL / ALGO_FREQ_AUDIO_LEVEL) -1);
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_CS_Pin_Pin|LD2_Pin|SPI1_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_EXTI_Pin_Pin */
  GPIO_InitStruct.Pin = SPI1_EXTI_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI1_EXTI_Pin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CS_Pin_Pin LD2_Pin SPI1_RST_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin_Pin|LD2_Pin|SPI1_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_ACC_INT_Pin */
  GPIO_InitStruct.Pin = MEMS_ACC_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_ACC_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  This function sets the ACC FS to 2g
  * @param  None
  * @retval None
  */
static void Set2GAccelerometerFullScale(void)
{
  /* Set Full Scale to +/-2g */
  MOTION_SENSOR_SetFullScale(ACCELERO_INSTANCE,MOTION_ACCELERO,2.0f);
  
  /* Read the Acc Sensitivity */
  MOTION_SENSOR_GetSensitivity(ACCELERO_INSTANCE,MOTION_ACCELERO,&sensitivity);
  sensitivity_Mul = sensitivity * ((float) FROM_MG_TO_G);
}

/**
  * @brief  This function dsets the ACC FS to 4g
  * @param  None
  * @retval None
  */
static void Set4GAccelerometerFullScale(void)
{
  /* Set Full Scale to +/-4g */
  MOTION_SENSOR_SetFullScale(ACCELERO_INSTANCE,MOTION_ACCELERO,4.0f);

  /* Read the Acc Sensitivity */
  MOTION_SENSOR_GetSensitivity(ACCELERO_INSTANCE,MOTION_ACCELERO,&sensitivity);
  sensitivity_Mul = sensitivity * ((float) FROM_MG_TO_G);
}

/**
  * @brief  Motion Libraries initialization
  * @param  None
  * @retval None
  */
static void InitMotionLibraries(void)
{
  /* Code for MotionFX integration - Start Section */
  /* Initialize MotionFX Library */
  if(TargetBoardFeatures.MotionFXIsInitalized==0) {
    MotionFX_manager_init();
    MotionFX_manager_start_9X();
    /* Enable magnetometer calibration */
    MagCalibTest();
  }
  /* Code for MotionFX integration - End Section */
  
  /* Code for MotionAR integration - Start Section */
  /* Initialize MotionAR Library */
  if(TargetBoardFeatures.MotionARIsInitalized==0) {
    MotionAR_manager_init();
  }
  /* Code for MotionAR integration - End Section */
  
#ifdef ALLMEMS1_MOTIONCP
  /* Initialize MotionCP Library */ 
  if(TargetBoardFeatures.MotionCPIsInitalized==0)
    MotionCP_manager_init();
#endif /*  ALLMEMS1_MOTIONCP */
  
#ifdef ALLMEMS1_MOTIONFA
  /* Initialize MotionFA Library */ 
  if(TargetBoardFeatures.MotionFAIsInitalized==0)
    MotionFA_manager_init();
#endif /* ALLMEMS1_MOTIONFA */
  
  /* Code for MotionGR integration - Start Section */
  /* Initialize MotionGR Library */
  if(TargetBoardFeatures.MotionGRIsInitalized==0) {
    MotionGR_manager_init();
  }
  /* Code for MotionGR integration - End Section */
  
  /* Initialize MotionID Library */
  if(TargetBoardFeatures.MotionIDIsInitalized==0) {
    MotionID_manager_init();
  }
  
      /* Code for MotionPE integration - Start Section */
      /* Initialize MotionPE Library */
      if(TargetBoardFeatures.MotionPEIsInitalized==0) {
        MotionPE_manager_init();
      }
      /* Code for MotionPE integration - End Section */
      
      /* Code for MotionSD integration - Start Section */
      /* Initialize MotionSD Library */
      if(TargetBoardFeatures.MotionSDIsInitalized==0) {
        MotionSD_manager_init();
      }
      /* Code for MotionSD integration - End Section */
      
      /* Code for MotionTL integration - Start Section */
      /* Initialize MotionTL Library */
      if(TargetBoardFeatures.MotionTLIsInitalized==0) {
        MotionTL_manager_init();
        AccCalibTest();
      }
      /* Code for MotionTL integration - End Section */
      
      /* Code for MotionVC integration - Start Section */
      /* Initialize MotionVC Library */
      if(TargetBoardFeatures.MotionVCIsInitalized==0) {
        MotionVC_manager_init();
      }
      /* Code for MotionVC integration - End Section */
  
  /* Code for AcousticSL integration - Start Section */
  /* Initialize AcousticSL library */ 
  if(TargetBoardFeatures.AcousticSLIsInitalized==0) {
    AcousticSL_Manager_init();
  }
  /* Code for AcousticSL integration - End Section */
}


/**
  * @brief  Enable/Disable BLE Features
  * @param  None
  * @retval None
  */
static void EnableDisableFeatures(void)
{
  /* Enviromental Features */
  if(BLE_Env_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    Environmental_StartStopTimer();
    BLE_Env_NotifyEvent = BLE_NOTIFY_NOTHING;
  }
  
  /* Inertial Features */
  if(BLE_Inertial_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    Inertial_StartStopTimer();   
    BLE_Inertial_NotifyEvent = BLE_NOTIFY_NOTHING;
  }
  
  /* Accelerometer events Features */
  if(BLE_AccEnv_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    AccEnv_StartStop();   
    BLE_AccEnv_NotifyEvent = BLE_NOTIFY_NOTHING;
  }
  
  /* Audio Level Features */
  if(BLE_AudioLevel_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    AudioLevel_StartStopTimer(); 
    BLE_AudioLevel_NotifyEvent = BLE_NOTIFY_NOTHING;
  }
  
  /* Audio Source Localization Features */
  if(BLE_AudioSourceLocalization_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    AudioSourceLocalization_StartStopTimer(); 
    BLE_AudioSourceLocalization_NotifyEvent = BLE_NOTIFY_NOTHING;
  }
  
  /* Sensor Fusion Features */
  if(BLE_SensorFusion_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    TIM1_CHANNEL_1_StartStop();   
    BLE_SensorFusion_NotifyEvent = BLE_NOTIFY_NOTHING;
  }
  
  /* E-Compass Features */
  if(BLE_ECompass_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    TIM1_CHANNEL_1_StartStop();   
    BLE_ECompass_NotifyEvent = BLE_NOTIFY_NOTHING;
  }
  
  /* Activity Recognition Features */
  if(BLE_CarryPosition_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    TIM1_CHANNEL_2_StartStop();   
    BLE_CarryPosition_NotifyEvent = BLE_NOTIFY_NOTHING;
  }
  
  /* Gesture Recognition Features */
  if(BLE_GestureRecognition_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    TIM1_CHANNEL_2_StartStop();   
    BLE_GestureRecognition_NotifyEvent = BLE_NOTIFY_NOTHING;
  }
  
  /* Gesture Recognition Features */
  if(BLE_MotionAlgorithms_VC_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    TIM1_CHANNEL_2_StartStop();   
    BLE_MotionAlgorithms_VC_NotifyEvent = BLE_NOTIFY_NOTHING;
  }
  
  /* Tilt Sensing Features */
  if(BLE_TiltSensing_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    TIM1_CHANNEL_2_StartStop();   
    BLE_TiltSensing_NotifyEvent = BLE_NOTIFY_NOTHING;
  }
  
  /* Activity Recognition Features */
  if(BLE_ActRec_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    TIM1_CHANNEL_3_StartStop();   
    BLE_ActRec_NotifyEvent = BLE_NOTIFY_NOTHING;
  }
  
  /* Motion Intensity Features */
  if(BLE_MotionIntensity_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    TIM1_CHANNEL_3_StartStop();   
    BLE_MotionIntensity_NotifyEvent = BLE_NOTIFY_NOTHING;
  }
  
  /* Motion Algorithms Features - Pose Estimation */
  if(BLE_MotionAlgorithms_PE_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    TIM1_CHANNEL_3_StartStop();   
    BLE_MotionAlgorithms_PE_NotifyEvent = BLE_NOTIFY_NOTHING;
  }
  
  if(BLE_MotionAlgorithms_SD_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    TIM1_CHANNEL_4_StartStop();   
    BLE_MotionAlgorithms_SD_NotifyEvent = BLE_NOTIFY_NOTHING;
  }
  
  if(BLE_FitnessActivities_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    TIM1_CHANNEL_4_StartStop();   
    BLE_FitnessActivities_NotifyEvent = BLE_NOTIFY_NOTHING;
  }
}


/**
  * @brief  Output Compare callback in non blocking mode 
  * @param  htim : TIM OC handle
  * @retval None
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t uhCapture=0;
  
  /* Code for MotionFX integration - Start Section */
  /* TIM1_CH1 toggling with frequency = 100Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + DEFAULT_uhCCR1_Val));

    if ((SensorFusionEnabled) || (ECompassEnabled)) {
      Quaternion=1;
    }
  }
  /* Code for MotionFX integration - End Section */

  /* Code for MotionCP & MotionGR & MotionVC integration - Start Section */
  /* TIM1_CH2 toggling with frequency = 50Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_2, (uhCapture + DEFAULT_uhCCR2_Val));

#ifdef ALLMEMS1_MOTIONCP
    if(CarryPositionEnabled) {
      UpdateMotionCP=1;
    }
#endif /*  ALLMEMS1_MOTIONCP */
    
    if(GestureRecognitionEnabled) {
      UpdateMotionGR=1;
    }
    
    /* Code for MotionTL integration - Start Section */
    if(TiltSensingEnabled) {
      UpdateMotionTL=1;
    }
    /* Code for MotionTL integration - End Section */
    
    /* Code for MotionVC integration - Start Section */
    if(VerticalContextEnabled) {
      UpdateMotionVC=1;
    }
    /* Code for MotionVC integration - End Section */
  }
  /* Code for MotionCP & MotionGR & MotionVC integration - End Section */
  
  /* Code for MotionAR & MotionID & MotionPE integration - Start Section */
  /* TIM1_CH3 toggling with frequency = 16Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
  {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_3, (uhCapture + DEFAULT_uhCCR3_Val));
    
    if(MotionIntensityEnabled) {
      UpdateMotionID=1;
    } else if(ActivityRecognitionEnabled) {
        UpdateMotionAR=1;
        TimeStamp += ALGO_FREQ_AR_ID_PE;
    } else if(PoseEstimationEnabled) {
      UpdateMotionPE=1;
    }
  }
  /* Code for MotionAR & MotionID & MotionPE integration - End Section */

  /* Code for MotionFA & MotionSD - Start Section */
  /* TIM1_CH4 toggling with frequency = 25 Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
  {
     uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_4, (uhCapture + uhCCR4_Val));
  
    /* Code for MotionSD integration - Start Section */
    if(Standing_VS_SittingDeskEnabled) {
      UpdateMotionSD=1;
    }
      
#ifdef ALLMEMS1_MOTIONFA
    if(FitnessActivitiesEnabled) {
      UpdateMotionFA=1;
    }
#endif /* ALLMEMS1_MOTIONFA */
  }
  /* Code for MotionFA & MotionSD - End Section */
}


/**
  * @brief  Period elapsed callback in non blocking mode for Environmental timer
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == (&TimInertialHandle)) {
    /* Inertial  */
    if(InertialTimerEnabled)
      SendInertial=1;
  } else if(htim == (&TimEnvHandle)) {
    /* Environmental */
    if(EnvironmentalTimerEnabled)
      SendEnv=1;
  } else if(htim == (&TimAudioDataHandle)) {
    /* Mic Data */
    if(AudioLevelTimerEnabled)
      SendAudioLevel=1;
    
    /* Code for AcousticSL integration - Start Section */    
    if(AudioSourceLocalizationTimerEnabled)
      SendAudioSourceLocalization= 1;
    /* Code for AcousticSL integration - End Section */
  }
}

/**
  * @brief  Callback for user button
  * @param  None
  * @retval None
  */
static void ButtonCallback(void)
{
  /* Only if connected */
  if(connected) {
    static uint32_t HowManyButtonPress=0;
    static uint32_t tickstart=0;
    uint32_t tickstop;

    if(!tickstart)
      tickstart = HAL_GetTick();

    tickstop = HAL_GetTick();

    if((tickstop-tickstart)>2000) {
      HowManyButtonPress=0;
      tickstart=tickstop;
    }

    if(TargetBoardFeatures.MotionFXIsInitalized)
    {
      if((HowManyButtonPress+1) == N_BUTTON_PRESS)
      {
        ForceReCalibration=1;
        HowManyButtonPress=0;
      }
      else
      {
        HowManyButtonPress++;
        if(BLE_StdTerm_Service==BLE_SERV_ENABLE)
        {
           BytesToWrite = sprintf((char *)BufferToWrite, "%ld in %ldmS Reset Calib\r\n",3-HowManyButtonPress,2000-(tickstop-tickstart));
           Term_Update(BufferToWrite,BytesToWrite);
        }
        else
        {
          ALLMEMS1_PRINTF("%ld in %ldmS Reset Calib\r\n",3-HowManyButtonPress,2000-(tickstop-tickstart));
        }
      }
    }
    else
    {
      ALLMEMS1_PRINTF("UserButton Pressed\r\n");
    }
  }
}

/**
  * @brief  Reset the magneto calibration 
  * @param  None
  * @retval None
  */
static void ReCalibration(uint8_t ReCalibrationDev)
{
  /* Only if connected */
  if(connected) {
    /* Reset the Calibration */
    MagnetoCalibrationDone=0;
    MFX_MagCal_output_t mag_cal_test;

    /* Notifications of Sensor Fusion Calibration */
    if(ReCalibrationDev == 1)
      Config_Update(FEATURE_MASK_SENSORFUSION_SHORT,W2ST_COMMAND_CAL_STATUS,MagnetoCalibrationDone ? 100: 0);
    
    /* Notifications of Compass Calibration */
    if(ReCalibrationDev == 2)
      Config_Update(FEATURE_MASK_ECOMPASS,W2ST_COMMAND_CAL_STATUS,MagnetoCalibrationDone ? 100: 0);

    /* Reset the Calibration */
    if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
       BytesToWrite = sprintf((char *)BufferToWrite, "\nForce ReCalibration\n\r");
       Term_Update(BufferToWrite,BytesToWrite);
    } else
      ALLMEMS1_PRINTF("\nForce ReCalibration\n\r");
    {
       ResetCalibrationInMemory();
    }

    /* Enable magnetometer calibration */
    MotionFX_manager_MagCal_start(SAMPLE_PERIOD);
    MotionFX_MagCal_getParams(&mag_cal_test);

    /* Switch off the LED */
    LedOffTargetPlatform();
  }
}

/**
  * @brief  Send Notification where there is a interrupt from MEMS
  * @param  None
  * @retval None
  */
static void MEMSCallback(void)
{
  MOTION_SENSOR_Event_Status_t status;
  
  MOTION_SENSOR_Get_Event_Status(ACCELERO_INSTANCE,&status);

  if( (W2ST_CHECK_HW_FEATURE(W2ST_HWF_PEDOMETER)) ||
	  (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)) ) {
    /* Check if the interrupt is due to Pedometer */
    if(status.StepStatus != 0) {
      PedometerStepCount = GetStepHWPedometer();
      if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_PEDOMETER)) {
         BLE_AccEnvUpdate(PedometerStepCount, 2);
      }
    }
  }

  if( (W2ST_CHECK_HW_FEATURE(W2ST_HWF_FREE_FALL)) ||
      (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)) ) {
    /* Check if the interrupt is due to Free Fall */
    if(status.FreeFallStatus != 0) {
      BLE_AccEnvUpdate(ACC_FREE_FALL, 2);
    }
  }
  
  if( (W2ST_CHECK_HW_FEATURE(W2ST_HWF_SINGLE_TAP)) ||
      (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)) ) {
    /* Check if the interrupt is due to Single Tap */
    if(status.TapStatus != 0) {
      BLE_AccEnvUpdate(ACC_SINGLE_TAP, 2);
    }
  }

  if( (W2ST_CHECK_HW_FEATURE(W2ST_HWF_DOUBLE_TAP)) ||
      (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)) ) {
    /* Check if the interrupt is due to Double Tap */
    if(status.DoubleTapStatus != 0) {
      BLE_AccEnvUpdate(ACC_DOUBLE_TAP, 2);
    }
  }

  if( (W2ST_CHECK_HW_FEATURE(W2ST_HWF_TILT)) ||
      (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)) ) {
    /* Check if the interrupt is due to Tilt */
    if(status.TiltStatus != 0) {
      BLE_AccEnvUpdate(ACC_TILT, 2);
    }
  }
  
  if( (W2ST_CHECK_HW_FEATURE(W2ST_HWF_6DORIENTATION)) ||
      (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)) ) {
    /* Check if the interrupt is due to 6D Orientation */
    if(status.D6DOrientationStatus != 0) {
      AccEventType Orientation = GetHWOrientation6D();
      BLE_AccEnvUpdate(Orientation, 2);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_WAKE_UP)) {
    /* Check if the interrupt is due to Wake Up */
    if(status.WakeUpStatus != 0) {
      BLE_AccEnvUpdate(ACC_WAKE_UP, 2);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)) {
    BLE_AccEnvUpdate(PedometerStepCount, 3);
  }
}

/**
  * @brief  Send Motion Data Acc/Mag/Gyro to BLE
  * @param  None
  * @retval None
  */
static void SendMotionData(void)
{
  MOTION_SENSOR_Axes_t ACC_Value;
  MOTION_SENSOR_Axes_t GYR_Value;
  MOTION_SENSOR_Axes_t MAG_Value;
  
  BLE_MANAGER_INERTIAL_Axes_t ACC_SensorValue;
  BLE_MANAGER_INERTIAL_Axes_t GYR_SensorValue;
  BLE_MANAGER_INERTIAL_Axes_t MAG_SensorValue;

  /* Read the Acc values */
  MOTION_SENSOR_GetAxes(ACCELERO_INSTANCE,MOTION_ACCELERO,&ACC_Value);

  /* Read the Magneto values */
  MOTION_SENSOR_GetAxes(MAGNETO_INSTANCE,MOTION_MAGNETO, &MAG_Value);

  /* Read the Gyro values */
  MOTION_SENSOR_GetAxes(GYRO_INSTANCE,MOTION_GYRO, &GYR_Value);
  
  ACC_SensorValue.x= ACC_Value.x;
  ACC_SensorValue.y= ACC_Value.y;
  ACC_SensorValue.z= ACC_Value.z;
  
  GYR_SensorValue.x= GYR_Value.x;
  GYR_SensorValue.y= GYR_Value.y;
  GYR_SensorValue.z= GYR_Value.z;

  MAG_SensorValue.x= MAG_Value.x;
  MAG_SensorValue.y= MAG_Value.y;
  MAG_SensorValue.z= MAG_Value.z;

  BLE_AccGyroMagUpdate(&ACC_SensorValue,&GYR_SensorValue,&MAG_SensorValue);
}

#ifdef ALLMEMS1_MOTIONCP
/**
  * @brief  MotionCP Working function
  * @param  None
  * @retval None
  */
static void ComputeMotionCP(void)
{  
  static MCP_output_t CarryPositionCodeStored = MCP_UNKNOWN;
  BLE_CP_output_t CarryPositionCodeSent;
  MOTION_SENSOR_AxesRaw_t ACC_Value_Raw;

  /* Read the Acc RAW values */
  MOTION_SENSOR_GetAxesRaw(ACCELERO_INSTANCE,MOTION_ACCELERO,&ACC_Value_Raw);
  MotionCP_manager_run(ACC_Value_Raw);

  if(CarryPositionCodeStored!=CarryPositionCode){
    CarryPositionCodeStored = CarryPositionCode;
    
    CarryPositionCodeSent= (BLE_CP_output_t)CarryPositionCode;
    BLE_CarryPositionUpdate(CarryPositionCodeSent);

    if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
       BytesToWrite = sprintf((char *)BufferToWrite,"Sending: Carry Position Code= %d\r\n",CarryPositionCode);
       Term_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Sending: Carry Position Code= %d\r\n",CarryPositionCode);
    }
  }
}
#endif /*  ALLMEMS1_MOTIONCP */

#ifdef ALLMEMS1_MOTIONFA
/**
  * @brief  MotionFA Working function
  * @param  None
  * @retval None
  */
static void ComputeMotionFA(void)
{  
  MFA_input_t data_in = {.AccX = 0.0f, .AccY = 0.0f, .AccZ = 0.0f, .GyrX = 0.0f, .GyrZ = 0.0f, .GyrZ = 0.0f, .Press = 0.0f};
  static MFA_output_t data_out;
  static MFA_output_t data_out_prev = {0};
  MFA_activity_t activity_type;
  
  MOTION_SENSOR_Axes_t ACC_Value;
  MOTION_SENSOR_Axes_t GYRO_Value;
  float PresValue;

  /* Read the Acc values */
  MOTION_SENSOR_GetAxes(ACCELERO_INSTANCE,MOTION_ACCELERO, &ACC_Value);
  
  /* Read the Gyro values */
  MOTION_SENSOR_GetAxes(ACCELERO_INSTANCE,MOTION_GYRO, &GYRO_Value);
  
  /* Read the Pressure value */
  ENV_SENSOR_GetValue(PRESSURE_INSTANCE,ENV_PRESSURE, &PresValue);

  /* Convert acceleration from [mg] to [g] */
  data_in.AccX = (float)ACC_Value.x / 1000.0f;
  data_in.AccY = (float)ACC_Value.y / 1000.0f;
  data_in.AccZ = (float)ACC_Value.z / 1000.0f;
  
  /* Convert angular velocity from [mdps] to [dps] */
  data_in.GyrX = (float)GYRO_Value.x / 1000.0f;
  data_in.GyrY = (float)GYRO_Value.y / 1000.0f;
  data_in.GyrZ = (float)GYRO_Value.z / 1000.0f;

  /* Add pressure [hPa] */
  data_in.Press = PresValue;
  
  /* Run Fitness Activities algorithm */
  MotionFA_manager_run(&data_in, &data_out);
  MotionFA_manager_get_activity(&activity_type);

  if( data_out_prev.Counter != data_out.Counter )
  {
    data_out_prev.Counter= data_out.Counter;
    
    BLE_FitnessActivitiesUpdate(activity_type, data_out.Counter);
    
    if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
       BytesToWrite = sprintf((char *)BufferToWrite,"Sending: FA --> ActivityType= %d   Counter= %ld\r\n", activity_type, data_out.Counter);
       Term_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Sending: FA --> ActivityType= %d   Counter= %ld\r\n", activity_type, data_out.Counter);
    }
  }
}
#endif /* ALLMEMS1_MOTIONFA */

/**
  * @brief  MotionGR Working function
  * @param  None
  * @retval None
  */
static void ComputeMotionGR(void)
{
  static MGR_output_t GestureRecognitionCodeStored = MGR_NOGESTURE;
  BLE_GR_output_t GestureRecognitionCodeSent;
  MOTION_SENSOR_AxesRaw_t ACC_Value_Raw;

  /* Read the Acc RAW values */
  MOTION_SENSOR_GetAxesRaw(ACCELERO_INSTANCE,MOTION_ACCELERO,&ACC_Value_Raw);
  MotionGR_manager_run(ACC_Value_Raw);

  if(GestureRecognitionCodeStored!=GestureRecognitionCode){
    GestureRecognitionCodeStored = GestureRecognitionCode;
    GestureRecognitionCodeSent= (BLE_GR_output_t)GestureRecognitionCode;
    BLE_GestureRecognitionUpdate(GestureRecognitionCodeSent);

    if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
       BytesToWrite = sprintf((char *)BufferToWrite,"Sending: Gesture Recognition Code= %d\r\n",GestureRecognitionCode);
       Term_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Sending: Gesture Recognition Code= %d\r\n",GestureRecognitionCode);
    }
  }
}

/**
  * @brief  MotionID Working function
  * @param  None
  * @retval None
  */
static void ComputeMotionID(void)
{
  static MID_output_t MIDStored = MID_ON_DESK; /* on desk */
  BLE_ID_output_t MIDCodeSent;
  MOTION_SENSOR_AxesRaw_t ACC_Value_Raw;

  /* Read the Acc RAW values */
  MOTION_SENSOR_GetAxesRaw(ACCELERO_INSTANCE,MOTION_ACCELERO,&ACC_Value_Raw);

  MotionID_manager_run(ACC_Value_Raw);

  if(MIDStored!=MIDCode){
    MIDStored = MIDCode;
    
    MIDCodeSent= (BLE_ID_output_t)MIDCode;

    BLE_MotionIntensityUpdate(MIDCodeSent);

    if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
       BytesToWrite = sprintf((char *)BufferToWrite,"Sending: Motion Intensity Code= %d\r\n",MIDCode);
       Term_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Sending: Motion Intensity Code= %d\r\n",MIDCode);
    }
  }
}

/* Code for MotionPE integration - Start Section */
/**
  * @brief  MotionPE Working function
  * @param  None
  * @retval None
  */
static void ComputeMotionPE(void)
{
  MPE_input_t data_in = {.AccX = 0.0f, .AccY = 0.0f, .AccZ = 0.0f};
  static MPE_output_t data_out;
  static uint8_t PoseEstimationCodeStored = 0xFF;
  
  MOTION_SENSOR_Axes_t ACC_Value;

  /* Read the Acc values */
  MOTION_SENSOR_GetAxes(ACCELERO_INSTANCE,MOTION_ACCELERO, &ACC_Value);

  /* Convert acceleration from [mg] to [g] */
  data_in.AccX = (float)ACC_Value.x / 1000.0f;
  data_in.AccY = (float)ACC_Value.y / 1000.0f;
  data_in.AccZ = (float)ACC_Value.z / 1000.0f;
  
  /* Run Pose Estimation algorithm */
  MotionPE_manager_run(&data_in, &data_out);
  
  if(PoseEstimationCodeStored!=(uint8_t)data_out)
  {
    PoseEstimationCodeStored = data_out;
    BLE_MotionAlgorithmsUpdate((uint8_t)data_out);

    if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
       BytesToWrite = sprintf((char *)BufferToWrite,"Sending: PE=%d\r\n",data_out);
       Term_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Sending: PE=%d\r\n",data_out);
    }
  }
}
/* Code for MotionPE integration - End Section */

/* Code for MotionSD integration - Start Section */
/**
  * @brief  MotionSD Working function
  * @param  None
  * @retval None
  */
static void ComputeMotionSD(void)
{
  MSD_input_t data_in = {.AccX = 0.0f, .AccY = 0.0f, .AccZ = 0.0f, .Press = 0.0f};
  static uint8_t StandingSittingDeskCodeStored = 0xFF;
  static MSD_output_t data_out;
  
  MOTION_SENSOR_Axes_t ACC_Value;
  float PresValue;

  /* Read the Acc values */
  MOTION_SENSOR_GetAxes(ACCELERO_INSTANCE,MOTION_ACCELERO, &ACC_Value);
  
  /* Read the Pressure value */
  ENV_SENSOR_GetValue(PRESSURE_INSTANCE,ENV_PRESSURE, &PresValue);

  /* Add acceleration [g] */
  data_in.AccX = (float)ACC_Value.x / 1000.0f;
  data_in.AccY = (float)ACC_Value.y / 1000.0f;
  data_in.AccZ = (float)ACC_Value.z / 1000.0f;

  /* Add pressure [hPa] */
  data_in.Press = PresValue;
  
  /* Run Standing vs Sitting Desk algorithm */
  MotionSD_manager_run(&data_in, &data_out);
  
  if(StandingSittingDeskCodeStored!=(uint8_t)data_out)
  {
    StandingSittingDeskCodeStored = data_out;
    BLE_MotionAlgorithmsUpdate((uint8_t)data_out);

    if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
       BytesToWrite = sprintf((char *)BufferToWrite,"Sending: SD=%d\r\n",data_out);
       Term_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Sending: SD=%d\r\n",data_out);
    }
  }
}
/* Code for MotionSD integration - End Section */

/* Code for MotionTL integration - Start Section */
static void ComputeMotionTL(void)
{
  MTL_input_t data_in = {.acc_x = 0.0f, .acc_y = 0.0f, .acc_z = 0.0f};
  
  static MTL_output_t data_out = {.theta_3x = 0.0f, .psi_3x   = 0.0f, .phi_3x   = 0.0f,
                                  .roll_3x  = 0.0f, .pitch_3x = 0.0f, .err_deg  = 0.0f,
                                  .valid    = 0 };
  
  uint64_t timestamp_ms;
  
  MOTION_SENSOR_Axes_t ACC_Value;
  
  BLE_ANGLES_output_t TiltSensingResult;

  /* Read the Acc values */
  MOTION_SENSOR_GetAxes(ACCELERO_INSTANCE,MOTION_ACCELERO, &ACC_Value);

  /* Convert acceleration from [mg] to [g] */
  data_in.acc_x = (float)ACC_Value.x / 1000.0f;
  data_in.acc_y = (float)ACC_Value.y / 1000.0f;
  data_in.acc_z = (float)ACC_Value.z / 1000.0f;
  timestamp_ms  = HAL_GetTick();
  
  /* Run Tilt Sensing algorithm */
  MotionTL_manager_run(&data_in, timestamp_ms, &data_out);

  switch (ANGLE_MODE)
  {
    case MODE_PITCH_ROLL_GRAVITY_INCLINATION:
      /* Get angles (pitch, roll and gravity inclination) */
      TiltSensingResult.AnglesArray[0]=  data_out.pitch_3x;
      TiltSensingResult.AnglesArray[1]=  data_out.roll_3x;
      TiltSensingResult.AnglesArray[2]=  data_out.phi_3x;  /* Gravity Inclination */
      break;

    case MODE_THETA_PSI_PHI:
      /* Get angles (theta, psi and phi) */
      TiltSensingResult.AnglesArray[0]=  data_out.theta_3x;
      TiltSensingResult.AnglesArray[1]=  data_out.psi_3x;
      TiltSensingResult.AnglesArray[2]=  data_out.phi_3x;
      break;
  }
  
  BLE_TiltSensingUpdate(TiltSensingResult);

  if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
     BytesToWrite = sprintf((char *)BufferToWrite,"Sending Tilt Measurement\r\n");
     Term_Update(BufferToWrite,BytesToWrite);
  } else {
    //ALLMEMS1_PRINTF("Sending Tilt Measurement: Pitch=\r\n");
  }
}

/**
 * @brief  Collect accelerometer data
 * @param  cal_data Pointer to 2D array of calibration data cal_data[num_records][3]
 * @param  num_records Number of records to be taken (3 axes per record)
 * @retval 0  Ok
 * @retval 1  Accelerometer error
 */
uint8_t CollectData(float cal_data[][3], uint32_t num_records)
{
  uint32_t i = 0;
  uint8_t status = 0;
  
  MOTION_SENSOR_Axes_t AccValue;

  /* Clean DRDY */
  (void)MOTION_SENSOR_GetAxes(ACCELERO_INSTANCE, MOTION_ACCELERO, &AccValue);

  while (i < num_records)
  {
    if (MOTION_SENSOR_Get_DRDY_Status(ACCELERO_INSTANCE, MOTION_ACCELERO, &status) != BSP_ERROR_NONE)
    {
      return 1;
    }

    if (status == 1)
    {
      if (MOTION_SENSOR_GetAxes(ACCELERO_INSTANCE, MOTION_ACCELERO, &AccValue) != BSP_ERROR_NONE)
      {
        return 1;
      }

      cal_data[i][0] = (float)AccValue.x / 1000.0f;
      cal_data[i][1] = (float)AccValue.y / 1000.0f;
      cal_data[i][2] = (float)AccValue.z / 1000.0f;
      i++;
    }
  }

  return 0;
}

/**
 * @brief  Get estimated measurement time
 * @param  time_s Pointer to time in [s]
 * @param  num_records Number of records taken
 * @retval None
 */
void GetEstimatedMeasTime(float *time_s, uint32_t num_records)
{
  float odr = 0.0f;

  (void)MOTION_SENSOR_Enable(ACCELERO_INSTANCE, MOTION_ACCELERO);


  (void)MOTION_SENSOR_GetOutputDataRate(ACCELERO_INSTANCE, MOTION_ACCELERO, &odr);

  if (odr > 0.001f)
  {
    *time_s = (float)num_records / odr;
  }
}
/* Code for MotionTL integration - End Section */

/* Code for MotionVC integration - Start Section */
/**
  * @brief  MotionVC Working function
  * @param  None
  * @retval None
  */
static void ComputeMotionVC(void)
{
  MVC_input_t data_in = {.AccX = 0.0f, .AccY = 0.0f, .AccZ = 0.0f, .Press = 0.0f};
  static MVC_output_t data_out;
  static uint8_t VerticalContextCodeStored = 0xFF;
  
  MOTION_SENSOR_Axes_t ACC_Value;
  float PresValue;

  /* Read the Acc values */
  MOTION_SENSOR_GetAxes(ACCELERO_INSTANCE,MOTION_ACCELERO, &ACC_Value);
  
  /* Read the Pressure value */
  ENV_SENSOR_GetValue(PRESSURE_INSTANCE,ENV_PRESSURE, &PresValue);

  /* Add acceleration [g] */
  data_in.AccX = (float)ACC_Value.x / 1000.0f;
  data_in.AccY = (float)ACC_Value.y / 1000.0f;
  data_in.AccZ = (float)ACC_Value.z / 1000.0f;

  /* Add pressure [hPa] */
  data_in.Press = PresValue;
  
  /* Run Vertical Context algorithm */
  MotionVC_manager_update(&data_in, &data_out);
  
  if(VerticalContextCodeStored!=data_out.Context)
  {
    VerticalContextCodeStored = data_out.Context;
    BLE_MotionAlgorithmsUpdate((uint8_t)data_out.Context);

    if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
       BytesToWrite = sprintf((char *)BufferToWrite,"Sending: VC=%d     Conf=%d\r\n",data_out.Context, data_out.Confidence);
       Term_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Sending: VC=%d   Conf=%d\r\n",data_out.Context, data_out.Confidence);
    }
  }
}
/* Code for MotionVC integration - End Section */

/* Code for MotionFX integration - Star Section */
/** @brief  MotionFX Working function
 * @param  None
 * @retval None
 */
static void ComputeQuaternions(void)
{
  static MOTION_SENSOR_Axes_t quat_axes[SEND_N_QUATERNIONS];
  
  static BLE_MOTION_SENSOR_Axes_t quat_axes_send[SEND_N_QUATERNIONS];
  
  static int32_t calibIndex =0;
  static int32_t CounterFX  =0;
  static int32_t CounterEC  =0;

  MOTION_SENSOR_Axes_t ACC_Value;
  MOTION_SENSOR_Axes_t GYR_Value;
  MOTION_SENSOR_Axes_t MAG_Value;
  
  MFX_input_t data_in;
  MFX_input_t *pdata_in = &data_in;
  MFX_output_t data_out;
  MFX_output_t *pdata_out = &data_out;
  
  MFX_MagCal_input_t mag_data_in;

  /* Increment the Counter */
  if(ECompassEnabled) {
    CounterEC++;
  } else {
    CounterFX++;
  }
  
  /* Read the Acc RAW values */
  MOTION_SENSOR_GetAxes(ACCELERO_INSTANCE,MOTION_ACCELERO,&ACC_Value);
  /* Convert acceleration from [mg] to [g] */
  data_in.acc[0] = (float)ACC_Value.x * FROM_MG_TO_G;
  data_in.acc[1] = (float)ACC_Value.y * FROM_MG_TO_G;
  data_in.acc[2] = (float)ACC_Value.z * FROM_MG_TO_G;

  /* Read the Magneto values */
  MOTION_SENSOR_GetAxes(MAGNETO_INSTANCE,MOTION_MAGNETO,&MAG_Value);
  /* Convert magnetic field intensity from [mGauss] to [uT / 50] */
  data_in.mag[0] = (float)(MAG_Value.x - MAG_Offset.x) * FROM_MGAUSS_TO_UT50;
  data_in.mag[1] = (float)(MAG_Value.y - MAG_Offset.y) * FROM_MGAUSS_TO_UT50;
  data_in.mag[2] = (float)(MAG_Value.z - MAG_Offset.z) * FROM_MGAUSS_TO_UT50;

  /* Read the Gyro values */
  MOTION_SENSOR_GetAxes(GYRO_INSTANCE,MOTION_GYRO,&GYR_Value);
  /* Convert angular velocity from [mdps] to [dps] */
  data_in.gyro[0] = (float)GYR_Value.x * FROM_MDPS_TO_DPS;
  data_in.gyro[1] = (float)GYR_Value.y * FROM_MDPS_TO_DPS;
  data_in.gyro[2] = (float)GYR_Value.z * FROM_MDPS_TO_DPS;

  /* Check if is calibrated */
  if(MagnetoCalibrationDone!=0x01){
    /* Run Compass Calibration @ 25Hz */
    calibIndex++;
    if (calibIndex == 4){
      calibIndex = 0;
      mag_data_in.mag[0]= (float)MAG_Value.x * FROM_MGAUSS_TO_UT50;
      mag_data_in.mag[1]= (float)MAG_Value.y * FROM_MGAUSS_TO_UT50;
      mag_data_in.mag[2]= (float)MAG_Value.z * FROM_MGAUSS_TO_UT50;
      
      mag_data_in.time_stamp = (int)mag_time_stamp;
      mag_time_stamp += SAMPLE_PERIOD;
      
      MotionFX_manager_MagCal_run(&mag_data_in, &magOffset);
      
      /* Control the calibration status */
      if( (magOffset.cal_quality == MFX_MAGCALOK) ||
          (magOffset.cal_quality == MFX_MAGCALGOOD) ) {
        MagnetoCalibrationDone= 1;
        
        MAG_Offset.x= (int32_t)(magOffset.hi_bias[0] * FROM_UT50_TO_MGAUSS);
        MAG_Offset.y= (int32_t)(magOffset.hi_bias[1] * FROM_UT50_TO_MGAUSS);
        MAG_Offset.z= (int32_t)(magOffset.hi_bias[2] * FROM_UT50_TO_MGAUSS);
        
        /* Disable magnetometer calibration */
        MotionFX_manager_MagCal_stop(SAMPLE_PERIOD);
        
        SaveMagnetoCalibrationToMemory();
      }
        
      if(MagnetoCalibrationDone == 0x01){
        if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
        //if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
          BytesToWrite = sprintf((char *)BufferToWrite, "Compass Calibrated\n\r");
          Term_Update(BufferToWrite,BytesToWrite);
        } else {
          ALLMEMS1_PRINTF("Compass Calibrated\n\r");
        }
        
        /* Switch on the Led */
        LedOnTargetPlatform();

        /* Notifications of Compass Calibration */
        Config_Update(FEATURE_MASK_SENSORFUSION_SHORT,W2ST_COMMAND_CAL_STATUS,MagnetoCalibrationDone ? 100: 0);
        Config_Update(FEATURE_MASK_ECOMPASS,W2ST_COMMAND_CAL_STATUS,MagnetoCalibrationDone ? 100: 0);
      }
    }
  } else {
    calibIndex=0;
  }
  
  MotionFX_manager_run(pdata_in, pdata_out, MOTION_FX_ENGINE_DELTATIME);

  if(ECompassEnabled) {
    /* E-Compass Updated every 0.1 Seconds*/
    if(CounterEC==10) {
      uint16_t Angle = (uint16_t)trunc(100*pdata_out->heading);
      CounterEC=0;
      BLE_ECompassUpdate(Angle);
    }
  } else {
    int32_t QuaternionNumber = (CounterFX>SEND_N_QUATERNIONS) ? (SEND_N_QUATERNIONS-1) : (CounterFX-1);

    /* Scaling quaternions data by a factor of 10000
      (Scale factor to handle float during data transfer BT) */

    /* Save the quaternions values */
    if(pdata_out->quaternion[3] < 0){
      quat_axes[QuaternionNumber].x = (int32_t)(pdata_out->quaternion[0] * (-10000));
      quat_axes[QuaternionNumber].y = (int32_t)(pdata_out->quaternion[1] * (-10000));
      quat_axes[QuaternionNumber].z = (int32_t)(pdata_out->quaternion[2] * (-10000));
    } else {
      quat_axes[QuaternionNumber].x = (int32_t)(pdata_out->quaternion[0] * 10000);
      quat_axes[QuaternionNumber].y = (int32_t)(pdata_out->quaternion[1] * 10000);
      quat_axes[QuaternionNumber].z = (int32_t)(pdata_out->quaternion[2] * 10000);
    }
    
    quat_axes_send[QuaternionNumber].x = quat_axes[QuaternionNumber].x;
    quat_axes_send[QuaternionNumber].y = quat_axes[QuaternionNumber].y;
    quat_axes_send[QuaternionNumber].z = quat_axes[QuaternionNumber].z;
      
    /* Every QUAT_UPDATE_MUL_10MS*10 mSeconds Send Quaternions informations via bluetooth */
    if(CounterFX==QUAT_UPDATE_MUL_10MS){
      BLE_SensorFusionUpdate(quat_axes_send, SEND_N_QUATERNIONS);
      CounterFX=0;
    }
  }
}
/* Code for MotionFX integration - End Section */

/* Code for MotionAR integration - Start Section */
/**
  * @brief  MotionAR Working function
  * @param  None
  * @retval None
  */
static void ComputeMotionAR(void)
{
  static MAR_output_t ActivityCodeStored = MAR_NOACTIVITY;
  BLE_AR_output_t ActivityCodeSent;
  MOTION_SENSOR_AxesRaw_t ACC_Value_Raw;

  /* Read the Acc RAW values */
  MOTION_SENSOR_GetAxesRaw(ACCELERO_INSTANCE,MOTION_ACCELERO,&ACC_Value_Raw);

  MotionAR_manager_run(ACC_Value_Raw, TimeStamp);

  if(ActivityCodeStored!=ActivityCode){
    ActivityCodeStored = ActivityCode;
    
    ActivityCodeSent= (BLE_AR_output_t)ActivityCode;

    BLE_ActRecUpdate(ActivityCodeSent, HAR_ALGO_IDX_NONE);

    if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
       BytesToWrite = sprintf((char *)BufferToWrite,"Sending: Activity Recognition Code= %d\r\n",ActivityCode);
       Term_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Sending: Activity Recognition Code= %d\r\n",ActivityCode);
    }
  }
}
/* Code for MotionAR integration - End Section */

/**
* @brief  User function that is called when 1 ms of PDM data is available.
* @param  none
* @retval None
*/
void AudioProcess(void)
{
  if(AudioLevelTimerEnabled)
  {
    AudioProcess_DB_Noise();
  }
  
  /* Code for AcousticSL integration - Start Section */
  if(AudioSourceLocalizationTimerEnabled)
    AudioProcess_SL();
  /* Code for AcousticSL integration - End Section */
}

/**
* @brief  User function that is called when 1 ms of PDM data is available.
* @param  none
* @retval None
*/
void AudioProcess_DB_Noise(void)
{
  int32_t i;
  int32_t NumberMic;
  
  if(AudioLevelTimerEnabled) {
    for(i = 0; i < (NumSample/AUDIO_IN_CHANNELS); i++){
      for(NumberMic=0;NumberMic<AUDIO_IN_CHANNELS;NumberMic++) {
        RMS_Ch[NumberMic] += (float)((int16_t)PCM_Buffer[i*AUDIO_IN_CHANNELS+NumberMic] * ((int16_t)PCM_Buffer[i*AUDIO_IN_CHANNELS+NumberMic]));
      }
    }
  }
}

/**
  * @brief  Send Audio Level Data (Ch1) to BLE
  * @param  None
  * @retval None
  */
static void SendAudioLevelData(void)
{
  int32_t NumberMic;
  uint16_t DBNOISE_Value_Ch[AUDIO_IN_CHANNELS];
  
  for(NumberMic=0;NumberMic<(AUDIO_IN_CHANNELS);NumberMic++) {
    DBNOISE_Value_Ch[NumberMic] = 0;

    RMS_Ch[NumberMic] /= ((float)(NumSample/AUDIO_IN_CHANNELS)*ALGO_PERIOD_AUDIO_LEVEL);

    DBNOISE_Value_Ch[NumberMic] = (uint16_t)((120.0f - 20 * log10f(32768 * (1 + 0.25f * (AUDIO_VOLUME_INPUT /*AudioInVolume*/ - 4))) + 10.0f * log10f(RMS_Ch[NumberMic])) * 0.3f + DBNOISE_Value_Old_Ch[NumberMic] * 0.7f);
    DBNOISE_Value_Old_Ch[NumberMic] = DBNOISE_Value_Ch[NumberMic];
    RMS_Ch[NumberMic] = 0.0f;
  }
  
  BLE_AudioLevelUpdate(DBNOISE_Value_Ch, AUDIO_IN_CHANNELS);
}

/* Code for AcousticSL integration - Start Section */
/**
  * @brief  Send Audio Source Localization Data to BLE
  * @param  None
  * @retval None
  */
void SendAudioSourceLocalizationData(void)
{
  BLE_AudioSourceLocalizationUpdate(SourceLocationToSend);
}
/* Code for AcousticSL integration - End Section */

/**
* @brief  Half Transfer user callback, called by BSP functions.
* @param  None
* @retval None
*/
void CCA02M2_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance)
{
  AudioProcess();
}

/**
* @brief  Transfer Complete user callback, called by BSP functions.
* @param  None
* @retval None
*/
void CCA02M2_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance)
{
  AudioProcess();
}

/**
  * @brief  Read Environmental Data (Temperature/Pressure/Humidity) from sensor
  * @param  int32_t *PressToSend
  * @param  uint16_t *HumToSend
  * @param  int16_t *Temp1ToSend
  * @param  int16_t *Temp2ToSend
  * @retval None
  */
void ReadEnvironmentalData(int32_t *PressToSend,uint16_t *HumToSend,int16_t *Temp1ToSend,int16_t *Temp2ToSend)
{
  float SensorValue;
  int32_t decPart, intPart;
  
  *PressToSend=0;
  *HumToSend=0;
  *Temp2ToSend=0,*Temp1ToSend=0;

  /* Read Humidity */
  ENV_SENSOR_GetValue(HUMIDITY_INSTANCE,ENV_HUMIDITY,&SensorValue);
  MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
  *HumToSend = intPart*10+decPart;

  /* Read Temperature for sensor 1 */
  ENV_SENSOR_GetValue(TEMPERATURE_INSTANCE_1,ENV_TEMPERATURE,&SensorValue);
  MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
  *Temp1ToSend = intPart*10+decPart;
  
  /* Read Pressure */
  ENV_SENSOR_GetValue(PRESSURE_INSTANCE,ENV_PRESSURE,&SensorValue);
  MCR_BLUEMS_F2I_2D(SensorValue, intPart, decPart);
  *PressToSend=intPart*100+decPart;

  /* Read Temperature for sensor 2 */
  ENV_SENSOR_GetValue(TEMPERATURE_INSTANCE_2,ENV_TEMPERATURE,&SensorValue);
  MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
  *Temp2ToSend = intPart*10+decPart;
}

/**
  * @brief  Send Environmetal Data (Temperature/Pressure/Humidity) to BLE
  * @param  None
  * @retval None
  */
static void SendEnvironmentalData(void)
{
  /* Notifications of Compass Calibration status*/
  if(FirstConnectionConfig) {
    Config_Update(FEATURE_MASK_SENSORFUSION_SHORT,W2ST_COMMAND_CAL_STATUS,MagnetoCalibrationDone ? 100: 0);
    Config_Update(FEATURE_MASK_ECOMPASS,W2ST_COMMAND_CAL_STATUS,MagnetoCalibrationDone ? 100: 0);
#ifdef ALLMEMS1_DEBUG_NOTIFY_TRAMISSION
     if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
       BytesToWrite = sprintf((char *)BufferToWrite,"Cal=%d\r\n",MagnetoCalibrationDone);
       Term_Update(BufferToWrite,BytesToWrite);
     } else {
      ALLMEMS1_PRINTF("Cal=%d\r\n",MagnetoCalibrationDone);
     }
#endif /* ALLMEMS1_DEBUG_NOTIFY_TRAMISSION */
    FirstConnectionConfig=0;
  
    /* Switch on/off the LED according to calibration */
    if(MagnetoCalibrationDone){
        LedOnTargetPlatform();
    } else {
      LedOffTargetPlatform();
    }
  }

  /* Pressure,Humidity, and Temperatures*/
  if(EnvironmentalTimerEnabled) {
    int32_t PressToSend;
    uint16_t HumToSend;
    int16_t Temp2ToSend,Temp1ToSend;
    
    /* Read all the Environmental Sensors */
    ReadEnvironmentalData(&PressToSend,&HumToSend, &Temp1ToSend,&Temp2ToSend);

#ifdef ALLMEMS1_DEBUG_NOTIFY_TRAMISSION
    if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
      BytesToWrite = sprintf((char *)BufferToWrite,"Sending: ");
      Term_Update(BufferToWrite,BytesToWrite);
      BytesToWrite = sprintf((char *)BufferToWrite,"Press=%ld ",PressToSend);
      Term_Update(BufferToWrite,BytesToWrite);
      BytesToWrite = sprintf((char *)BufferToWrite,"Hum=%d ",HumToSend);
      Term_Update(BufferToWrite,BytesToWrite);
      BytesToWrite = sprintf((char *)BufferToWrite,"Temp=%d ",Temp1ToSend);
      Term_Update(BufferToWrite,BytesToWrite);
      BytesToWrite = sprintf((char *)BufferToWrite,"Temp2=%d ",Temp2ToSend);
      Term_Update(BufferToWrite,BytesToWrite);
      BytesToWrite = sprintf((char *)BufferToWrite,"\r\n");
      Term_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Sending: ");
      ALLMEMS1_PRINTF("Press=%ld ",PressToSend);
      ALLMEMS1_PRINTF("Hum=%d ",HumToSend);
      ALLMEMS1_PRINTF("Temp1=%d ",Temp1ToSend);
      ALLMEMS1_PRINTF("Temp2=%d ",Temp2ToSend);
      ALLMEMS1_PRINTF("\r\n");
    }
#endif /* ALLMEMS1_DEBUG_NOTIFY_TRAMISSION */
    
    BLE_EnvironmentalUpdate(PressToSend,HumToSend,Temp2ToSend,Temp1ToSend);
  }
}

/**
  * @brief This function provides accurate delay (in milliseconds) based 
  *        on variable incremented.
  * @note This is a user implementation using WFI state
  * @param Delay: specifies the delay time length, in milliseconds.
  * @retval None
  */
void HAL_Delay(__IO uint32_t Delay)
{
  uint32_t tickstart = 0;
  tickstart = HAL_GetTick();
  while((HAL_GetTick() - tickstart) < Delay){
    __WFI();
  }
}

/**
 * @brief  EXTI line detection callback.
 * @param  uint16_t GPIO_Pin Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{  
  switch(GPIO_Pin){
  case MEMS_ACC_INT_Pin:
    MEMSInterrupt=1;
    break;

  case B1_Pin:
    ButtonPressed = 1;
    break;
  }
}

/**
 * @brief  BSP Push Button callback
 * @param  Button Specifies the pin connected EXTI line
 * @retval None.
 */
void BSP_PB_Callback(Button_TypeDef Button)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Button);
  
  ButtonPressed = 1; 
}

/**
 * @brief  Test if calibration data are available
 * @param  None
 * @retval None
 */
static void MagCalibTest(void)
{
  ReCallMagnetoCalibrationFromMemory();
    
  if(MagCalibrationData[0]== CHECK_CALIBRATION) {
    if(MagCalibrationData[1] == TargetBoardFeatures.mems_expansion_board) {
      if( (MagCalibrationData[2] == MFX_MAGCALOK) ||
          (MagCalibrationData[2] == MFX_MAGCALGOOD) ) {
        MAG_Offset.x = MagCalibrationData[3];
        MAG_Offset.y = MagCalibrationData[4];
        MAG_Offset.z = MagCalibrationData[5];

        MagnetoCalibrationDone =1;
        
        ALLMEMS1_PRINTF("Magneto Calibration Read\r\n");
      } else {
        MagnetoCalibrationDone =0;
        ALLMEMS1_PRINTF("Magneto Calibration quality is not good\r\n");
      }
    } else {
      ALLMEMS1_PRINTF("Magneto Calibration Not correct for Current %s board\r\n",TargetBoardFeatures.mems_expansion_board ? "IKS01A3" : "IKS01A2");
      ResetCalibrationInMemory();    
      MagnetoCalibrationDone=0;
    }
  } else {
    ALLMEMS1_PRINTF("Magneto Calibration Not present\r\n");
    MagnetoCalibrationDone=0;
  }
  
  if(!MagnetoCalibrationDone) {
    MAG_Offset.x = 0;
    MAG_Offset.y = 0;
    MAG_Offset.z = 0;
  }
}

/**
 * @brief  Check if there are a valid Calibration Values in Memory and read them
 * @param uint32_t *MagnetoCalibration the Magneto Calibration
 * @retval unsigned char Success/Not Success
 */
unsigned char ReCallMagnetoCalibrationFromMemory(void)
{
  /* ReLoad the Calibration Values from RAM */
  unsigned char Success=0;

  /* Recall the calibration Credential saved */
  MDM_ReCallGMD(GMD_MAG_CALIBRATION,(void *)&MagCalibrationData);
  
  return Success;
}

/**
 * @brief  Save the Magnetometer Calibration Values to Memory
 * @param uint32_t *MagnetoCalibration the Magneto Calibration
 * @retval unsigned char Success/Not Success
 */
unsigned char SaveMagnetoCalibrationToMemory(void)
{
  unsigned char Success=1;

  /* Store in RAM */
  MagCalibrationData[0] = CHECK_CALIBRATION;
  MagCalibrationData[1] = TargetBoardFeatures.mems_expansion_board;
  MagCalibrationData[2] = magOffset.cal_quality;
  MagCalibrationData[3] = MAG_Offset.x;
  MagCalibrationData[4] = MAG_Offset.y;
  MagCalibrationData[5] = MAG_Offset.z;

  if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
   BytesToWrite = sprintf((char *)BufferToWrite, "Magneto Calibration will be saved in FLASH\r\n");
   Term_Update(BufferToWrite,BytesToWrite);
  } else {
    ALLMEMS1_PRINTF("Magneto Calibration will be saved in FLASH\r\n");
  }

  MDM_SaveGMD(GMD_MAG_CALIBRATION,(void *)&MagCalibrationData);
  
  NecessityToSaveMetaDataManager=1;

  return Success;
}

/**
 * @brief  Reset the Magnetometer Calibration Values in Memory
 * @param uint32_t *MagnetoCalibration the Magneto Calibration
 * @retval unsigned char Success/Not Success
 */
static unsigned char ResetCalibrationInMemory(void)
{
  /* Reset Calibration Values in RAM */
  unsigned char Success=1;
  int32_t Counter;

  for(Counter=0;Counter<6;Counter++)
    MagCalibrationData[Counter]=0x0;
    //MagCalibrationData[Counter]=0xFFFFFFFF;

  if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
     BytesToWrite = sprintf((char *)BufferToWrite, "Magneto Calibration will be eresed in FLASH\r\n");
     Term_Update(BufferToWrite,BytesToWrite);
  } else {
    ALLMEMS1_PRINTF("Magneto Calibration will be eresed in FLASH\r\n");
  }
  
  MDM_SaveGMD(GMD_MAG_CALIBRATION,(void *)&MagCalibrationData);

  NecessityToSaveMetaDataManager=1;
  return Success;
}

/**
 * @brief  Check if there are a valid Node Name Values in Memory and read them
 * @param  None
 * @retval unsigned char Success/Not Success
 */
static unsigned char ReCallNodeNameFromMemory(void)
{
  const char DefaultBoardName[7] = {NAME_BLUEMS};
  
  /* ReLoad the Node Name Values from RAM */
  unsigned char Success=0;

  /* Recall the node name Credential saved */
  MDM_ReCallGMD(GMD_NODE_NAME,(void *)&NodeName);
  
  if(NodeName[0] != 0x12)
  {
    NodeName[0]= 0x12;
    
    for(int i=0; i<7; i++)
    {
      NodeName[i+1]= DefaultBoardName[i];
      BlueNRG_StackValue.BoardName[i]= DefaultBoardName[i];
    }
    
    MDM_SaveGMD(GMD_NODE_NAME,(void *)&NodeName);
    NecessityToSaveMetaDataManager=1;
  }
  else
  {
    for(int i=0; i<7; i++)
      BlueNRG_StackValue.BoardName[i]= NodeName[i+1];
  }

  return Success;
}

/* Code for MotionTL integration - Start Section */
/**
 * @brief  Test if calibration data are available
 * @param  None
 * @retval None
 */
static void AccCalibTest(void)
{
  if(AccCalibrationData[0]== CHECK_CALIBRATION) {
    if(AccCalibrationData[1] == TargetBoardFeatures.mems_expansion_board) {
      for (int i = 0; i <= 2; i++)
      {
        AccelerometerCalibrationValue.offset[i]= AccCalibrationData[i + 2];
        AccelerometerCalibrationValue.gain[i]=   AccCalibrationData[i + 2 + 3];
      }
      
      AccelleroCalibrationDone =1;
        
        ALLMEMS1_PRINTF("\t--> Accelerometer Calibration Read\r\n");
    } else {
      ALLMEMS1_PRINTF("\t--> Accelerometer Calibration Not correct for Current %s board\r\n",TargetBoardFeatures.mems_expansion_board ? "IKS01A3" : "IKS01A2");
      ResetAccellerometerCalibrationInMemory();     
      AccelleroCalibrationDone=0;
    }
  } else {
    ALLMEMS1_PRINTF("\t--> Accelerometer Calibration Not present\r\n");
    AccelleroCalibrationDone=0;
  }
  
  if(!AccelleroCalibrationDone)
  {
    for (int i = 0; i <= 2; i++)
    {
      AccelerometerCalibrationValue.offset[i]= 0.0f;
      AccelerometerCalibrationValue.gain[i]=   1.0f;
    }
  }
}
/* Code for MotionTL integration - End Section */

/**
 * @brief  Check if there are a valid Accellerometer Calibration Values in Memory and read them
 * @param  uint16_t dataSize
 * @param  uint32_t *data
 * @retval uint32_t Success/Not Success
 */
unsigned char ReCallAccellerometerCalibrationFromMemory(uint16_t dataSize, uint32_t *data)
{
  /* ReLoad the Accellerometer Calibration Values from RAM */
  uint32_t Success=1;

  int i;
  
  /* Recall the accellerometer calibration Credential saved */
  if(MDM_ReCallGMD(GMD_ACC_CALIBRATION,(void *)&AccCalibrationData))
  {
    for(i=0; i<dataSize; i++)
    {
      data[i]= AccCalibrationData[i+2];
    }
    
    Success= 0;
  }

  return Success;
}

/**
 * @brief  Save the Accellerometer Calibration Values to Memory
 * @param  uint16_t dataSize
 * @param  uint32_t *data
 * @retval uint32_t Success/Not Success
 */
unsigned char SaveAccellerometerCalibrationToMemory(uint16_t dataSize, uint32_t *data)
{
  uint32_t Success=1;
  
  int i;

  /* Store in RAM */
  AccCalibrationData[0] = CHECK_CALIBRATION;
  AccCalibrationData[1] = TargetBoardFeatures.mems_expansion_board;

  for(i=0; i<dataSize; i++)
  {
    AccCalibrationData[i+2]= data[i];
  }
  
  if(MDM_SaveGMD(GMD_ACC_CALIBRATION,(void *)&AccCalibrationData))
  {
    if(BLE_StdTerm_Service==BLE_SERV_ENABLE){
     BytesToWrite = sprintf((char *)BufferToWrite, "Accellerometer Calibration will be saved in FLASH\r\n");
     Term_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Accellerometer Calibration will be saved in FLASH\r\n");
    }
    
    NecessityToSaveMetaDataManager=1;
    
    Success= 0;
  }

  return Success;
}

/**
 * @brief  Reset the Accellerometer Calibration Values in Memory
 * @param  None
 * @retval unsigned char Success/Not Success
 */
unsigned char ResetAccellerometerCalibrationInMemory(void)
{
  /* Reset Calibration Values in RAM */
  unsigned char Success=1;
  int32_t Counter;

  for(Counter=0;Counter<8;Counter++)
    AccCalibrationData[Counter]=0x0;

  if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
     BytesToWrite = sprintf((char *)BufferToWrite, "Accellerometer Calibration will be eresed in FLASH\r\n");
     Term_Update(BufferToWrite,BytesToWrite);
  } else {
    ALLMEMS1_PRINTF("Accellerometer Calibration will be eresed in FLASH\r\n");
  }
  
  MDM_SaveGMD(GMD_ACC_CALIBRATION,(void *)&AccCalibrationData);

  NecessityToSaveMetaDataManager=1;
  return Success;
}

/**********************************/
/* Characteristics Notify Service */
/**********************************/

/**
 * @brief  This function is called when there is a change on the gatt attribute for Inertial (Acc/Gyro/Mag plot)
 *         for Start/Stop Timer
 * @param  None
 * @retval None
 */
static void Inertial_StartStopTimer(void)
{
  if( (BLE_Inertial_NotifyEvent == BLE_NOTIFY_SUB) &&
      (!InertialTimerEnabled) ){
        /* Start the TIM Base generation in interrupt mode */
        if(HAL_TIM_Base_Start_IT(&TimInertialHandle) != HAL_OK){
          /* Starting Error */
          Error_Handler();
        }
        
    InertialTimerEnabled= 1;
  }
  
  if( (BLE_Inertial_NotifyEvent == BLE_NOTIFY_UNSUB) &&
      (InertialTimerEnabled) ){
        /* Stop the TIM Base generation in interrupt mode */
        if(HAL_TIM_Base_Stop_IT(&TimInertialHandle) != HAL_OK){
          /* Stopping Error */
          Error_Handler();
        }
    
    InertialTimerEnabled= 0;
  }
}

/**
 * @brief  This function is called when there is a change on the gatt attribute for Environmental
 *         for Start/Stop Timer
 * @param  None
 * @retval None
 */
static void Environmental_StartStopTimer(void)
{
  if( (BLE_Env_NotifyEvent == BLE_NOTIFY_SUB) &&
      (!EnvironmentalTimerEnabled) ){
        /* Start the TIM Base generation in interrupt mode */
        if(HAL_TIM_Base_Start_IT(&TimEnvHandle) != HAL_OK){
          /* Starting Error */
          Error_Handler();
        }
        
    EnvironmentalTimerEnabled= 1;
  }
  
  if( (BLE_Env_NotifyEvent == BLE_NOTIFY_UNSUB) &&
      (EnvironmentalTimerEnabled) ){
        /* Stop the TIM Base generation in interrupt mode */
        if(HAL_TIM_Base_Stop_IT(&TimEnvHandle) != HAL_OK){
          /* Stopping Error */
          Error_Handler();
        }
    
    EnvironmentalTimerEnabled= 0;
  }
}

/**
  * @brief  Enable/Disable Accelerometer events 
  * @param  None
  * @retval None
  */
static void AccEnv_StartStop(void)
{
  if( (BLE_AccEnv_NotifyEvent == BLE_NOTIFY_SUB) &&
      (!AccEventEnabled) ){
        EnableHWMultipleEvents();
        ResetHWPedometer();
        Config_Update(FEATURE_MASK_ACC_EVENTS,'m',1);
        AccEventEnabled= 1;
  }
  
  if( (BLE_AccEnv_NotifyEvent == BLE_NOTIFY_UNSUB) &&
      (AccEventEnabled) ){
        DisableHWMultipleEvents();
        AccEventEnabled= 0;
  }
}

/**
 * @brief  This function is called when there is a change on the gatt attribute for Audio Level
 *         for Start/Stop Timer
 * @param  None
 * @retval None
 */
static void AudioLevel_StartStopTimer(void)
{
  if( (BLE_AudioLevel_NotifyEvent == BLE_NOTIFY_SUB) &&
      (!AudioLevelTimerEnabled) ){
        int32_t Count;
      
        InitMics(AUDIO_IN_SAMPLING_FREQUENCY, AUDIO_VOLUME_INPUT, PCM_AUDIO_IN_SAMPLES);
    
        for(Count=0;Count<AUDIO_IN_CHANNELS;Count++) {
          RMS_Ch[Count]=0;
          DBNOISE_Value_Old_Ch[Count] =0;
        }
    
        /* Start the TIM Base generation in interrupt mode */
        if(HAL_TIM_Base_Start_IT(&TimAudioDataHandle) != HAL_OK){
          /* Starting Error */
          Error_Handler();
        }
      
        AudioLevelTimerEnabled= 1;
  }
  
  if( (BLE_AudioLevel_NotifyEvent == BLE_NOTIFY_UNSUB) &&
      (AudioLevelTimerEnabled) ){
        DeInitMics();
        
        /* Stop the TIM Base generation in interrupt mode */
        if(HAL_TIM_Base_Stop_IT(&TimAudioDataHandle) != HAL_OK){
          /* Stopping Error */
          Error_Handler();
        }  
    
        AudioLevelTimerEnabled= 0;
  }
}

/**
 * @brief  This function is called when there is a change on the gatt attribute for Audio Source Localization
 *         for Start/Stop Timer
 * @param  None
 * @retval None
 */
static void AudioSourceLocalization_StartStopTimer(void)
{
  if( (BLE_AudioSourceLocalization_NotifyEvent == BLE_NOTIFY_SUB) &&
      (!AudioSourceLocalizationTimerEnabled) ){
      
        InitMics(AUDIO_IN_SAMPLING_FREQUENCY, AUDIO_VOLUME_INPUT, PCM_AUDIO_IN_SAMPLES);
    
        /* Start the TIM Base generation in interrupt mode */
        if(HAL_TIM_Base_Start_IT(&TimAudioDataHandle) != HAL_OK){
          /* Starting Error */
          Error_Handler();
        }
      
        AudioSourceLocalizationTimerEnabled= 1;
  }
  
  if( (BLE_AudioSourceLocalization_NotifyEvent == BLE_NOTIFY_UNSUB) &&
      (AudioSourceLocalizationTimerEnabled) ){
        DeInitMics();
        
        /* Stop the TIM Base generation in interrupt mode */
        if(HAL_TIM_Base_Stop_IT(&TimAudioDataHandle) != HAL_OK){
          /* Stopping Error */
          Error_Handler();
        }  
    
        AudioSourceLocalizationTimerEnabled= 0;
  }
}

/**
  * @brief  Enable/Disable TIM1 Channel 1 
  * @param  None
  * @retval None
  */
static void TIM1_CHANNEL_1_StartStop(void)
{
  if( ((BLE_SensorFusion_NotifyEvent == BLE_NOTIFY_SUB) || (BLE_ECompass_NotifyEvent == BLE_NOTIFY_SUB)) &&
      (!TIM1_CHANNEL_1_Enabled) ){
        
    /* Get ODR accelerometer default Value */
    MOTION_SENSOR_GetOutputDataRate(ACCELERO_INSTANCE,MOTION_ACCELERO,&UsedAccelerometerDataRate);
    /* Set ODR accelerometer: >= 100 Hz*/
    MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE, MOTION_ACCELERO, (float)100.0f);
    /* Set FS accelerometer: = <-4g, 4g> */
    Set4GAccelerometerFullScale();
    
    /* Start the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
      /* Starting Error */
      Error_Handler();
    }

    /* Set the new Capture compare value */
    {
      uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
      /* Set the Capture Compare Register value */
      __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + uhCCR1_Val));
    }
    
    TIM1_CHANNEL_1_Enabled= 1;
    
    if(BLE_SensorFusion_NotifyEvent == BLE_NOTIFY_SUB)
      SensorFusionEnabled= 1;
    
    if(BLE_ECompass_NotifyEvent == BLE_NOTIFY_SUB)
      ECompassEnabled= 1;
  }
  
  if( ((BLE_SensorFusion_NotifyEvent == BLE_NOTIFY_UNSUB) || (BLE_ECompass_NotifyEvent == BLE_NOTIFY_UNSUB)) &&
      (TIM1_CHANNEL_1_Enabled) ){
    /* Stop the TIM Base generation in interrupt mode (for Acc/Gyro/Mag sensor) */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }      
    
    /* Set default ODR accelerometer value */
    MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE, MOTION_ACCELERO, UsedAccelerometerDataRate);
    /* Set default FS accelerometer: = <-2g, 2g> */
    Set2GAccelerometerFullScale();
    
    TIM1_CHANNEL_1_Enabled= 0;
    SensorFusionEnabled= 0;
    ECompassEnabled= 0;
  }
}

/**
  * @brief  Enable/Disable TIM1 Channel 2 
  * @param  None
  * @retval None
  */
static void TIM1_CHANNEL_2_StartStop(void)
{
  if( ((BLE_CarryPosition_NotifyEvent == BLE_NOTIFY_SUB) ||
       (BLE_GestureRecognition_NotifyEvent == BLE_NOTIFY_SUB) ||
       (BLE_TiltSensing_NotifyEvent == BLE_NOTIFY_SUB) ||
       (BLE_MotionAlgorithms_VC_NotifyEvent == BLE_NOTIFY_SUB)) &&
      (!TIM1_CHANNEL_2_Enabled) ){
        
    /* Get ODR accelerometer default Value */
    MOTION_SENSOR_GetOutputDataRate(ACCELERO_INSTANCE,MOTION_ACCELERO,&UsedAccelerometerDataRate);
    /* Set ODR accelerometer: >= 50 Hz*/
    MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE, MOTION_ACCELERO, 50.0f);
    /* Set FS accelerometer: = <-4g, 4g> */
    Set4GAccelerometerFullScale();
    
    if(BLE_MotionAlgorithms_VC_NotifyEvent == BLE_NOTIFY_SUB) {
      /* Get ODR pressure sensor default Value */
      ENV_SENSOR_GetOutputDataRate(PRESSURE_INSTANCE, ENV_PRESSURE, &UsedPressureDataRate);
      /* Set ODR pressure sensor: >= 25 Hz*/
      ENV_SENSOR_SetOutputDataRate(PRESSURE_INSTANCE, ENV_PRESSURE, 25.0f);
    }
    
    
    /* Start the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_2) != HAL_OK){
      /* Starting Error */
      Error_Handler();
    }

    /* Set the new Capture compare value */
    {
      uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
      /* Set the Capture Compare Register value */
      __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_2, (uhCapture + uhCCR2_Val));
    }
    
    TIM1_CHANNEL_2_Enabled=1;
      
    if(BLE_CarryPosition_NotifyEvent == BLE_NOTIFY_SUB)
      CarryPositionEnabled= 1;
    if(BLE_GestureRecognition_NotifyEvent == BLE_NOTIFY_SUB)
      GestureRecognitionEnabled= 1;
     if(BLE_TiltSensing_NotifyEvent == BLE_NOTIFY_SUB)
      TiltSensingEnabled= 1;
    if(BLE_MotionAlgorithms_VC_NotifyEvent == BLE_NOTIFY_SUB)
      VerticalContextEnabled= 1;
  }
  
  if( ((BLE_CarryPosition_NotifyEvent == BLE_NOTIFY_UNSUB) ||
       (BLE_GestureRecognition_NotifyEvent == BLE_NOTIFY_UNSUB) ||
       /*(BLE_TiltSensing_NotifyEvent == BLE_NOTIFY_UNSUB) ||*/
       (BLE_MotionAlgorithms_VC_NotifyEvent == BLE_NOTIFY_UNSUB)) &&
      (TIM1_CHANNEL_2_Enabled) ){
    /* Stop the TIM Base generation in interrupt mode (for Acc/Gyro/Mag sensor) */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_2) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }      
    
    /* Set default ODR accelerometer value */
    MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE, MOTION_ACCELERO, UsedAccelerometerDataRate);
    /* Set default FS accelerometer: = <-2g, 2g> */
    Set2GAccelerometerFullScale();
    
    if(BLE_MotionAlgorithms_VC_NotifyEvent == BLE_NOTIFY_UNSUB) {
      /* Set default ODR pressure sensor value*/
      ENV_SENSOR_SetOutputDataRate(PRESSURE_INSTANCE, ENV_PRESSURE, UsedPressureDataRate);
    }
    
    TIM1_CHANNEL_2_Enabled=0;
    CarryPositionEnabled= 0;
    GestureRecognitionEnabled= 0;
    TiltSensingEnabled= 0;
    VerticalContextEnabled= 0;
  }
}


/**
  * @brief  Enable/Disable TIM1 Channel 3 
  * @param  None
  * @retval None
  */
static void TIM1_CHANNEL_3_StartStop(void)
{
  if( ((BLE_ActRec_NotifyEvent == BLE_NOTIFY_SUB) ||
       (BLE_MotionIntensity_NotifyEvent == BLE_NOTIFY_SUB) ||
       (BLE_MotionAlgorithms_PE_NotifyEvent == BLE_NOTIFY_SUB)) &&
      (!TIM1_CHANNEL_3_Enabled) ){
        
    /* Get ODR accelerometer default Value */
    MOTION_SENSOR_GetOutputDataRate(ACCELERO_INSTANCE,MOTION_ACCELERO,&UsedAccelerometerDataRate);
    /* Set ODR accelerometer: >= 16 Hz*/
    MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE, MOTION_ACCELERO, 16.0f);
    /* Set FS accelerometer: = <-4g, 4g> */
    Set4GAccelerometerFullScale();
    
    /* Start the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_3) != HAL_OK){
      /* Starting Error */
      Error_Handler();
    }

    /* Set the new Capture compare value */
    {
      uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
      /* Set the Capture Compare Register value */
      __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_3, (uhCapture + uhCCR3_Val));
    }
    
    Set4GAccelerometerFullScale();
    
    TIM1_CHANNEL_3_Enabled= 1;
    
    if(BLE_ActRec_NotifyEvent == BLE_NOTIFY_SUB)
      ActivityRecognitionEnabled= 1;
    
    if(BLE_MotionIntensity_NotifyEvent == BLE_NOTIFY_SUB)
      MotionIntensityEnabled= 1;
    
    if(BLE_MotionAlgorithms_PE_NotifyEvent == BLE_NOTIFY_SUB)
    {
      PoseEstimationEnabled= 1;
    }
  }
  
  if( ((BLE_ActRec_NotifyEvent == BLE_NOTIFY_UNSUB) ||
       (BLE_MotionIntensity_NotifyEvent == BLE_NOTIFY_UNSUB) ||
       (BLE_MotionAlgorithms_PE_NotifyEvent == BLE_NOTIFY_UNSUB)) &&
      (TIM1_CHANNEL_3_Enabled) ){
    /* Stop the TIM Base generation in interrupt mode (for Acc/Gyro/Mag sensor) */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_3) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }      
    
    /* Set default ODR accelerometer value */
    MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE, MOTION_ACCELERO, UsedAccelerometerDataRate);
    /* Set default FS accelerometer: = <-2g, 2g> */
    Set2GAccelerometerFullScale();
    
    TIM1_CHANNEL_3_Enabled= 0;
    ActivityRecognitionEnabled= 0;
    MotionIntensityEnabled= 0;
    PoseEstimationEnabled= 0;
  }
}

/**
  * @brief  Enable/Disable TIM1 Channel 4 
  * @param  None
  * @retval None
  */
static void TIM1_CHANNEL_4_StartStop(void)
{ 
  if( ((BLE_MotionAlgorithms_SD_NotifyEvent == BLE_NOTIFY_SUB) ||
       (BLE_FitnessActivities_NotifyEvent == BLE_NOTIFY_SUB)) &&
      (!TIM1_CHANNEL_4_Enabled) ){
        
    /* Get ODR accelerometer default Value */
    MOTION_SENSOR_GetOutputDataRate(ACCELERO_INSTANCE,MOTION_ACCELERO,&UsedAccelerometerDataRate);
    /* Set ODR accelerometer: >= 25 Hz*/
    MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE, MOTION_ACCELERO, 25.0f);
    
    if(BLE_FitnessActivities_NotifyEvent == BLE_NOTIFY_SUB)
    {
      /* Get ODR gyroscope default Value */
      MOTION_SENSOR_GetOutputDataRate(ACCELERO_INSTANCE, MOTION_GYRO,&UsedGyroscopeDataRate);
      /* Set ODR gyroscope: >= 25 Hz*/
      MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE, MOTION_GYRO, 25.0f);
    }

    /* Get ODR pressure sensor default Value */
    ENV_SENSOR_GetOutputDataRate(PRESSURE_INSTANCE, ENV_PRESSURE, &UsedPressureDataRate);
    /* Set ODR pressure sensor: >= 25 Hz*/
    ENV_SENSOR_SetOutputDataRate(PRESSURE_INSTANCE, ENV_PRESSURE, 25.0f);

    /* Start the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_4) != HAL_OK){
      /* Starting Error */
      Error_Handler();
    }

    /* Set the new Capture compare value */
    {
      uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
      /* Set the Capture Compare Register value */
      __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_4, (uhCapture + uhCCR4_Val));
    }
    
    TIM1_CHANNEL_4_Enabled= 1;
    
    if(BLE_MotionAlgorithms_SD_NotifyEvent == BLE_NOTIFY_SUB)
    {
      Standing_VS_SittingDeskEnabled= 1;
    }
    
    #ifdef ALLMEMS1_MOTIONFA
    if(BLE_FitnessActivities_NotifyEvent == BLE_NOTIFY_SUB)
    {
      MotionFA_manager_reset_counter();
      FitnessActivitiesEnabled= 1;
    }
    #endif /* ALLMEMS1_MOTIONFA */
  }
  
  if( ((BLE_MotionAlgorithms_SD_NotifyEvent == BLE_NOTIFY_UNSUB) ||
       (BLE_FitnessActivities_NotifyEvent == BLE_NOTIFY_UNSUB)) &&
      (TIM1_CHANNEL_4_Enabled) ){
    /* Stop the TIM Base generation in interrupt mode (for Acc/Gyro/Mag sensor) */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_4) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }
    
    /* Set default ODR accelerometer */
    MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE, MOTION_ACCELERO, UsedAccelerometerDataRate);

    if(BLE_FitnessActivities_NotifyEvent == BLE_NOTIFY_UNSUB)
    {
      /* Set default ODR gyroscope */
      MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE, MOTION_GYRO, UsedGyroscopeDataRate);
    }
    
    /* Set default ODR pressure sensor */
    ENV_SENSOR_SetOutputDataRate(PRESSURE_INSTANCE, ENV_PRESSURE, UsedPressureDataRate);

    TIM1_CHANNEL_4_Enabled= 0;
    Standing_VS_SittingDeskEnabled= 0;
    FitnessActivitiesEnabled= 0;
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

