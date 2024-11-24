/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  *
  *			Blue_color_Discovery__STM32F407
  *
  *				NodeID = 0x3b
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CO_app_STM32.h"
#include "301/CO_SDOclient.h"
#include "CANopen.h"
#include "OD.h"

#include "format_out.h"
#include "SDO_utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// #define ADC_SAMPLES 10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim14;
DMA_HandleTypeDef hdma_tim8_up;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

DMA_HandleTypeDef hdma_memtomem_dma2_stream0;
DMA_HandleTypeDef hdma_memtomem_dma2_stream5;
DMA_HandleTypeDef hdma_memtomem_dma2_stream3;
/* USER CODE BEGIN PV */

RTC_DateTypeDef DateToUpdate = {0};
RTC_TimeTypeDef sTime = {0};
//					    0,//uint8_t Hours; Max_Data=12 if the RTC_HourFormat_12; Max_Data=23 if the RTC_HourFormat_24
//						0,//uint8_t Minutes; Max_Data = 59
//						0,//uint8_t Seconds; Max_Data = 59 */
//						0,//uint8_t TimeFormat;Specifies the RTC AM/PM Time.
//						0,//uint32_t SecondFraction;parameter corresponds to a time unit range between [0-1] Second with [1 Sec / SecondFraction +1] granularity
//						0,//uint32_t DayLightSaving;  This interface is deprecated.
//						0//uint32_t StoreOperation;

uint8_t Tx_Array[16]={0x51,0x62,0x73,0x84,0x55,0x46,0x87,0x18,0x29,0x10,0x11,0x12,0x13,0x14,0x15,0x33};
uint8_t Rx_Array[16]={0};
uint32_t Array_32u[16]={0};
uint8_t Array_8u[16]={0x54,0x34,0x21,0xea,0xf3,0x7a,0xd4,0x46};
char Message_to_Terminal[128]={};
char Message_to_Terminal_1[128]={};
char Message_to_Terminal_2[128]={};
char Message_to_Terminal_3[128]={};
uint8_t Array_from_Terminal[128]={0};
uint8_t Length_of_Message;
uint8_t Length_of_Ext_Var=0;
uint8_t Local_Count=0;
uint64_t Count_of_while1=0;
float ChipTemperature;

CANopenNodeSTM32    canOpenNodeSTM32;
CO_SDO_abortCode_t  Code_return_SDO;
CAN_TxHeaderTypeDef Tx_Header;
uint32_t            TxMailbox;
uint32_t            tmp32u_1   = 0x1e1f1a1b;
uint32_t            tmp32u_0   = 0x0e0f0a0b;
uint64_t            tmp64u_0   = 0x0e1f1a1b56789a;
uint64_t            tmp64u_1   = 0x0e1f1a1b56789a;
uint32_t            Ticks;
char String_H[]={"String_for_Test_UART_"};
char String_L[]={"String_for_Test_UART_"};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_CAN1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

void CAN_interface_Test(void);
void UART_interface_Test(void);
void Board_Name_to_Terminal(void);
void CO_Init_Return_State(uint16_t Returned_Code);
void SDO_abortCode_ASCII_to_Terminal(void);
CO_SDO_abortCode_t SDO_Read_Write_Read(void);
void TPDO_send(uint32_t Period_ms);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Timer interrupt function executes every 1 ms */
void
HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == canopenNodeSTM32->timerHandle) {
        canopen_app_interrupt();
    }
}


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
  MX_DMA_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_CAN1_Init();
  MX_RTC_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  /* CANHandle : Pass in the CAN Handle to this function and it wil be used for all CAN Communications. It can be FDCan or CAN
   * and CANOpenSTM32 Driver will take of care of handling that
   * HWInitFunction : Pass in the function that initialize the CAN peripheral, usually MX_CAN_Init
   * timerHandle : Pass in the timer that is going to be used for generating 1ms interrupt for tmrThread function,
   * please note that CANOpenSTM32 Library will override HAL_TIM_PeriodElapsedCallback function, if you also need this function
   * in your codes, please take required steps
   * desiredNodeID : This is the Node ID that you ask the CANOpen stack to assign to your device, although it might not always
   * be the final NodeID, after calling canopen_app_init() you should check ActiveNodeID of CANopenNodeSTM32 structure for assigned Node ID.
   * baudrate: This is the baudrate you've set in your CubeMX Configuration
   *
   */

//   GPIO_Blink_Test(GPIOA, GPIO_PIN_7|GPIO_PIN_6, 25, 33); 						// for_STM32F4XX_Ali_pcb
    GPIO_Blink_Test(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, 25, 33);// blink_at_Discovery_EVB
//    UART_interface_Test(); //while(1){;}
//    CAN_interface_Test();

	HAL_TIM_Base_Start_IT(&htim8);
	extern uint16_t adc_buffer[];
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, ADC_SAMPLES * 2 * 2);
    HAL_UART_Receive_DMA(&huart2, Array_from_Terminal, sizeof Array_from_Terminal );
    HAL_Delay(2500);
    Board_Name_to_Terminal();


	HAL_TIM_Base_Start_IT(&htim4);

//	CANopenNodeSTM32 canOpenNodeSTM32;
	canOpenNodeSTM32.CANHandle = &hcan1;
	canOpenNodeSTM32.HWInitFunction = MX_CAN1_Init;
	canOpenNodeSTM32.timerHandle = &htim4;
	canOpenNodeSTM32.desiredNodeID = CO_Disco407_Blue;//0x3b;
	canOpenNodeSTM32.baudrate = 125*4;
uint16_t Ret_value = canopen_app_init(&canOpenNodeSTM32);
	CO_Init_Return_State(Ret_value );

	 SDO_Read_Write_Read();

		  OD_PERSIST_COMM.x6000_disco_Blue_VAR32_6000_TX=0;
		  Local_Count=0;



//		    if (CO_GET_CNT(LSS_MST) == 1U) {
//		        err = CO_LSSmaster_init(co->LSSmaster, CO_LSSmaster_DEFAULT_TIMEOUT, co->CANmodule, CO_GET_CO(RX_IDX_LSS_MST),
//		                                CO_CAN_ID_LSS_SLV, co->CANmodule, CO_GET_CO(TX_IDX_LSS_MST), CO_CAN_ID_LSS_MST);
//		        if (err != CO_ERROR_NO) {
//		            return err;
//		        }
//		    }



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



		  while (1)
		  {
		   RTC_update_and_Terminal(1999);
		  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, !canOpenNodeSTM32.outStatusLEDRed  );
		  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, !canOpenNodeSTM32.outStatusLEDGreen);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, !canOpenNodeSTM32.outStatusLEDGreen);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, !canOpenNodeSTM32.outStatusLEDRed  );

		  canopen_app_process();
#if 4
			if(tmp32u_0 != OD_PERSIST_COMM.x6001_disco_Blue_VAR32_6001_R)
			{
			tmp32u_0 = OD_PERSIST_COMM.x6001_disco_Blue_VAR32_6001_R;
			while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
		    Length_of_Message = sprintf( Message_to_Terminal,
		  	  	                     "copy  OD_PERSIST_COMM.x6001_disco_Blue_VAR32_6001_R "
                  	  	  	  	  	  " to tmp32u_0 = 0x%X%X\n\r",(uint16_t)(tmp32u_0>>16),(uint16_t)tmp32u_0);
		    //HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal, Length_of_Message);
			}
#endif	//4

			if(tmp32u_1 != OD_PERSIST_COMM.x6002_disco_Blue_VAR32_6002_R)
			{
			Length_of_Message = sprintf( Message_to_Terminal,
		                "check for updates tmp32u_1 = OD_PERSIST_COMM.x6002_disco_Blue_VAR32_6002_R"
		                "tmp32u_1 = 0x%X%X\n\r",(uint16_t)(tmp32u_1>>16),(uint16_t)tmp32u_1);
				while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
			//HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal, Length_of_Message);
			}

			Count_of_while1++;//Counter_of_while_cycles

   if(Count_of_while1==2000)
   {
	   uint32_t Local_var;
//	   CO_LSSmaster_t My_CO_LSSmaster;
//	   CO_ReturnError_t Code_Err;
//	   Code_Err = CO_LSSmaster_init(
//			   	   	   	   	   	   My_CO_LSSmaster,
//								   10,
//								   CANdevRx,
//								   CANdevRxIdx,
//								   CANidLssSlave,
//								   CANdevTx,
//								   CANdevTxIdx,
//								   CANidLssMaster);

	   Local_var = canOpenNodeSTM32.canOpenStack->LSSmaster->timeoutTimer;

	   //Local_var = CO_GET_CNT(LSS_MST);
	   while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
		Length_of_Message = sprintf( Message_to_Terminal,
	                				"\n\r\n\rcanOpenNodeSTM32.canOpenStack->LSSmaster->timeoutTimer = 0x%X%X\n\r\n\r",
									(uint16_t)(Local_var>>16),(uint16_t)Local_var);
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal, Length_of_Message);
	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}

   }
     TPDO_send(1700); /// uint32_t Period_ms

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x13;
  sTime.Minutes = 0x10;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
  sDate.Month = RTC_MONTH_NOVEMBER;
  sDate.Date = 0x20;
  sDate.Year = 0x24;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm B
  */
  sAlarm.Alarm = RTC_ALARM_B;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  htim4.Init.Prescaler = 167;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 167;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 41;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 39999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 167;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_2;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_2;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream0
  *   hdma_memtomem_dma2_stream5
  *   hdma_memtomem_dma2_stream3
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_stream0 on DMA2_Stream0 */
  hdma_memtomem_dma2_stream0.Instance = DMA2_Stream0;
  hdma_memtomem_dma2_stream0.Init.Channel = DMA_CHANNEL_0;
  hdma_memtomem_dma2_stream0.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream0.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_memtomem_dma2_stream0.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_memtomem_dma2_stream0.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream0.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  hdma_memtomem_dma2_stream0.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream0.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream0.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream0.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream0) != HAL_OK)
  {
    Error_Handler( );
  }

  /* Configure DMA request hdma_memtomem_dma2_stream5 on DMA2_Stream5 */
  hdma_memtomem_dma2_stream5.Instance = DMA2_Stream5;
  hdma_memtomem_dma2_stream5.Init.Channel = DMA_CHANNEL_0;
  hdma_memtomem_dma2_stream5.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream5.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream5.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream5.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_memtomem_dma2_stream5.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_memtomem_dma2_stream5.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream5.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_memtomem_dma2_stream5.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream5.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream5.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream5.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream5) != HAL_OK)
  {
    Error_Handler( );
  }

  /* Configure DMA request hdma_memtomem_dma2_stream3 on DMA2_Stream3 */
  hdma_memtomem_dma2_stream3.Instance = DMA2_Stream3;
  hdma_memtomem_dma2_stream3.Init.Channel = DMA_CHANNEL_0;
  hdma_memtomem_dma2_stream3.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream3.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream3.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream3.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_memtomem_dma2_stream3.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_memtomem_dma2_stream3.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream3.Init.Priority = DMA_PRIORITY_MEDIUM;
  hdma_memtomem_dma2_stream3.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream3.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream3.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream3.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream3) != HAL_OK)
  {
    Error_Handler( );
  }

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  //USB_Reset();
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

////////////////////////////////////////////////////////////
void CAN_interface_Test(void)
{
 Tx_Header.IDE    = CAN_ID_STD;
 Tx_Header.ExtId  = 0;
 Tx_Header.DLC    = 8;
 Tx_Header.StdId  = 0x7EC;
 Tx_Header.RTR    = CAN_RTR_DATA;
 HAL_CAN_Start(&hcan1);  HAL_Delay(1500);

 if(HAL_CAN_AddTxMessage( &hcan1,
   		               	  &Tx_Header,
						  Tx_Array, &TxMailbox )==HAL_OK
   )
	  {  /* Wait transmission complete */
	  //while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) {}
		  for(uint8_t cnt=0;cnt<22;cnt++)
		  {
		   HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);//LED2_Pin//yellow
		   HAL_Delay(46);
		  }
	  }
}

///////////////////////////////////////////////////
void UART_interface_Test(void)
{
	// Test_Terminal__ASCII
	  Length_of_Message = sprintf( Message_to_Terminal,
			  	  	  	  	  	  	  "Rx_Array[0]=0x%x, Rx_Array[1]= 0x%x, Rx_Array[2]= 0x%x, Rx_Array[3]= 0x%x \n\r",
									   Rx_Array[0],Rx_Array[1],Rx_Array[2],Rx_Array[3]
								 );
	  TerminalInterface.gState = HAL_UART_STATE_READY;
//	  HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal, Length_of_Message);
	  HAL_UART_Transmit( &TerminalInterface, (uint8_t*)Message_to_Terminal, Length_of_Message,50);

//    Test_Terminal__HEX
//
//	  HAL_Delay(500);
//	  Local_Count = sizeof String_L;
//	  String_L[Local_Count-1] = 0x0d;
//	  TerminalInterface.gState = HAL_DMA_STATE_READY;
//	  HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)(String_L), Local_Count);

}


//////////////////////////////////////////////////
void Board_Name_to_Terminal(void)
{
	const char Message_0[]={"   ******************************************\n\r"};
//	const char Message_1[]={"*  Upper Blackboard  STM32F4XX___Ali     *\n\r"};
//	const char Message_1[]={"*  Lower Blackboard  STM32F4XX___Ali     *\n\r"};
//  const char Message_1[]={"*  STM32F4DISCOVERY Green_board China    *\n\r"};
	const char Message_1[]={"*  STM32F4DISCOVERY Blue_board Original  *\n\r"};
//  const char Message_1[]={"*  STM32F4DISCOVERY Green_board Original *\n\r"};
	char Array_for_Messages[128]={};
	uint16_t Msg_Length;
//	uint32_t Chip_ID_96bit[4]={};
//	uint16_t  *pChip_ID_96bit =(uint16_t*)Chip_ID_96bit ;

//	Chip_ID_96bit[0] = HAL_GetUIDw0();
//	Chip_ID_96bit[1] = HAL_GetUIDw1();
//	Chip_ID_96bit[2] = HAL_GetUIDw2();



//	 uint8_t Sekunden=0;
//	 uint8_t Minuten=0;
//	 uint8_t Uhr=0;

	Msg_Length = sizeof(Message_0);
	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_0, Msg_Length);

	Msg_Length = sizeof(Message_1);
	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_1, Msg_Length);

	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	Msg_Length = sprintf( Array_for_Messages,
			  	  	  	  "*  SystemClock = %d MHz                 *\n\r",
						  (uint16_t)(HAL_RCC_GetSysClockFreq()/1000000)
						);
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Array_for_Messages, Msg_Length);

	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	Msg_Length = sprintf( Array_for_Messages,
			  	  	  	  "   *  Unical_ID %X%X%X%X%X%X        *\n\r",
						  (uint16_t)(HAL_GetUIDw2()>>16),(uint16_t)(HAL_GetUIDw2()),
						  (uint16_t)(HAL_GetUIDw1()>>16),(uint16_t)(HAL_GetUIDw1()),
						  (uint16_t)(HAL_GetUIDw0()>>16),(uint16_t)(HAL_GetUIDw0())
						);
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Array_for_Messages, Msg_Length);

	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	Msg_Length = sprintf( Array_for_Messages,
			  	  	  	  "   *  Device identifier %X%X                *\n\r",
						  (uint16_t)(HAL_GetDEVID()>>16), (uint16_t)(HAL_GetDEVID() & 0x0000FFFF)
						);
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Array_for_Messages, Msg_Length);

	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	Msg_Length = sprintf( Array_for_Messages,
			  	  	  	  "   *  Device revision identifier %X%X      *\n\r",
						  (uint16_t)( HAL_GetREVID()>>16 ),
						  (uint16_t)( HAL_GetREVID() & 0x0000FFFF )
						);
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Array_for_Messages, Msg_Length);


///////////////////////TimeStamp___DataStamp
	Get_Time();
	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	Get_Date();
	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}

///////////////Temperature //////////////////////////////////////////
	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	extern uint16_t adc_buffer[];
	ChipTemperature = (float)process_adc_buffer(adc_buffer);;
	Msg_Length = sprintf( Array_for_Messages,
			  	  	  	  "   *  Temperature %03.2f                      *\n\r",
						  ChipTemperature );
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Array_for_Messages, Msg_Length);

///**********************Asterics
	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	Msg_Length = sizeof(Message_0);
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_0, Msg_Length);

	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	Array_for_Messages[0]=0x0a;		Array_for_Messages[1]=0x0d;
	Array_for_Messages[2]=0x0a;		Array_for_Messages[3]=0x0d;
	Array_for_Messages[4]=0x0a;		Array_for_Messages[5]=0x0d;
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)(Array_for_Messages), 6);
	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}

}


//////////////////////////////////////////////////
void CO_Init_Return_State(uint16_t Returned_Code)
{
	uint16_t Lngth_of_Message;
	char Msg_2_Terminal[400];
//extern uint32_t *heapMemoryUsed;
	   Lngth_of_Message = sprintf( Msg_2_Terminal,
		  	  	  "   *  CANopenNodeSTM32 canOpenNodeSTM32;        *\n\r"
		  	  	  "   *  .CANHandle = &hcan1;                      *\n\r"
		  	  	  "   *  .HWInitFunction = MX_CAN1_Init            *\n\r"
		  	  	  "   *  .timerHandle = &htim4                     *\n\r"
		  	  	  "   *  .baudrate = 500kbps                       *\n\r"
		  	  	  "   *  .desiredNodeID = CO_Disco407_Blue;//0x3b; *\n\r"
		  	  	  "   *  canopen_app_init(&canOpenNodeSTM32);      * \n\r\n\r"
			   );
		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Msg_2_Terminal, Lngth_of_Message);
		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}

	   if (Returned_Code==0){
	     uint8_t Msg_0[128];// ="  canopen_app_init OK !\n\r\n\r";
	   //Lngth_of_Message = sizeof(Msg_0);
	   Lngth_of_Message = sprintf( (char*)Msg_0," canopen_app_init OK !\n\r"
			   "  Allocated.....bytes for CANopen objects\n\r");
	   //


	   	  while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	   	  HAL_UART_Transmit_DMA( &TerminalInterface, Msg_0, Lngth_of_Message);
	   	 }else if(Returned_Code==1) {
	   		const uint8_t Msg_1[]="Error: Can't allocate memory!\n\r\n\r";
	   		Lngth_of_Message = sizeof(Msg_1);
	   		 while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	   		  HAL_UART_Transmit_DMA( &TerminalInterface, Msg_1, Lngth_of_Message);
	   		 }else if(Returned_Code==2) {
	   			const uint8_t Msg_2[]="Error: Storage %d ! \n\r\n\r";
	   			Lngth_of_Message = sizeof(Msg_2);
	   			 while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	   			  HAL_UART_Transmit_DMA( &TerminalInterface, Msg_2, Lngth_of_Message);
	   			 }else{;}

		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
}

///////////////////////////////////////////////////
void SDO_abortCode_ASCII_to_Terminal(void)
{
char Message_2_Terminal_0[128];
uint16_t Message_Length;
while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
Message_Length = SDO_abortCode_to_String(Code_return_SDO,  Message_2_Terminal_0);
HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_2_Terminal_0, Message_Length);
while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
}

//////////////////////////////////////////


CO_SDO_abortCode_t SDO_Read_Write_Read(void)
{

	Code_return_SDO = read_SDO (
			    canOpenNodeSTM32.canOpenStack->SDOclient,
			  	0x3d,										//remote desiredNodeID Upper_F407XX
				0x6004,										//Index_of_OD_variable_at_remote_NodeID_6004_u64
				0x00,										//Sub_Index_of_OD_variable
				Rx_Array,									//Save_Received_Data_to Local_Array
				8,											//Number_of_Bytes_to_read
				(size_t*)&Length_of_Ext_Var ); HAL_Delay(100);


#if 1

		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
		Length_of_Message = sprintf( Message_to_Terminal,
		  	  	                     "\r EXECUTED read_SDO(...); for the first time\n\r "
                  "by canOpenNodeSTM32.canOpenStack->SDOclient\n\r");

		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal, Length_of_Message);HAL_Delay(10);

		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
		Length_of_Message = sprintf( Message_to_Terminal_3,
                "from RemoteNode=0x3d OD_Index=0x6004_u64 SubIndex=0x0\n\r and Save to \n\r"
				"Rx_Array={0x%X,0x%X,0x%X,0x%X,0x%X,0x%X,0x%X,0x%X} \n\r",
				Rx_Array[0],Rx_Array[1],Rx_Array[2],Rx_Array[3],
				Rx_Array[4],Rx_Array[5],Rx_Array[6],Rx_Array[7]);
		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal_3, Length_of_Message);
		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
		SDO_abortCode_ASCII_to_Terminal();		HAL_Delay(10);
#endif


#if 2
		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
		Length_of_Message = sprintf( Message_to_Terminal_2, "   \n\r Run write_SDO(.....);\n\r"
														  " write_SDO  by "
                  	  	  	  	  	  	  	  	  	  	  "canOpenNodeSTM32.canOpenStack->SDOclient\n\r");
		Message_to_Terminal_2[0]=0x08;
		Message_to_Terminal_2[1]=0x08;
		Message_to_Terminal_2[2]=0x08;
		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal_2, Length_of_Message);HAL_Delay(10);

		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
		Length_of_Message = sprintf( Message_to_Terminal_1,
                "Get Data \n\r from Local\n\r Array_8u={0x%X,0x%X,0x%X,0x%X,0x%X,0x%X,0x%X,0x%X} \n\r",
				Array_8u[0],Array_8u[1],Array_8u[2],Array_8u[3],
				Array_8u[4],Array_8u[5],Array_8u[6],Array_8u[7]);
		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal_1, Length_of_Message);
		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
		HAL_Delay(10);

		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
		Length_of_Message = sprintf( Message_to_Terminal_2,
                "and \n\r write to\n\r"
                "OD_Index=0x6004 SubIndex=0x0E  @ Remote Node_0x3d\n\r");
		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal_2, Length_of_Message);
		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
		HAL_Delay(10);

#endif//2

Code_return_SDO = write_SDO(
				canOpenNodeSTM32.canOpenStack->SDOclient,
				0x3d,										//remote desiredNodeID Upper_F407XX
				0x6004,										//Index_of_OD_variable_at_remote_NodeID_6004_u64
				0x00,										//Sub_Index_of_OD_variable
				Array_8u,									//Source_of_data
				4);
HAL_Delay(50);

while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
SDO_abortCode_ASCII_to_Terminal();

Code_return_SDO = read_SDO (
  			    canOpenNodeSTM32.canOpenStack->SDOclient,
				0x3d,										//remote desiredNodeID Upper_F407XX
				0x6004,										//Index_of_OD_variable_at_remote_NodeID_6004_u64
  				0x00,										//Sub_Index_of_OD_variable
  				Rx_Array,									//Saved_Received_Data
  				4,											//Number_of_Byte_to_read
  				(size_t*)&Length_of_Ext_Var );HAL_Delay(50);


#if 3

		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
		Length_of_Message = sprintf( Message_to_Terminal_2,
		  	  	                     "   \n\r EXECUTED read_SDO(...); for the SECOND time\n\r");
		Message_to_Terminal_2[0]=0x08;
		Message_to_Terminal_2[1]=0x08;
		Message_to_Terminal_2[2]=0x08;
		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal_2, Length_of_Message);

		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
		Length_of_Message = sprintf( Message_to_Terminal_1,
                " Read NEW DATA from Node_0x3d OD_Index=0x6004 SubIndex=0x0E\n\r"
                "and\n\r Save to\n\r");
		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal_1, Length_of_Message);

		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
		Length_of_Message = sprintf( Message_to_Terminal,
                "Rx_Array={0x%X,0x%X,0x%X,0x%X,0x%X,0x%X,0x%X,0x%X} \n\r",
				Array_8u[0],Array_8u[1],Array_8u[2],Array_8u[3],
				Array_8u[4],Array_8u[5],Array_8u[6],Array_8u[7]);
		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal, Length_of_Message); HAL_Delay(10);

SDO_abortCode_ASCII_to_Terminal();
 HAL_Delay(50);
#endif//3
 return Code_return_SDO;
}
/////////////////////////////////////////////////////////////////////

  void TPDO_send(uint32_t Period_ms) {
		  if(HAL_GetTick() - Ticks>Period_ms)
		  {
			Ticks = HAL_GetTick();
			CO_TPDOsendRequest(&canOpenNodeSTM32.canOpenStack->TPDO[0] );
		  }
   }


/////////////////////////////////////////////////////////////////////



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
