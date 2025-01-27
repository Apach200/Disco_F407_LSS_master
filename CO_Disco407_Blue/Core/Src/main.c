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
  *			Aliboard__STM32F407_LCD
  *
  *				NodeID = 0x3f
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CO_app_STM32.h"
#include "301/CO_SDOclient.h"
#include "CANopen.h"
#include "OD.h"

#include "format_out.h"
#include "SDO_utils.h"
#include "lcd.h"
#include "Encoder.h"
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

/* USER CODE BEGIN PV */

RTC_DateTypeDef DateToUpdate;//  = {02, 01, 21, 25};  //; ={0,0,0};   //21jan2025
RTC_TimeTypeDef sTime;// = {19, 24, 0,0,0,0,0};; // ={0,0,0};      ///19h16min
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
uint32_t            Ticks_1;
char String_H[]={"String_for_Test_UART_"};
char String_L[]={"String_for_Test_UART_"};
char buff[16]={8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8};
//const	char XPOMOC[] = "XPOMOC_CANOpen";
//const	char WC[] = "LSS_Master";
char String_LCD[32];
char String_2_UART[128];
uint16_t L_str;
int16_t currCounter=0 ;
int32_t prevCounter =0;
CO_ReturnError_t Err_return;

uint8_t Menu_step=0;
const uint16_t Datum[64]={0,0,
						  0x30,0x31, 0x30,0x32, 0x30,0x33, 0x30,0x34, 0x30,0x35, 0x30,0x36, 0x30,0x37, 0x30,0x38, 0x30,0x39, 0x31,0x30,
					      0x31,0x31, 0x31,0x32, 0x31,0x33, 0x31,0x34, 0x31,0x35, 0x31,0x36, 0x31,0x37, 0x31,0x38, 0x31,0x39, 0x32,0x30,
					      0x32,0x31, 0x32,0x32, 0x32,0x33, 0x32,0x34, 0x32,0x35, 0x32,0x36, 0x32,0x37, 0x32,0x38, 0x32,0x39, 0x33,0x30,
					      0x33,0x31
					     };



uint8_t Node_ID_Read=0xff;
uint32_t Data_u32;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void CAN_interface_Test(void);
void UART_interface_Test(void);
void Board_Name_to_Terminal(void);
void CO_Init_Return_State(uint16_t Returned_Code);
void SDO_abortCode_ASCII_to_Terminal(void);
CO_SDO_abortCode_t SDO_Read_Write_Read(void);
void TPDO_send(uint32_t Period_ms);

//void Callback_GTW_Read();

int16_t Encoder_to_LCD(void);
int16_t LCD_Menu_Encoder_with_Key(int16_t* Var_0,int16_t* Var_1,int16_t* Var_2);
Encoder_Status encoderStatus;

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
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
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

//	HAL_RTC_SetTime(&hrtc, &sTime,        RTC_FORMAT_BIN);
//	HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);

	HAL_RTC_GetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
	HAL_RTC_GetTime(&hrtc, &sTime,        RTC_FORMAT_BIN);

 Encoder_Config();  // configure the encoders timer
 Encoder_Init();    // start the encoders timer
 LCD_ini();
// Logo_to_1602LCD();
Datum_to_1602LCD();
//   GPIO_Blink_Test(GPIOA, GPIO_PIN_7|GPIO_PIN_6, 25, 33); 						// for_STM32F4XX_Ali_pcb
    GPIO_Blink_Test(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, 25, 33);// blink_at_Discovery_EVB
//    UART_interface_Test(); //while(1){;}
//    CAN_interface_Test();

	HAL_TIM_Base_Start_IT(&htim8);
    HAL_UART_Receive_DMA(&huart2, Array_from_Terminal, sizeof Array_from_Terminal );
    HAL_Delay(1500);
    Board_Name_to_Terminal();
	HAL_TIM_Base_Start_IT(&htim4);

//	CANopenNodeSTM32 canOpenNodeSTM32;//перемещено вверх в глобальные переменнные
	canOpenNodeSTM32.CANHandle = &hcan1;
	canOpenNodeSTM32.HWInitFunction = MX_CAN1_Init;
	canOpenNodeSTM32.timerHandle = &htim4;
	canOpenNodeSTM32.desiredNodeID = CO_Disco407_Blue;//0x3b;
	canOpenNodeSTM32.baudrate = 125*4;
uint16_t Ret_value = canopen_app_init(&canOpenNodeSTM32);
	CO_Init_Return_State(Ret_value );

	 //SDO_Read_Write_Read();  while (1){};

Err_return = CO_LSSmaster_init(
				  	  	  	  canOpenNodeSTM32.canOpenStack->LSSmaster,			//CO_LSSmaster_t* LSSmaster,
							  CO_LSSmaster_DEFAULT_TIMEOUT,						//uint16_t timeout_ms,
							  canOpenNodeSTM32.canOpenStack->CANmodule ,		//CO_CANmodule_t* CANdevRx,
							  canOpenNodeSTM32.canOpenStack->RX_IDX_LSS_MST,	//uint16_t CANdevRxIdx,
							  CO_CAN_ID_LSS_SLV,								//uint16_t CANidLssSlave,
							  canOpenNodeSTM32.canOpenStack->CANmodule ,		//CO_CANmodule_t* CANdevTx,
							  canOpenNodeSTM32.canOpenStack->TX_IDX_LSS_MST,	//uint16_t CANdevTxIdx,
							  CO_CAN_ID_LSS_MST									//uint16_t CANidLssMaster
							);


L_str = LSS_Init_Message_Return(Err_return, String_2_UART);



//		 uint16_t StateLSS =  canOpenNodeSTM32.canOpenStack->LSSmaster->state ;
//		 L_str = sprintf(String_2_UART,"canOpenNodeSTM32.canOpenStack->LSSmaster->state=0x%04x;\n\r",StateLSS);
		  OD_PERSIST_COMM.x6000_disco_Blue_VAR32_6000_TX=0;
		  Local_Count=0;

//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, !canOpenNodeSTM32.outStatusLEDGreen);
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, !canOpenNodeSTM32.outStatusLEDRed  );
//		  canopen_app_process();HAL_Delay(1);

//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, !canOpenNodeSTM32.outStatusLEDGreen);
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, !canOpenNodeSTM32.outStatusLEDRed  );
//		  canopen_app_process(); HAL_Delay(1);
//
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, !canOpenNodeSTM32.outStatusLEDGreen);
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, !canOpenNodeSTM32.outStatusLEDRed  );
//		  canopen_app_process(); HAL_Delay(1);
//
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, !canOpenNodeSTM32.outStatusLEDGreen);
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, !canOpenNodeSTM32.outStatusLEDRed  );
//		  canopen_app_process(); HAL_Delay(1);
//
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, !canOpenNodeSTM32.outStatusLEDGreen);
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, !canOpenNodeSTM32.outStatusLEDRed  );
//		  canopen_app_process(); HAL_Delay(1);
//
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, !canOpenNodeSTM32.outStatusLEDGreen);
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, !canOpenNodeSTM32.outStatusLEDRed  );
//		  canopen_app_process(); HAL_Delay(1);
//
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, !canOpenNodeSTM32.outStatusLEDGreen);
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, !canOpenNodeSTM32.outStatusLEDRed  );
//		  canopen_app_process(); HAL_Delay(1);
//
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, !canOpenNodeSTM32.outStatusLEDGreen);
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, !canOpenNodeSTM32.outStatusLEDRed  );
//		  canopen_app_process(); HAL_Delay(1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

		  int16_t VSar_0=0, VSar_1=0, VSar_2=0;
		  LCD_Menu_Encoder_with_Key( &VSar_0, &VSar_1, &VSar_2);
		  while (1)
		  {
		  //Encoder_to_LCD();
		  RTC_update_and_Terminal(2000);
		  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, !canOpenNodeSTM32.outStatusLEDRed  );
	      // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, !canOpenNodeSTM32.outStatusLEDGreen);
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, !canOpenNodeSTM32.outStatusLEDGreen);
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, !canOpenNodeSTM32.outStatusLEDRed  );

		  canopen_app_process();
#if 0
			if(tmp32u_0 != OD_PERSIST_COMM.x6001_disco_Blue_VAR32_6001_R)
			{
			tmp32u_0 = OD_PERSIST_COMM.x6001_disco_Blue_VAR32_6001_R;
			while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
		    Length_of_Message = sprintf( Message_to_Terminal,
		  	  	                     "copy  OD_PERSIST_COMM.x6001_disco_Blue_VAR32_6001_R "
                  	  	  	  	  	  " to tmp32u_0 = 0x%X%X\n\r",(uint16_t)(tmp32u_0>>16),(uint16_t)tmp32u_0);
		    //HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal, Length_of_Message);
			}


			if(tmp32u_1 != OD_PERSIST_COMM.x6002_disco_Blue_VAR32_6002_R)
			{
			Length_of_Message = sprintf( Message_to_Terminal,
		                "check for updates tmp32u_1 = OD_PERSIST_COMM.x6002_disco_Blue_VAR32_6002_R"
		                "tmp32u_1 = 0x%X%X\n\r",(uint16_t)(tmp32u_1>>16),(uint16_t)tmp32u_1);
				while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
			//HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal, Length_of_Message);
			}

			Count_of_while1++;//Counter_of_while_cycles
#endif	//4
   if(0)//(Count_of_while1==2000)
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
	                				"\n\rcanOpenNodeSTM32.canOpenStack->LSSmaster->timeoutTimer"
	                				" = 0x%04X%04X  \n\r\n\r\n\r\n\r\n\r\n\r",
									(uint16_t)(Local_var>>16),(uint16_t)Local_var);
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal, Length_of_Message);
	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}

   }
    // TPDO_send(1700); /// uint32_t Period_ms

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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
//  const char Message_1[]={"*  Blackboard  STM32F4XX___Ali with LCD  *\n\r"};
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
//	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
//	extern uint16_t adc_buffer[];
//	ChipTemperature = (float)process_adc_buffer(adc_buffer);;
//	Msg_Length = sprintf( Array_for_Messages,
//			  	  	  	  "   *  Temperature %03.2f                      *\n\r",
//						  ChipTemperature );
//	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Array_for_Messages, Msg_Length);

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
	   Lngth_of_Message = sizeof(Msg_0);
	   Lngth_of_Message = sprintf( (char*)Msg_0," \n\r");
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
 return (Code_return_SDO);
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
  int16_t Encoder_to_LCD(void)
  {
		encoderStatus = Encoder_Get_Status();

		  switch(encoderStatus) {
		    case Incremented:
		    	currCounter++;
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
				sprintf(String_LCD,"%04d",currCounter);
				LCD_SetPos(11, 0);
				HAL_Delay(10);
				LCD_String(String_LCD);HAL_Delay(10);

				L_str=	snprintf(buff, sizeof(buff), "\n\r %04d ", currCounter);
				while(TerminalInterface.gState != HAL_UART_STATE_READY ){;}
				HAL_UART_Transmit(&TerminalInterface, (uint8_t*)buff, L_str,16);
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET  );
		      break;
		    case Decremented:
		    	currCounter--;
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
				sprintf(String_LCD,"%04d",currCounter);
				LCD_SetPos(11,0);HAL_Delay(10);

				LCD_String(String_LCD);HAL_Delay(10);

				L_str=	snprintf(buff, sizeof(buff), "\n\r %04d ", currCounter);
				while(TerminalInterface.gState != HAL_UART_STATE_READY ){;}
				HAL_UART_Transmit(&TerminalInterface, (uint8_t*)buff, L_str,16);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET  );
		      break;

		    case Neutral:

		    	break;
		    default: break;
		  }///switch(encoderStatus)

		    	if(HAL_GetTick()-Ticks_1 >1500){
											//Ticks_1=HAL_GetTick();
											//Get_Time_to_LCD(0,1);
		    								}

		    	// RTC_update_and_Terminal(1999);
		    	return (currCounter);
  }


  /////////////////////////////////////////////////////////////////////
    int16_t LCD_Menu_Encoder_with_Key(int16_t* Var_0,int16_t* Var_1,int16_t* Var_2)
    {
    	uint32_t Button_pressed;
    	uint16_t L_string;
    	char String_to_UART[32];
    	char String_0_to_LCD[20];
    	char String_1_to_LCD[20];
    	static uint16_t Menu_Enter=0;
    	uint32_t Timeout;
    	uint16_t Step;

    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
    	Timeout = HAL_GetTick();
    	Step=0;
    	if(HAL_GPIO_ReadPin(Encoder_Key_GPIO_Port, Encoder_Key_Pin)==GPIO_PIN_RESET)
    	{
    		Button_pressed = HAL_GetTick();
    		while(HAL_GPIO_ReadPin(Encoder_Key_GPIO_Port, Encoder_Key_Pin)==GPIO_PIN_RESET)
    		{
    			if(HAL_GPIO_ReadPin(Encoder_Key_GPIO_Port, Encoder_Key_Pin)==GPIO_PIN_RESET)
    			{HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);}else
    			{HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);}

    			if(Button_pressed - HAL_GetTick() > 4321){ Menu_Enter=1; }// break;
				 else {	Menu_Enter=0;
						Button_pressed = HAL_GetTick();
				 	 	 }
				if(HAL_GetTick()-Timeout>7654){LCD_Clear();HAL_Delay(10);
												return currCounter;}//exit_from_menu_after_timeout
    		}//while(HAL_GPIO_ReadPin
    		Button_pressed = HAL_GetTick();
    		while(HAL_GPIO_ReadPin(Encoder_Key_GPIO_Port, Encoder_Key_Pin)!=GPIO_PIN_SET){;}//__key_unpressed__

			if(HAL_GPIO_ReadPin(Encoder_Key_GPIO_Port, Encoder_Key_Pin)==GPIO_PIN_RESET)
			{HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);}else
			{HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);}
			if(Menu_Enter==0){LCD_Clear();return currCounter;}

    		HAL_Delay(50);
    		while(Menu_Enter)
    		{
    			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
    			if (Step==0){
        		switch(Menu_Enter)
        		{
        		case 0: ; break;

        		case 1:
        			LCD_Clear();
					L_string=sprintf(String_to_UART,"\n\r_Menu_Enter=1_");
					HAL_UART_Transmit(&TerminalInterface, (uint8_t*)String_to_UART, L_string,16);
					L_string=sprintf(String_0_to_LCD,"Var_0 %06f",(float)(*Var_0));
        			break;

        		case 2:
        			LCD_Clear();
    				L_string=sprintf(String_to_UART,"\n\r_Menu_Enter=2_");
    				HAL_UART_Transmit(&TerminalInterface, (uint8_t*)String_to_UART, L_string,16);
    				L_string=sprintf(String_0_to_LCD,"Var_1 %06f",(float)(*Var_1));
        			break;

        		case 3:
        			LCD_Clear();
    				L_string=sprintf(String_to_UART,"\n\r_Menu_Enter=3_");
    				HAL_UART_Transmit(&TerminalInterface, (uint8_t*)String_to_UART, L_string,16);
    				L_string=sprintf(String_0_to_LCD,"Var_2 %06f",(float)(*Var_2));
        			break;
        		default: break;
        		}
				LCD_SetPos(0, 0);HAL_Delay(10);
				LCD_String(String_0_to_LCD);	HAL_Delay(10);
        		Timeout=HAL_GetTick();
        		Step=1;
    			}///if (Step==0)
			do{
        		encoderStatus = Encoder_Get_Status();
				if(encoderStatus==Incremented)
				{
					Timeout=HAL_GetTick();
					switch(Menu_Enter)
	        		{
	        		case 0: ; break;

	        		case 1:
	        			*Var_0 = (*Var_0) + 1;
	        			L_string=sprintf(String_0_to_LCD,"Var_0 %06f",(float)(*Var_0));
	        			break;

	        		case 2:
	        			*Var_1 = (*Var_1) + 1;
	        			L_string=sprintf(String_0_to_LCD,"Var_1 %06f",(float)(*Var_1));
	        			break;

	        		case 3:
	        			*Var_2 = (*Var_2) + 1;
	        			L_string=sprintf(String_0_to_LCD,"Var_2 %06f",(float)(*Var_2));
	        			break;
	        		default: break;
	        		}//switch
				LCD_SetPos(0, 0);				HAL_Delay(100);
				LCD_String(String_0_to_LCD);	HAL_Delay(100);

				}///if(encoderStatus==Incremented)

				if(encoderStatus==Decremented)
				{
					Timeout=HAL_GetTick();
					switch(Menu_Enter)
	        		{
	        		case 0: Timeout = 0; break;

	        		case 1:
	        			*Var_0 = (*Var_0) - 1;
	        			L_string=sprintf(String_0_to_LCD,"Var_0 %d",(*Var_0));
	        			break;

	        		case 2:
	        			*Var_1 = (*Var_1) - 1;
	        			L_string=sprintf(String_0_to_LCD,"Var_1 %d",(*Var_1));
	        			break;

	        		case 3:
	        			*Var_2 = (*Var_2) - 1;
	        			L_string=sprintf(String_0_to_LCD,"Var_2 %d",(*Var_2));
	        			break;
	        		default: break;
	        		}//switch

				LCD_SetPos(0, 0);				HAL_Delay(100);
				LCD_String(String_0_to_LCD);	HAL_Delay(100);
				}

				if(HAL_GPIO_ReadPin(Encoder_Key_GPIO_Port, Encoder_Key_Pin)==GPIO_PIN_RESET)
				{
					HAL_Delay(200);
					L_string=sprintf(String_1_to_LCD,"Ready? Y or N     ");
					LCD_SetPos(0, 1);				HAL_Delay(10);
					LCD_String(String_1_to_LCD);	HAL_Delay(200);

					Timeout=HAL_GetTick();
				while(
					  (HAL_GetTick()-Timeout<5432)
					  ||(Encoder_Get_Status()==Decremented)
					  ||(Encoder_Get_Status()==Incremented) )
					{
					if(HAL_GetTick()-Timeout>5432)
						{Menu_Enter=0;
						LCD_Clear();HAL_Delay(10);
						 return currCounter;}//exit_from_menu_after_timeout

					encoderStatus = Encoder_Get_Status();
					if(encoderStatus==Incremented)
					{
					L_string=sprintf(String_1_to_LCD,"Yes Ready to Next");
					LCD_SetPos(0, 1);				HAL_Delay(100);
					LCD_String(String_1_to_LCD);	HAL_Delay(200);
					L_string=sprintf(String_0_to_LCD,"Press Key      ");
					LCD_SetPos(0, 0);				HAL_Delay(10);
					LCD_String(String_0_to_LCD);	HAL_Delay(200);
					Timeout=HAL_GetTick();
					while(HAL_GPIO_ReadPin(Encoder_Key_GPIO_Port, Encoder_Key_Pin)!=GPIO_PIN_RESET)
					{
						if(HAL_GetTick()-Timeout>7654){LCD_Clear();HAL_Delay(10);
														return currCounter;}//exit_from_menu_after_timeout
					}
					Menu_Enter++;
					if(Menu_Enter>3){Menu_Enter=0;}
					Step=0;
					}//if(encoderStatus==Incremented)

					if(encoderStatus==Decremented)
					{
					L_string=sprintf(String_1_to_LCD,"No, Out from Menu  ");
					LCD_SetPos(0, 1);				HAL_Delay(10);
					LCD_String(String_1_to_LCD);	HAL_Delay(200);

					L_string=sprintf(String_0_to_LCD,"Press Key      ");
					LCD_SetPos(0, 0);				HAL_Delay(10);
					LCD_String(String_0_to_LCD);	HAL_Delay(200);
					Timeout=HAL_GetTick();
					while(HAL_GPIO_ReadPin(Encoder_Key_GPIO_Port, Encoder_Key_Pin)!=GPIO_PIN_RESET)
					{
						if(HAL_GetTick()-Timeout>7654){LCD_Clear();HAL_Delay(10);
														return currCounter;}//exit_from_menu_after_timeout
					}
					Menu_Enter=0;
					}//if(encoderStatus==Decremented)


					}//while ||(Encoder_Get_Status

				encoderStatus = Encoder_Get_Status();
				if(encoderStatus==Incremented)
					{
					L_string=sprintf(String_0_to_LCD,"Press Key     ");
					LCD_SetPos(0, 0);				HAL_Delay(100);
					LCD_String(String_0_to_LCD);	HAL_Delay(200);
					Timeout=HAL_GetTick();
					while(HAL_GPIO_ReadPin(Encoder_Key_GPIO_Port, Encoder_Key_Pin)!=GPIO_PIN_RESET){}
					Menu_Enter=2;
					}//if(encoderStatus==Incremented)

				if(encoderStatus==Decremented)
					{
					L_string=sprintf(String_0_to_LCD,"Press Key      ");
					LCD_SetPos(0, 0);				HAL_Delay(10);
					LCD_String(String_0_to_LCD);	HAL_Delay(200);
					Timeout=HAL_GetTick();
					while(HAL_GPIO_ReadPin(Encoder_Key_GPIO_Port, Encoder_Key_Pin)!=GPIO_PIN_RESET){}
					Menu_Enter=0;
					}//if(encoderStatus==Incremented)


				}

			  }while((HAL_GetTick()-Timeout<5432)&&(Menu_Enter!=0));

			  if(HAL_GetTick()-Timeout>5432){LCD_Clear();HAL_Delay(10);Menu_Enter=0;}//exit_from_menu_after_timeout

    		}//while(Menu_Enter)

    	}//if(HAL_GPIO_ReadPin

  		    	return (currCounter);
    }///LCD_Menu_Encoder_with_Key()

/////////////////////////////////////////////////////////////////////

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
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
