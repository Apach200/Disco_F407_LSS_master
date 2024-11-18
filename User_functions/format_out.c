/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Format_out.c
  * @brief          : User Format output functions
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "format_out.h"


/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/

//#define CO_Aliex_Disco407green	0x3A



/* Private variables ---------------------------------------------------------*/
extern	DMA_HandleTypeDef hdma_memtomem_dma2_stream0;
extern	DMA_HandleTypeDef hdma_memtomem_dma2_stream1;
extern	DMA_HandleTypeDef hdma_memtomem_dma2_stream3;


extern	RTC_HandleTypeDef hrtc;
extern	RTC_DateTypeDef DateToUpdate;
extern	RTC_TimeTypeDef sTime;

extern	UART_HandleTypeDef huart1;
extern	UART_HandleTypeDef huart2;
extern	DMA_HandleTypeDef hdma_usart1_rx;
extern	DMA_HandleTypeDef hdma_usart1_tx;
extern	DMA_HandleTypeDef hdma_usart2_tx;
extern	DMA_HandleTypeDef hdma_usart2_rx;

/* Private functions -----------------------------------------------*/
///////////////////////////////////////////////////////////////////////////

void Message_2_UART(char *pMessage)
{
static uint8_t	Array_2_UART_a[128];
static uint8_t	Array_2_UART_b[128];
static uint8_t	Array_2_UART_c[128];
static uint8_t Select_Array =0;
uint16_t Size_to_Send;

switch (Select_Array)
{
case 0:	Size_to_Send = sprintf((char*)Array_2_UART_a,pMessage);
		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Array_2_UART_a, Size_to_Send);
break;
case 1:	Size_to_Send = sprintf((char*)Array_2_UART_b,pMessage);
		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Array_2_UART_b, Size_to_Send);
break;
case 2:	Size_to_Send = sprintf((char*)Array_2_UART_a,pMessage);
		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Array_2_UART_c, Size_to_Send);
break;
default: break;
}

Select_Array++;
Select_Array = Select_Array %3;
}

///////////////////////////////////////////////////////////////////////////


uint16_t Process_Rx_Array_UART_DMA(uint8_t *Array,uint16_t Size_of_Array)
{
uint16_t N_shift=0;
if((uint16_t)Array[N_shift]==0x0d
	&&
	Array[(N_shift+1)%Size_of_Array]==0x0A
){return N_shift; }	//end of message detected

if(Array[N_shift]==0x0A
	&&
	Array[(N_shift+1)%Size_of_Array]==0x0d
){return N_shift; }	//end of message detected
return Size_of_Array+1;
}

///////////////////////////////////////////////

uint16_t RTC_update_and_Terminal(uint32_t Period_update_ms)
{
	static uint8_t Sekunden=0;
	static uint8_t Minuten=0;
	static uint8_t Uhr=0;
	static	uint32_t Tick_old=0;
    uint16_t cnt=0;
//    RTC_DateTypeDef DateToUpdate = {0};
//    RTC_TimeTypeDef sTime = {0};
    //					    0,//uint8_t Hours; Max_Data=12 if the RTC_HourFormat_12; Max_Data=23 if the RTC_HourFormat_24
    //						0,//uint8_t Minutes; Max_Data = 59
    //						0,//uint8_t Seconds; Max_Data = 59 */
    //						0,//uint8_t TimeFormat;Specifies the RTC AM/PM Time.
    //						0,//uint32_t SecondFraction;parameter corresponds to a time unit range between [0-1] Second with [1 Sec / SecondFraction +1] granularity
    //						0,//uint32_t DayLightSaving;  This interface is deprecated.
    //						0//uint32_t StoreOperation;

  if(
	  (HAL_GetTick() - Tick_old) > 1999
	){
	  Tick_old=HAL_GetTick();
	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
	  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
	 }

	if(Sekunden != sTime.Seconds){
		Sekunden = sTime.Seconds;
		Minuten  = sTime.Minutes;
		Uhr      = sTime.Hours;
		Get_Time_output(&Uhr, &Minuten, &Sekunden);
		cnt++;
		}
	return cnt;
}

////////////////////////////////////////////////////////////////////////////

void GPIO_Blink_Test(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t Count_of_Blink, uint16_t Period_of_blink_ms)
{
	for(uint8_t cnt=0;cnt<Count_of_Blink;cnt++)
	{
	HAL_GPIO_TogglePin(GPIOx, GPIO_Pin );
	HAL_Delay(Period_of_blink_ms);
	}
	  //HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}



uint16_t  SDO_abortCode_to_String(CO_SDO_abortCode_t Code, char* pString)
{
//static	char String_Loc[128];
	/// SDO abort codes
const char Message_SDO_AB_NONE[]="CO_SDO_AB_NONE = 0x00000000UL  /* No abort */\n\r\n\r";
const char Message_SDO_AB_TOGGLE_BIT[]="CO_SDO_AB_TOGGLE_BIT = 0x05030000UL /* Toggle bit not altered */\n\r\n\r";
const char Message_SDO_AB_TIMEOUT[]="CO_SDO_AB_TIMEOUT = 0x05040000UL /* SDO protocol timed out */\n\r\n\r";
const char Message_SDO_AB_CMD[]="CO_SDO_AB_CMD = 0x05040001UL  /*Command specifier not valid or unknown */\n\r\n\r";
const char Message_SDO_AB_BLOCK_SIZE[]="CO_SDO_AB_BLOCK_SIZE = 0x05040002UL /* Invalid block size in block mode */\n\r\n\r";
const char Message_SDO_AB_SEQ_NUM[]="CO_SDO_AB_SEQ_NUM = 0x05040003UL /* Invalid sequence number in block mode */\n\r\n\r";
const char Message_SDO_AB_CRC[]="CO_SDO_AB_CRC = 0x05040004UL /*  CRC error (block mode only) */\n\r\n\r";
const char Message_SDO_AB_OUT_OF_MEM[]="CO_SDO_AB_OUT_OF_MEM = 0x05040005UL /* Out of memory */\n\r\n\r";
const char Message_SDO_AB_UNSUPPORTED_ACCESS[]="CO_SDO_AB_UNSUPPORTED_ACCESS = 0x06010000UL /* Unsupported access to an object */\n\r\n\r";
const char Message_SDO_AB_WRITEONLY[]="CO_SDO_AB_WRITEONLY = 0x06010001UL /*  Attempt to read a write only object */\n\r\n\r";
const char Message_SDO_AB_READONLY[]="CO_SDO_AB_READONLY = 0x06010002UL /* Attempt to write a read only object */\n\r\n\r";
const char Message_SDO_AB_NOT_EXIST[]="CO_SDO_AB_NOT_EXIST = 0x06020000UL /* Object does not exist in the object dictionary */\n\r\n\r";
const char Message_SDO_AB_NO_MAP[]="CO_SDO_AB_NO_MAP = 0x06040041UL /*  Object cannot be mapped to the PDO */\n\r\n\r";
const char Message_SDO_AB_MAP_LEN[]="CO_SDO_AB_MAP_LEN = 0x06040042UL /* Number and length of object to be mapped exceeds PDO length does not match */\n\r\n\r";
const char Message_SDO_AB_PRAM_INCOMPAT[]="CO_SDO_AB_PRAM_INCOMPAT = 0x06040043UL /* General parameter incompatibility reasons */\n\r\n\r";
const char Message_SDO_AB_DEVICE_INCOMPAT[]="CO_SDO_AB_DEVICE_INCOMPAT = 0x06040047UL /* General internal incompatibility in device */\n\r\n\r";
const char Message_SDO_AB_HW[]="CO_SDO_AB_HW = 0x06060000UL /* Access failed due to hardware error */\n\r\n\r";
const char Message_SDO_AB_TYPE_MISMATCH[]="CO_SDO_AB_TYPE_MISMATCH = 0x06070010UL /* Data type does not match, length of service parameter\n\r\n\r";
const char Message_SDO_AB_DATA_LONG[]="CO_SDO_AB_DATA_LONG = 0x06070012UL /* Data type does not match, length of service parameter too high */\n\r\n\r";
const char Message_SDO_AB_DATA_SHORT[]="CO_SDO_AB_DATA_SHORT = 0x06070013UL /* Data type does not match, length of service parameter too short */\n\r\n\r";
const char Message_SDO_AB_SUB_UNKNOWN[]="CO_SDO_AB_SUB_UNKNOWN = 0x06090011UL /* Sub index does not exist */\n\r\n\r";
const char Message_SDO_AB_INVALID_VALUE[]="CO_SDO_AB_INVALID_VALUE = 0x06090030UL /* Invalid value for parameter (download only). */\n\r\n\r";
const char Message_SDO_AB_VALUE_HIGH[]="CO_SDO_AB_VALUE_HIGH = 0x06090031UL /* Value range of parameter written too high */\n\r\n\r";
const char Message_SDO_AB_VALUE_LOW[]="CO_SDO_AB_VALUE_LOW = 0x06090032UL /* Value range of parameter written too low */\n\r\n\r";
const char Message_SDO_AB_MAX_LESS_MIN[]="CO_SDO_AB_MAX_LESS_MIN = 0x06090036UL /* Maximum value is less than minimum value. */\n\r\n\r";
const char Message_SDO_AB_NO_RESOURCE[]="CO_SDO_AB_NO_RESOURCE = 0x060A0023UL /* Resource not available: SDO connection */\n\r\n\r";
const char Message_SDO_AB_GENERAL[]="CO_SDO_AB_GENERAL = 0x08000000UL /* General error */\n\r\n\r";
const char Message_SDO_AB_DATA_TRANSF[]="CO_SDO_AB_DATA_TRANSF = 0x08000020UL /* Data cannot be transferred or stored to application */\n\r\n\r";
const char Message_SDO_AB_DATA_LOC_CTRL[]="CO_SDO_AB_DATA_LOC_CTRL = 0x08000021UL /* Data cannot be transferred or stored to application because of local control */\n\r\n\r";
const char Message_SDO_AB_DATA_DEV_STATE[]="CO_SDO_AB_DATA_DEV_STATE = 0x08000022UL /*Data cannot be transferred or stored to application because of present device state */\n\r\n\r";
const char Message_SDO_AB_DATA_OD[]="CO_SDO_AB_DATA_OD = 0x08000023UL /* Object dictionary not present or dynamic generation fails */\n\r\n\r";
const char Message_SDO_AB_NO_DATA[]="CO_SDO_AB_NO_DATA = 0x08000024UL /* No data available */\n\r\n\r";

//The abort codes not listed above are reserved.
const char Message_Default[]="UNKNOUN CODE\n\r\n\r";
uint8_t Length_Message;
//char *pMessage;

	
switch((uint32_t )Code)
{
case CO_SDO_AB_NONE://pMessage=(char*)Message_SDO_AB_NONE;
	Length_Message = sizeof( Message_SDO_AB_NONE);
	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_NONE, (uint32_t)pString, Length_Message);
//	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_NONE, (uint32_t)String_Loc, Length_Message);
	break;


case CO_SDO_AB_TOGGLE_BIT://pMessage= (char*)Message_SDO_AB_TOGGLE_BIT;
	Length_Message = sizeof( Message_SDO_AB_TOGGLE_BIT);
	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_TOGGLE_BIT, (uint32_t)pString, Length_Message);
//	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_TOGGLE_BIT, (uint32_t)String_Loc, Length_Message);
	break;


case CO_SDO_AB_TIMEOUT://pMessage= (char*)Message_SDO_AB_TIMEOUT;
	Length_Message = sizeof( Message_SDO_AB_TIMEOUT);
	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_TIMEOUT, (uint32_t)pString, Length_Message);
//	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_TIMEOUT, (uint32_t)String_Loc, Length_Message);
		break;

case CO_SDO_AB_CMD://pMessage=(char*)Message_SDO_AB_CMD;
	Length_Message = sizeof( Message_SDO_AB_CMD);
	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_CMD, (uint32_t)pString, Length_Message);
	break;

case CO_SDO_AB_BLOCK_SIZE://pMessage=(char*)Message_SDO_AB_BLOCK_SIZE;
	Length_Message = sizeof( Message_SDO_AB_BLOCK_SIZE);
	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_BLOCK_SIZE, (uint32_t)pString, Length_Message);
	break;

case CO_SDO_AB_SEQ_NUM://pMessage=(char*)Message_SDO_AB_SEQ_NUM;
	Length_Message = sizeof( Message_SDO_AB_SEQ_NUM);
	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_SEQ_NUM, (uint32_t)pString, Length_Message);
break;

case CO_SDO_AB_CRC://pMessage=(char*)Message_SDO_AB_CRC;
	Length_Message = sizeof( Message_SDO_AB_CRC);
	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_CRC, (uint32_t)pString, Length_Message);
break;

case CO_SDO_AB_OUT_OF_MEM://pMessage=(char*)Message_SDO_AB_OUT_OF_MEM;
	Length_Message = sizeof( Message_SDO_AB_OUT_OF_MEM);
	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_OUT_OF_MEM, (uint32_t)pString, Length_Message);
break;

case CO_SDO_AB_UNSUPPORTED_ACCESS://pMessage=(char*)Message_SDO_AB_UNSUPPORTED_ACCESS;
	Length_Message = sizeof( Message_SDO_AB_UNSUPPORTED_ACCESS);
	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_UNSUPPORTED_ACCESS, (uint32_t)pString, Length_Message);
break;

case CO_SDO_AB_WRITEONLY://pMessage=(char*)Message_SDO_AB_WRITEONLY;
	Length_Message = sizeof( Message_SDO_AB_WRITEONLY);
	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_WRITEONLY, (uint32_t)pString, Length_Message);
break;

case CO_SDO_AB_READONLY://pMessage=(char*)Message_SDO_AB_READONLY;
	Length_Message = sizeof( Message_SDO_AB_READONLY);
	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_READONLY, (uint32_t)pString, Length_Message);
break;

case CO_SDO_AB_NOT_EXIST://pMessage=(char*)Message_SDO_AB_NOT_EXIST;
	Length_Message = sizeof( Message_SDO_AB_NOT_EXIST);
	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_NOT_EXIST, (uint32_t)pString, Length_Message);
break;

case CO_SDO_AB_NO_MAP://pMessage=(char*)Message_SDO_AB_NO_MAP;
	Length_Message = sizeof( Message_SDO_AB_NO_MAP);
	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_NO_MAP, (uint32_t)pString, Length_Message);
break;

case CO_SDO_AB_MAP_LEN://pMessage=(char*)Message_SDO_AB_MAP_LEN;
	Length_Message = sizeof( Message_SDO_AB_MAP_LEN);
	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_MAP_LEN, (uint32_t)pString, Length_Message);
break;

case CO_SDO_AB_PRAM_INCOMPAT://pMessage=(char*)Message_SDO_AB_PRAM_INCOMPAT;
	Length_Message = sizeof( Message_SDO_AB_PRAM_INCOMPAT);
	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_PRAM_INCOMPAT, (uint32_t)pString, Length_Message);
break;

case CO_SDO_AB_DEVICE_INCOMPAT://pMessage=(char*)Message_SDO_AB_DEVICE_INCOMPAT;
	Length_Message = sizeof( Message_SDO_AB_DEVICE_INCOMPAT);
	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_DEVICE_INCOMPAT, (uint32_t)pString, Length_Message);
break;

case CO_SDO_AB_HW://pMessage=(char*)Message_SDO_AB_HW;
	Length_Message = sizeof( Message_SDO_AB_HW);
	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_HW, (uint32_t)pString, Length_Message);
break;

case CO_SDO_AB_TYPE_MISMATCH://pMessage=(char*)Message_SDO_AB_TYPE_MISMATCH;
	Length_Message = sizeof( Message_SDO_AB_TYPE_MISMATCH);
	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_TYPE_MISMATCH, (uint32_t)pString, Length_Message);
break;

case CO_SDO_AB_DATA_LONG://pMessage=(char*)Message_SDO_AB_DATA_LONG;
	Length_Message = sizeof( Message_SDO_AB_DATA_LONG);
	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_DATA_LONG, (uint32_t)pString, Length_Message);
break;

case CO_SDO_AB_DATA_SHORT://pMessage=(char*)Message_SDO_AB_DATA_SHORT;
	Length_Message = sizeof( Message_SDO_AB_DATA_SHORT);
	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_DATA_SHORT, (uint32_t)pString, Length_Message);
break;

case CO_SDO_AB_SUB_UNKNOWN://pMessage=(char*)Message_SDO_AB_SUB_UNKNOWN;
	Length_Message = sizeof( Message_SDO_AB_SUB_UNKNOWN);
	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_SUB_UNKNOWN, (uint32_t)pString, Length_Message);
break;

case CO_SDO_AB_INVALID_VALUE://pMessage=(char*)Message_SDO_AB_INVALID_VALUE;
	Length_Message = sizeof( Message_SDO_AB_INVALID_VALUE);
	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_INVALID_VALUE, (uint32_t)pString, Length_Message);
break;

case CO_SDO_AB_VALUE_HIGH://pMessage=(char*)Message_SDO_AB_VALUE_HIGH;
	Length_Message = sizeof( Message_SDO_AB_VALUE_HIGH);
	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_VALUE_HIGH, (uint32_t)pString, Length_Message);
break;

case CO_SDO_AB_VALUE_LOW://pMessage=(char*)Message_SDO_AB_VALUE_LOW;
	Length_Message = sizeof( Message_SDO_AB_VALUE_LOW);
	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_VALUE_LOW, (uint32_t)pString, Length_Message);
	//pString =  Message_SDO_AB_VALUE_LOW;
break;
case CO_SDO_AB_MAX_LESS_MIN://pMessage=(char*)Message_SDO_AB_MAX_LESS_MIN;
	Length_Message = sizeof( Message_SDO_AB_MAX_LESS_MIN);
	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_MAX_LESS_MIN, (uint32_t)pString, Length_Message);
	//pString =  Message_SDO_AB_MAX_LESS_MIN;
break;

case CO_SDO_AB_NO_RESOURCE://pMessage=(char*)Message_SDO_AB_NO_RESOURCE;
	Length_Message = sizeof( Message_SDO_AB_NO_RESOURCE);
	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_NO_RESOURCE, (uint32_t)pString, Length_Message);
	//pString =  Message_SDO_AB_NO_RESOURCE;
break;

case CO_SDO_AB_GENERAL://pMessage=(char*)Message_SDO_AB_GENERAL;
	Length_Message = sizeof( Message_SDO_AB_GENERAL);
	HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_GENERAL, (uint32_t)pString, Length_Message);
	//pString =  Message_SDO_AB_GENERAL;
break;

case CO_SDO_AB_DATA_TRANSF:	//pMessage=(char*)Message_SDO_AB_DATA_TRANSF;
Length_Message = sizeof( Message_SDO_AB_DATA_TRANSF);
HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_DATA_TRANSF, (uint32_t)pString, Length_Message);
break;

case CO_SDO_AB_DATA_LOC_CTRL://pMessage=(char*)Message_SDO_AB_DATA_LOC_CTRL;
Length_Message = sizeof( Message_SDO_AB_DATA_LOC_CTRL);
HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_DATA_LOC_CTRL, (uint32_t)pString, Length_Message);
break;

case CO_SDO_AB_DATA_DEV_STATE://pMessage=(char*)Message_SDO_AB_DATA_DEV_STATE;
Length_Message = sizeof( Message_SDO_AB_DATA_DEV_STATE);
HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_DATA_DEV_STATE, (uint32_t)pString, Length_Message);
break;

case CO_SDO_AB_DATA_OD:	//pMessage =(char*)Message_SDO_AB_DATA_OD;
Length_Message = sizeof( Message_SDO_AB_DATA_OD);
HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_DATA_OD, (uint32_t)pString, Length_Message);
break;

case CO_SDO_AB_NO_DATA://pMessage =(char*)Message_SDO_AB_NO_DATA;
Length_Message = sizeof( Message_SDO_AB_NO_DATA);
HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_SDO_AB_NO_DATA, (uint32_t)pString, Length_Message);
break;

default://pMessage=(char*)Message_Default;
Length_Message = sizeof( Message_Default);
HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)Message_Default, (uint32_t)pString, Length_Message);
break;
}
//while( *(pString+Length_Message-5) !=*(pMessage+Length_Message-5)){;}
//while(
//		HAL_DMA_GetState(&hdma_memtomem_dma2_stream0) != HAL_DMA_STATE_READY
//	    ||(HAL_DMA_GetState(&hdma_memtomem_dma2_stream0) != HAL_DMA_STATE_RESET)
//	    ||(HAL_DMA_GetState(&hdma_memtomem_dma2_stream0) != HAL_DMA_STATE_TIMEOUT)
//	){;}
//while(hdma_memtomem_dma2_stream0.State != HAL_DMA_STATE_READY){;}
HAL_Delay(1);
HAL_DMA_Abort(&hdma_memtomem_dma2_stream0);
return Length_Message;
}//end_of_SDO_abortCode_to_String(CO_SDO_abortCode_t Code)

/////////////////////////////////////////////////////////////////////////

void Get_Time(void)
{
///Time & Date output
	char Array_char_x_64[64]={};
	uint16_t Length_Msg;
	//RTC_TimeTypeDef sTime = {0};
	//					    0,//uint8_t Hours; Max_Data=12 if the RTC_HourFormat_12; Max_Data=23 if the RTC_HourFormat_24
	//						0,//uint8_t Minutes; Max_Data = 59
	//						0,//uint8_t Seconds; Max_Data = 59 */
	//						0,//uint8_t TimeFormat;Specifies the RTC AM/PM Time.
	//						0,//uint32_t SecondFraction;parameter corresponds to a time unit range between [0-1] Second with [1 Sec / SecondFraction +1] granularity
	//						0,//uint32_t DayLightSaving;  This interface is deprecated.
	//						0//uint32_t StoreOperation;
			HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
			Length_Msg=sprintf(Array_char_x_64,
								"   *  System Time %d.%d.%02d \n\r", //  System
								sTime.Hours, sTime.Minutes, sTime.Seconds);
			HAL_UART_Transmit( &TerminalInterface, (uint8_t*)(Array_char_x_64), Length_Msg,5);
			while(TerminalInterface.gState != HAL_UART_STATE_READY){;}

}


//////////////////////////////////////////////////////////////////////////////////

void Get_Time_output(uint8_t *Uhren,uint8_t *Minutn,uint8_t *Sekundn)
{
//Time & Date output
	char Array_char_x_32[32]={0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08};
	uint16_t Length_Msg;
	//RTC_TimeTypeDef sTime = {0};
	//					    0,//uint8_t Hours; Max_Data=12 if the RTC_HourFormat_12; Max_Data=23 if the RTC_HourFormat_24
	//						0,//uint8_t Minutes; Max_Data = 59
	//						0,//uint8_t Seconds; Max_Data = 59 */
	//						0,//uint8_t TimeFormat;Specifies the RTC AM/PM Time.
	//						0,//uint32_t SecondFraction;parameter corresponds to a time unit range between [0-1] Second with [1 Sec / SecondFraction +1] granularity
	//						0,//uint32_t DayLightSaving;  This interface is deprecated.
	//						0//uint32_t StoreOperation;

		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
		Length_Msg=sprintf( 8+Array_char_x_32 ,
							"%02d.%02d.%02d", //  System
							*Uhren, *Minutn, *Sekundn);//	sTime.Hours, sTime.Minutes, sTime.Seconds

		HAL_UART_Transmit( &TerminalInterface, (uint8_t*)(Array_char_x_32), Length_Msg+8,5);
		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}

}

////////////////////////////////////////////////////////////////////////

void Get_Date(void)
{
///Date output
	char Array_char_x_32[32]={};
	uint16_t Length_Msg;
	//RTC_DateTypeDef DateToUpdate = {0};

HAL_RTC_GetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
Length_Msg=sprintf(Array_char_x_32,
					"   *  Date %d.%d.20%d\n\r",
					DateToUpdate.Date, DateToUpdate.Month, DateToUpdate.Year);
//CDC_Transmit_FS((uint8_t*)Array_for_Messages, strlen(Array_for_Messages));//to_usb
HAL_UART_Transmit( &TerminalInterface, (uint8_t*)(Array_char_x_32), Length_Msg,2);
//while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
}



//////////////////////////////////////////////////
