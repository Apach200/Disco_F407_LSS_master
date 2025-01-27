/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : SDO_utils.h
  * @brief          : Header for SDO_utils.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SDO_UTILS
#define __SDO_UTILS

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "CO_SDOclient.h"
#include "CO_SDOserver.h"
#include "CO_app_STM32.h"
#include "CANopen.h"
#include "OD.h"

/* Private includes ----------------------------------------------------------*/
#include "stdio.h"

/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/
#define TerminalInterface		huart2

/* Exported functions prototypes ---------------------------------------------*/
CO_SDO_abortCode_t	read_SDO	(
								  CO_SDOclient_t* SDO_C,
								  uint8_t nodeId, 	//Remote_NodeID
								  uint16_t index,	//OD_Index_of_entire_at_Remote_NodeID
								  uint8_t subIndex, // OD_SubIndex_of_entire_at_Remote_NodeID
								  uint8_t* buf, 	//Saved_Data_Array
								  size_t bufSize, 	//Number_of_Bytes_Read_from_Remote_NodeID
								  size_t* readSize 	//pointer_at_Number_of_Bytes_to_save
								  );


CO_SDO_abortCode_t	write_SDO 	(
								CO_SDOclient_t* SDO_C,
								uint8_t nodeId, 	//Remote_NodeID
								uint16_t index,	//OD_Index_of_entire_at_Remote_NodeID
								uint8_t subIndex, // OD_SubIndex_of_entire_at_Remote_NodeID
								uint8_t* data,	//Data_Array_to_write_into_entire_at_Remote_NodeID
								size_t dataSize	//Number_of_Bytes_write_into_entire_at_Remote_NodeID
								);

uint16_t LSS_Init_Message_Return(CO_ReturnError_t Err, char* String_64);

/* Private defines -----------------------------------------------------------*/



#ifdef __cplusplus
}
#endif

#endif /* __SDO_UTILS */
