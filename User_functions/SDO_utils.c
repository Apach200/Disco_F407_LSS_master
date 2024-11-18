/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : SDO_utils.c
  * @brief          : Test_SDO_read_SDO_write
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "SDO_utils.h"


#include "CO_app_STM32.h"
#include "301/CO_SDOclient.h"
#include "CANopen.h"
#include "OD.h"

/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/

//#define CO_Aliex_Disco407green	0x3A



/* Private variables ---------------------------------------------------------*/
extern	DMA_HandleTypeDef hdma_memtomem_dma2_stream0;
extern	DMA_HandleTypeDef hdma_memtomem_dma2_stream1;
extern	DMA_HandleTypeDef hdma_memtomem_dma2_stream3;


extern	RTC_HandleTypeDef hrtc;
extern	RTC_DateTypeDef   DateToUpdate;
extern	RTC_TimeTypeDef   sTime;

extern	UART_HandleTypeDef huart1;
extern	UART_HandleTypeDef huart2;
extern	DMA_HandleTypeDef hdma_usart1_rx;
extern	DMA_HandleTypeDef hdma_usart1_tx;
extern	DMA_HandleTypeDef hdma_usart2_tx;
extern	DMA_HandleTypeDef hdma_usart2_rx;


extern	uint8_t  Tx_Array [16];
extern	uint8_t  Rx_Array [16];
extern	uint32_t Array_32u[16];
extern	uint8_t  Array_8u [16];
extern	char Message_to_Terminal  [128];
extern	char Message_to_Terminal_1[128];
extern	char Message_to_Terminal_2[128];
extern	char Message_to_Terminal_3[128];
extern	uint8_t Length_of_Message;
extern	uint8_t Length_of_Ext_Var;
extern	uint8_t Local_Count;

/* Private functions -----------------------------------------------*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////


CO_SDO_abortCode_t	read_SDO	(
								  CO_SDOclient_t* SDO_C,
								  uint8_t nodeId, 	//Remote_NodeID
								  uint16_t index,	//OD_Index_of_entire_at_Remote_NodeID
								  uint8_t subIndex, // OD_SubIndex_of_entire_at_Remote_NodeID
								  uint8_t* buf, 	//Saved_Data_Array
								  size_t bufSize, 	//Number_of_Bytes_Read_from_Remote_NodeID
								  size_t* readSize 	//pointer_at_Number_of_Bytes_to_save
								  )
{
    CO_SDO_return_t SDO_ret;

    // setup client (this can be skipped, if remote device don't change)
    SDO_ret = CO_SDOclient_setup (
    								SDO_C, CO_CAN_ID_SDO_CLI + nodeId,
									CO_CAN_ID_SDO_SRV + nodeId,
									nodeId);

    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) { return CO_SDO_AB_GENERAL; }



    // initiate upload
    SDO_ret = CO_SDOclientUploadInitiate ( SDO_C,
    										index,
											subIndex,
											1000,
											false);

    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) { return CO_SDO_AB_GENERAL; }



    // upload data
    do 	{
        uint32_t timeDifference_us = 10000;
        CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;

        SDO_ret = CO_SDOclientUpload(SDO_C, timeDifference_us, false, &abortCode, NULL, NULL, NULL);

        if (SDO_ret < 0) {  return abortCode;  }

        HAL_Delay(timeDifference_us/1000);// sleep_us(timeDifference_us);

    	} while (SDO_ret > 0);


    // copy data to the user buffer (for long data function must be called several times inside the loop)
    *readSize = CO_SDOclientUploadBufRead(SDO_C, buf, bufSize);

    return CO_SDO_AB_NONE;
}

CO_SDO_abortCode_t	write_SDO 	(
								CO_SDOclient_t* SDO_C,
								uint8_t nodeId, 	//Remote_NodeID
								uint16_t index,	//OD_Index_of_entire_at_Remote_NodeID
								uint8_t subIndex, // OD_SubIndex_of_entire_at_Remote_NodeID
								uint8_t* data,	//Data_Array_to_write_into_entire_at_Remote_NodeID
								size_t dataSize	//Number_of_Bytes_write_into_entire_at_Remote_NodeID
								)
{
    CO_SDO_return_t SDO_ret;
    bool_t bufferPartial = false;

    // setup client (this can be skipped, if remote device is the same)
    SDO_ret = CO_SDOclient_setup (	SDO_C,
    								CO_CAN_ID_SDO_CLI + nodeId,
									CO_CAN_ID_SDO_SRV + nodeId,
									nodeId);

    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) { return -1; }



    // initiate download
    SDO_ret = CO_SDOclientDownloadInitiate(SDO_C, index, subIndex, dataSize, 1000, false);

    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) /**< Success, end of communication. SDO client: uploaded data must be read. */
    	{ return -1; }



    // fill data
    size_t nWritten = CO_SDOclientDownloadBufWrite(SDO_C, data, dataSize);

    if (nWritten < dataSize) { bufferPartial = true; } // If SDO Fifo buffer is too small, data can be refilled in the loop.




    // download data
    do {
        uint32_t timeDifference_us = 10000;
        CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;

        SDO_ret = CO_SDOclientDownload (	SDO_C,
        									timeDifference_us,
											false, bufferPartial,
											&abortCode,
											NULL,
											NULL
										);

        if (SDO_ret < 0) {  return abortCode;}

        HAL_Delay(timeDifference_us/1000); //sleep_us(timeDifference_us);

       } while (SDO_ret > 0);

    return CO_SDO_AB_NONE;
}


//////////////////////////////////////////////////////////


