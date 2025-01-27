
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
#include "CO_SDOclient.h"
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

    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) { return (CO_SDO_AB_GENERAL); }



    // initiate upload
    SDO_ret = CO_SDOclientUploadInitiate ( SDO_C,
    										index,
											subIndex,
											1000,
											false);

    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) { return (CO_SDO_AB_GENERAL); }



    // upload data
    do 	{
        uint32_t timeDifference_us = 10000;
        CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;

        SDO_ret = CO_SDOclientUpload(SDO_C, timeDifference_us, false, &abortCode, NULL, NULL, NULL);

        if (SDO_ret < 0) {  return (abortCode);  }

        HAL_Delay(timeDifference_us/1000);// sleep_us(timeDifference_us);

    	} while (SDO_ret > 0);


    // copy data to the user buffer (for long data function must be called several times inside the loop)
    *readSize = CO_SDOclientUploadBufRead(SDO_C, buf, bufSize);

    return (CO_SDO_AB_NONE);
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

    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) { return (-1); }



    // initiate download
    SDO_ret = CO_SDOclientDownloadInitiate(SDO_C, index, subIndex, dataSize, 1000, false);

    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) /**< Success, end of communication. SDO client: uploaded data must be read. */
    { return (-1); }



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

        if (SDO_ret < 0) {  return (abortCode);}

        HAL_Delay(timeDifference_us/1000); //sleep_us(timeDifference_us);

       } while (SDO_ret > 0);

    return (CO_SDO_AB_NONE);
}


uint16_t LSS_Init_Message_Return(CO_ReturnError_t Err, char* String_64){



uint16_t	Lng_String;
uint8_t	Num;

char Message_0[] ={"\n\rLSS_Init_Result\n\r  CO_ERROR_NO = 0,  __Operation completed successfully__          \r\n\r\n"};
char Message_1[] ={"\n\rLSS_Init_Result\n\r  CO_ERROR_ILLEGAL_ARGUMENT = -1,  __Error in function arguments__    \n\r"};
char Message_2[] ={"\n\rLSS_Init_Result\n\r  CO_ERROR_OUT_OF_MEMORY = -2,   __Memory allocation failed          \n\r" };
char Message_3[] ={"\n\rLSS_Init_Result\n\r  CO_ERROR_TIMEOUT = -3,   __Function timeout__                    \n\r"   };
char Message_4[] ={"\n\rLSS_Init_Result\n\r  CO_ERROR_ILLEGAL_BAUDRATE = -4, __Illegal baudrate passed to function CO_CANmodule_init()__\n\r"  };
char Message_5[] ={"\n\rLSS_Init_Result\n\r  CO_ERROR_RX_OVERFLOW = -5, /**< Previous message was not processed yet */\n\r"  };
char Message_6[] ={"\n\rLSS_Init_Result\n\r  CO_ERROR_RX_PDO_OVERFLOW = -6,  /**< previous PDO was not processed yet */\n\r" };
char Message_7[] ={"\n\rLSS_Init_Result\n\r  CO_ERROR_RX_MSG_LENGTH = -7,    /**< Wrong receive message length */\n\r"   };
char Message_8[] ={"\n\rLSS_Init_Result\n\r  CO_ERROR_RX_PDO_LENGTH = -8,    /**< Wrong receive PDO length */\n\r"   };
char Message_9[] ={"\n\rLSS_Init_Result\n\r  CO_ERROR_TX_OVERFLOW = -9,      /**< Previous message is still waiting, buffer full */\n\r"   };
char Message_10[]={"\n\rLSS_Init_Result\n\r  CO_ERROR_TX_PDO_WINDOW = -10,   /**< Synchronous TPDO is outside window */\n\r"   };
char Message_11[]={"\n\rLSS_Init_Result\n\r  CO_ERROR_TX_UNCONFIGURED = -11, /**< Transmit buffer was not configured properly */\n\r"   };
char Message_12[]={"\n\rLSS_Init_Result\n\r  CO_ERROR_OD_PARAMETERS = -12,   /**< Error in Object Dictionary parameters */\n\r"   };
char Message_13[]={"\n\rLSS_Init_Result\n\r  CO_ERROR_DATA_CORRUPT = -13,    /**< Stored data are corrupt */\n\r"   };
char Message_14[]={"\n\rLSS_Init_Result\n\r  CO_ERROR_CRC = -14,   /**< CRC does not match */\n\r"   };
char Message_15[]={"\n\rLSS_Init_Result\n\r  CO_ERROR_TX_BUSY = -15,  /**< Sending rejected because driver is busy. Try again */\n\r"   };
char Message_16[]={"\n\rLSS_Init_Result\n\r  CO_ERROR_WRONG_NMT_STATE = -16, /**< Command can't be processed in current state */\n\r"   };
char Message_17[]={"\n\rLSS_Init_Result\n\r  CO_ERROR_SYSCALL = -17,   /**< Syscall failed */\n\r"};
char Message_18[]={"\n\rLSS_Init_Result\n\r  CO_ERROR_INVALID_STATE = -18,  /**< Driver not ready */\n\r"   };
char Message_19[]={"\n\rLSS_Init_Result\n\r  CO_ERROR_NODE_ID_UNCONFIGURED_LSS =-19 /**< Node-id is in LSS unconfigured state.\n\r \
		 If objects are handled properly, this may not be an error. */\n\r"   };


Num = (uint8_t)(-Err);
switch (Num) {
case 0: Lng_String = sprintf((char*)String_64, Message_0);break;
case 1: Lng_String = sprintf((char*)String_64, Message_1);break;
case 2: Lng_String = sprintf((char*)String_64, Message_2);break;
case 3: Lng_String = sprintf((char*)String_64, Message_3);break;
case 4: Lng_String = sprintf((char*)String_64, Message_4);break;
case 5: Lng_String = sprintf((char*)String_64, Message_5);break;
case 6: Lng_String = sprintf((char*)String_64, Message_6);break;
case 7: Lng_String = sprintf((char*)String_64, Message_7);break;
case 8: Lng_String = sprintf((char*)String_64, Message_8);break;
case 9: Lng_String = sprintf((char*)String_64, Message_9);break;
case 10:Lng_String = sprintf((char*)String_64, Message_10);break;
case 11:Lng_String = sprintf((char*)String_64, Message_11);break;
case 12:Lng_String = sprintf((char*)String_64, Message_12);break;
case 13:Lng_String = sprintf((char*)String_64, Message_13);break;
case 14:Lng_String = sprintf((char*)String_64, Message_14);break;
case 15:Lng_String = sprintf((char*)String_64, Message_15);break;
case 16:Lng_String = sprintf((char*)String_64, Message_16);break;
case 17:Lng_String = sprintf((char*)String_64, Message_17);break;
case 18:Lng_String = sprintf((char*)String_64, Message_18);break;
case 19:Lng_String = sprintf((char*)String_64, Message_19);break;
default:break;
}

HAL_UART_Transmit(&TerminalInterface, (uint8_t*)String_64, Lng_String, 100);
return (Lng_String);
}

//////////////////////////////////////////////////////////


