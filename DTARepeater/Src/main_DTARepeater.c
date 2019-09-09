/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************

  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"
#include "iwdg.h"

/* Private variables ---------------------------------------------------------*/
extern CAN_HandleTypeDef hcan;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern IWDG_HandleTypeDef hiwdg;
char	debug_str[100];

void DebugInfo(char *format)
{
		while(*format)
		{
				HAL_UART_Transmit(&huart3, (uint8_t*)format, 1, 0xffff);
				format++;
		}
}

void CameraReset(void)
{
	uint8_t cmd[]={0x81, 0x01, 0x04, 0x19, 0x03, 0xFF};
	HAL_UART_Transmit(&huart2, cmd, sizeof(cmd), 1000);
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *CanHandle)
{
		if(CanHandle ==&hcan)
		{		
			
				if (HAL_CAN_Receive_IT(&hcan, CAN_FIFO0) != HAL_OK)
				{
						Error_Handler();
				}
				else
				{
						sprintf(debug_str, "HAL_CAN_RxCpltCallback ok! data:%x %x %x %x   %x %x %x %x len:%d.\n", hcan.pRxMsg->Data[0],\
						hcan.pRxMsg->Data[1],hcan.pRxMsg->Data[2],hcan.pRxMsg->Data[3],\
						hcan.pRxMsg->Data[4],hcan.pRxMsg->Data[5],hcan.pRxMsg->Data[6],hcan.pRxMsg->Data[7], hcan.pRxMsg->DLC);
						DebugInfo(debug_str);
				}
		}
}

void CanFill(CAN_HandleTypeDef *CanHandle)
{
		static CanTxMsgTypeDef        TxMessage;
		static CanRxMsgTypeDef        RxMessage;

		CanHandle->pTxMsg = &TxMessage;
		CanHandle->pRxMsg = &RxMessage;
		CanHandle->pTxMsg->StdId = 0x321;
		CanHandle->pTxMsg->ExtId = 0x01;
		CanHandle->pTxMsg->RTR = CAN_RTR_DATA;
		CanHandle->pTxMsg->IDE = CAN_ID_STD;
		CanHandle->pTxMsg->DLC = 8;
		CanHandle->pTxMsg->Data[0] =0x01;
		CanHandle->pTxMsg->Data[1] =0x02;
		CanHandle->pTxMsg->Data[2] =0x03;
		CanHandle->pTxMsg->Data[3] =0x04;
		CanHandle->pTxMsg->Data[4] =0x05;
		CanHandle->pTxMsg->Data[5] =0x06;
		CanHandle->pTxMsg->Data[6] =0x07;
		CanHandle->pTxMsg->Data[7] =0x08;		
}

void CanPreProcess(CAN_HandleTypeDef *CanHandle)
{
		CAN_FilterConfTypeDef  		sFilterConfig;

	/*
		uint32_t StdId =0x321;                //??????CAN ID,?????CAN ID  
		uint32_t ExtId =0x1800f001;           //?????CAN ID  
			
		sFilterConfig.FilterNumber = 0;               //?????0  
		sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;     //??????  
		sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;    //???32??  
		sFilterConfig.FilterIdHigh = StdId<<5;          //??ID???STID?  
		sFilterConfig.FilterIdLow = 0|CAN_ID_STD;         //??IDE??0  
		sFilterConfig.FilterMaskIdHigh = ((ExtId<<3)>>16)&0xffff;  
		sFilterConfig.FilterMaskIdLow = ((ExtId<<3) & 0xffff) | CAN_ID_EXT;   //??IDE??1  
		sFilterConfig.FilterFIFOAssignment = 0;           //?????????FIFO0?  
		sFilterConfig.FilterActivation = ENABLE;  
		sFilterConfig.BankNumber = 14;  
	*/
	
		sFilterConfig.FilterNumber = 0;
		sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
		sFilterConfig.FilterIdHigh = 0x0000;
		sFilterConfig.FilterIdLow = 0x0000;
		sFilterConfig.FilterMaskIdHigh = 0x0000;
		sFilterConfig.FilterMaskIdLow = 0x0000;
		sFilterConfig.FilterFIFOAssignment = 0;
		sFilterConfig.FilterActivation = ENABLE;
		sFilterConfig.BankNumber = 14;
	
		if (HAL_CAN_ConfigFilter(CanHandle, &sFilterConfig) != HAL_OK)
		{
				Error_Handler();
				DebugInfo("CanConfigFilter err!\n");
		}
		else
		{
				DebugInfo("CanConfigFilter ok!\n");
		}	
		
		if (HAL_CAN_Receive_IT(&hcan, CAN_FIFO0) != HAL_OK)
		{
				Error_Handler();
				DebugInfo("start CAN receive err!\n");
		}
		else
		{
				DebugInfo("start CAN receive ok!\n");
		}		
		
		CanFill(&hcan);
}

void main_DTARepeater(void)
{
		HAL_StatusTypeDef HAL_Status;
		
		DebugInfo("init ok!\n");	
		
		CanPreProcess(&hcan);

		HAL_Delay(1000);
		CameraReset();
		DebugInfo("CameraReset.\n");
		
		HAL_IWDG_Start(&hiwdg);
		while (1)
		{
				HAL_Delay(1000);	
				DebugInfo("loop..\n");
				
				HAL_Status = HAL_CAN_Transmit(&hcan, 1000);
				if (HAL_Status != HAL_OK)
				{
						sprintf(debug_str, "HAL_CAN_Transmit err. err_code:%d.\n", HAL_Status);
						DebugInfo(debug_str);
				}
				else
				{
						DebugInfo("CAN tx ok!\n");	
				}
				
				HAL_IWDG_Refresh(&hiwdg);
		}
}
