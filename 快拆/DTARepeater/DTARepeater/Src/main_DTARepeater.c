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
#include "mavlink2.h"
#include "math.h"
#include "modules.h"
#include "mavlink2.h"
#include "common/mavlink.h"

/* Private variables ---------------------------------------------------------*/
extern CAN_HandleTypeDef hcan;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern IWDG_HandleTypeDef hiwdg;
extern void gimbal_control_standard_send(mavlink_channel_t chan, uint8_t priority, \
	uint8_t yaw_mode, uint8_t pitch_mode, uint8_t roll_mode, \
	float yaw_channel, float pitch_channel, float roll_channel, float drones_yawvelocity_desire);

#define	DEVICE_CAMERA							1
#define	DEVICE_YUNTAI							2
#define	FUNC_CAMERA_STOP					1
#define	FUNC_CAMERA_FOCUS_NEAR		2
#define	FUNC_CAMERA_FOCUS_FAR			3
#define	FUNC_CAMERA_PHOTO					4
#define	FUNC_CAMERA_RECORD_START	5
#define	FUNC_CAMERA_RECORD_STOP		6
#define	FUNC_CAMERA_INIT					7
#define	FUNC_CAMERA_ZOOM_POS					0x10
#define	FUNC_CAMERA_EXP_COMP_ON				0x20
#define	FUNC_CAMERA_EXP_COMP_OFF			0x21
#define	FUNC_CAMERA_EXP_COMP_RESET		0x22
#define	FUNC_CAMERA_EXP_COMP_UP				0x23
#define	FUNC_CAMERA_EXP_COMP_DOWN			0x24
#define	FUNC_CAMERA_EXP_COMP_DIRECT		0x25
#define	FUNC_CAMERA_EXP_MONITOR_MODE	0x30		//etc. 1080p/60...
#define	FUNC_YUNTAI_YAW_PITCH					1

void CameraReset(void);
void CameraStop(void);
void CameraFocusNear(void);
void CameraFocusFar(void);
void CameraPhoto(void);
void CameraRecordStart(void);
void CameraRecordStop(void);
void CameraRecordInit(void);
void CameraZoomPos(uint8_t *data, uint8_t len);
void CameraExpCompOn(void);
void CameraExpCompOff(void);
void CameraExpCompReset(void);
void CameraExpCompUp(void);
void CameraExpCompDown(void);
void CameraExpCompDirect(uint8_t *data, uint8_t len);
void CameraMonitoeMode(uint8_t *data, uint8_t len);
float Q12ToFloat(uint16_t in);
void YuntaiYawPitch(uint8_t *data, uint8_t len);
void CANDataProcess(uint8_t *data, uint8_t len);
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *CanHandle);
void CanFill(CAN_HandleTypeDef *CanHandle);
void main_DTARepeater(void);

void CameraReset(void)
{
	uint8_t cmd[]={0x81, 0x01, 0x04, 0x19, 0x03, 0xFF};
	HAL_UART_Transmit(&huart2, cmd, sizeof(cmd), 1000);
}

void CameraStop(void)
{
	uint8_t cmd[]={0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01};
	HAL_UART_Transmit(&huart2, cmd, sizeof(cmd), 1000);
}

void CameraFocusNear(void)
{
	uint8_t cmd[]={0xff, 0x01, 0x01, 0x00, 0x00, 0x00, 0x02};
	HAL_UART_Transmit(&huart2, cmd, sizeof(cmd), 1000);
}

void CameraFocusFar(void)
{
	uint8_t cmd[]={0xff, 0x01, 0x00, 0x80, 0x00, 0x00, 0x81};
	HAL_UART_Transmit(&huart2, cmd, sizeof(cmd), 1000);
}

void CameraPhoto(void)
{
	uint8_t cmd[]={0xff, 0x01, 0x00, 0x07, 0x00, 0xe1, 0xe9};
	HAL_UART_Transmit(&huart2, cmd, sizeof(cmd), 1000);
}

void CameraRecordStart(void)
{
	uint8_t cmd[]={0xff, 0x01, 0x00, 0x07, 0x00, 0xe2, 0xea};
	HAL_UART_Transmit(&huart2, cmd, sizeof(cmd), 1000);
}

void CameraRecordStop(void)
{
	uint8_t cmd[]={0xff, 0x01, 0x00, 0x07, 0x00, 0xe3, 0xeb};
	HAL_UART_Transmit(&huart2, cmd, sizeof(cmd), 1000);
}

void CameraRecordInit(void)
{
	uint8_t cmd[]={0xff, 0x01, 0x00, 0x07, 0x00, 0xdf, 0xe7};
	HAL_UART_Transmit(&huart2, cmd, sizeof(cmd), 1000);
}

void CameraZoomPos(uint8_t *data, uint8_t len)
{
	uint8_t cmd[]={0xff, 0x01, 0x04, 0x47, 0x00, 0x00, 0x00, 0x00, 0xFF};
	cmd[4] = data[2] >> 4;
	cmd[5] = data[2] & 0x0f;
	cmd[6] = data[3] >> 4;
	cmd[7] = data[4] & 0x0f;	
	HAL_UART_Transmit(&huart2, cmd, sizeof(cmd), 1000);
}

void CameraExpCompOn(void)
{
	uint8_t cmd[]={0xff, 0x01, 0x04, 0x3e, 0x02, 0xFF};
	HAL_UART_Transmit(&huart2, cmd, sizeof(cmd), 1000);
}

void CameraExpCompOff(void)
{
	uint8_t cmd[]={0xff, 0x01, 0x04, 0x3e, 0x03, 0xFF};
	HAL_UART_Transmit(&huart2, cmd, sizeof(cmd), 1000);
}

void CameraExpCompReset(void)
{
	uint8_t cmd[]={0xff, 0x01, 0x04, 0x0e, 0x00, 0xFF};
	HAL_UART_Transmit(&huart2, cmd, sizeof(cmd), 1000);
}

void CameraExpCompUp(void)
{
	uint8_t cmd[]={0xff, 0x01, 0x04, 0x0e, 0x02, 0xFF};
	HAL_UART_Transmit(&huart2, cmd, sizeof(cmd), 1000);
}

void CameraExpCompDown(void)
{
	uint8_t cmd[]={0xff, 0x01, 0x04, 0x0e, 0x03, 0xFF};
	HAL_UART_Transmit(&huart2, cmd, sizeof(cmd), 1000);
}

void CameraExpCompDirect(uint8_t *data, uint8_t len)
{
	uint8_t cmd[]={0xff, 0x01, 0x04, 0x4e, 0x00, 0x00, 0x00, 0x00, 0xFF};
	cmd[6] = data[2] >> 4;
	cmd[7] = data[2] & 0x0f;	
	HAL_UART_Transmit(&huart2, cmd, sizeof(cmd), 1000);
}

void CameraMonitoeMode(uint8_t *data, uint8_t len)
{
	uint8_t cmd[]={0xff, 0x01, 0x04, 0x24, 0x72, 0x00, 0x00, 0xFF};
	cmd[5] = data[2] >> 4;
	cmd[6] = data[2] & 0x0f;	
	HAL_UART_Transmit(&huart2, cmd, sizeof(cmd), 1000);
}

float Q12ToFloat(uint16_t in)
{
	float	out;
	
	out = (in & 0x0fff) * (1 / 0x1000);
	out += (in << 1) >> 13;
	if (in >> 15)
	{
		out *= -1;
	}
	
	return out;
}

void YuntaiYawPitch(uint8_t *data, uint8_t len)
{
	float yaw = Q12ToFloat(((uint16_t)data[2]) << 8 | data[3]);
	float pitch = Q12ToFloat(((uint16_t)data[4]) << 8 | data[5]);
	uint8_t yaw_mode = data[6] >> 4;
	uint8_t pitch_mode = data[6] | 0x0f;
	
	gimbal_control_standard_send((mavlink_channel_t)0, 20, yaw_mode, pitch_mode, 4, yaw, pitch, 0.0, 0.0);
}

void CANDataProcess(uint8_t *data, uint8_t len)
{
		if (*data == DEVICE_CAMERA)
		{
				switch(data[1])
				{
					case FUNC_CAMERA_STOP:
						CameraStop();
						break;
					
					case FUNC_CAMERA_FOCUS_NEAR:
						CameraFocusNear();
						break;		
					
					case FUNC_CAMERA_FOCUS_FAR:
						CameraFocusFar();
						break;	
					
					case FUNC_CAMERA_PHOTO:
						CameraPhoto();
						break;	
					
					case FUNC_CAMERA_RECORD_START:
						CameraRecordStart();
						break;	
					
					case FUNC_CAMERA_RECORD_STOP:
						CameraRecordStop();
						break;	
					
					case FUNC_CAMERA_INIT:
						CameraRecordInit();
						break;	
					
					case FUNC_CAMERA_ZOOM_POS:
						CameraZoomPos(data, len);
						break;	
					
					case FUNC_CAMERA_EXP_COMP_ON:
						CameraExpCompOn();
						break;
					
					case FUNC_CAMERA_EXP_COMP_OFF:
						CameraExpCompOff();
						break;
					
					case FUNC_CAMERA_EXP_COMP_RESET:
						CameraExpCompReset();
						break;	
					
					case FUNC_CAMERA_EXP_COMP_UP:
						CameraExpCompUp();
						break;	
					
					case FUNC_CAMERA_EXP_COMP_DOWN:
						CameraExpCompDown();
						break;	
					
					case FUNC_CAMERA_EXP_COMP_DIRECT:
						CameraExpCompDirect(data, len);
						break;
					
					case FUNC_CAMERA_EXP_MONITOR_MODE:
						CameraMonitoeMode(data, len);
						break;	
					
					default:
						break;
				}
		}
		else if(*data == DEVICE_YUNTAI)
		{
				if (data[1] == FUNC_YUNTAI_YAW_PITCH)
				{
						YuntaiYawPitch(data, len);
				}
		}
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
						CANDataProcess(hcan.pRxMsg->Data, hcan.pRxMsg->DLC);
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

		uint32_t StdId =0x200; 
	
		sFilterConfig.FilterNumber = 0;
		sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
		sFilterConfig.FilterIdHigh = ((((StdId <<21)&0xFFFF0000)>>16));
		sFilterConfig.FilterIdLow = (((StdId<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xFFFF); 
		sFilterConfig.FilterMaskIdHigh = 0xFFFF;
		sFilterConfig.FilterMaskIdLow = 0xFFFF;
		sFilterConfig.FilterFIFOAssignment = 0;
		sFilterConfig.FilterActivation = ENABLE;
		sFilterConfig.BankNumber = 14;
	
		if (HAL_CAN_ConfigFilter(CanHandle, &sFilterConfig) != HAL_OK)
		{
				Error_Handler();
		}
		
		CanFill(&hcan);
		
		if (HAL_CAN_Receive_IT(&hcan, CAN_FIFO0) != HAL_OK)
		{
				Error_Handler();
		}
}

void main_DTARepeater(void)
{
		CanPreProcess(&hcan);
	
		HAL_Delay(3000);
		CameraReset();
		
		HAL_IWDG_Start(&hiwdg);
		while (1)
		{
				HAL_Delay(1000);	
				HAL_IWDG_Refresh(&hiwdg);
		}
}
