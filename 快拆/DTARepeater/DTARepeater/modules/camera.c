#include "camera.h"
#include "string.h"
#include "main.h"

extern UART_HandleTypeDef huart2;//camera 115200
uint8_t OSD[48];
uint8_t cmd_long[20];
int Posi_Step = 0x4000 / 256;
void Init()
{
	memset(OSD,0,48*sizeof(uint8_t));
	memset(OSD,0,20*sizeof(uint8_t));
}

void Set_Time(uint16_t year,uint8_t month,uint8_t day,uint8_t hour,uint8_t min,uint8_t sec)
{
	uint8_t sum=0;
	uint8_t i=0;
	OSD[0]=0x7E;
	OSD[1]=0x7E;//
	OSD[2]=0x44;
	OSD[3]=0xd6;
	OSD[4]=0x00;////////
	OSD[5]=0x83;
	OSD[6]=0x02;
	OSD[7]=year&0xFF;
	OSD[8]=0x07;
	OSD[9]=month;
	OSD[10]=day;
	OSD[11]=hour;
	OSD[12]=min;
	OSD[13]=sec;
	for(i=0;i<47;i++)
		sum+=OSD[i];
	
	OSD[47]=sum;
	HAL_UART_Transmit_IT(&huart2,OSD,48);
}

void combine_long_cmd_control(int16_t pitch,int16_t roll,int16_t yaw)
{
		uint8_t sum=0;
		uint8_t i=0;
		uint8_t *angle=&cmd_long[7];

		pitch=50*pitch;
		roll =50*roll;
		yaw  =50*yaw;

		cmd_long[0]=0xFF;
		cmd_long[1]=0x01;
		cmd_long[2]=0x0F;
	  cmd_long[3]=0x10;

		cmd_long[4]=0x05;
	  cmd_long[5]=0x05;
	  cmd_long[6]=0x05;

		*angle++=0;
		*angle++=0;

		*angle++=roll&0xFF;
		*angle++=roll>>8;

		*angle++=0;
		*angle++=0;

		*angle++=pitch&0xFF;
		*angle++=pitch>>8;

		*angle++=0;
		*angle++=0;

		*angle++=yaw&0xFF;
		*angle++=yaw>>8;
		for(i=4;i<19;i++)
			sum+=cmd_long[i];

		cmd_long[19]=sum;
		HAL_UART_Transmit_IT(&huart2,cmd_long,20);
}

void Reset()
{
	uint8_t cmd[]={0x81,0x01,0x04,0x19,0x03,0xFF}; //6
	HAL_UART_Transmit(&huart2,cmd,6,1000);
}

void Start_rec()
{
	uint8_t cmd[]={0xFF,0x01,0x00,0x07,0x00,0xE2,0xEA}; //7
	HAL_UART_Transmit(&huart2,cmd,7,1000);
}

void Stop_rec()
{
	uint8_t cmd[]={0xFF,0x01,0x00,0x07,0x00,0xE3,0xEB}; //7
	HAL_UART_Transmit(&huart2,cmd,7,1000);
}

void Take_Pic()
{
	uint8_t cmd[]={0xFF,0x01,0x00,0x07,0x00,0xE1,0xE9}; //7
	HAL_UART_Transmit(&huart2,cmd,7,1000);
}


void Stabilization_On()
{
	uint8_t cmd[]={0x81,0x01,0x04,0x34,0x02,0xFF}; //6
	HAL_UART_Transmit(&huart2,cmd,6,1000);
}

void Stabilization_Off()
{
	uint8_t cmd[]={0x81,0x01,0x04,0x34,0x03,0xFF}; //6
	HAL_UART_Transmit(&huart2,cmd,6,1000);
}

void Stabilization_Hold()
{
	uint8_t cmd[]={0x81,0x01,0x04,0x34,0x03,0xFF}; //6
	HAL_UART_Transmit(&huart2,cmd,6,1000);
}

void set_wide_open()
{
	uint8_t cmd[]={0xFF,0x01,0x00,0x07,0x00,0xF3,0xFB}; //6
	HAL_UART_Transmit(&huart2,cmd,7,1000);
}

void set_wide_close()
{
	uint8_t cmd[]={0xFF,0x01,0x00,0x07,0x00,0xF4,0xFC}; //6
	HAL_UART_Transmit(&huart2,cmd,7,1000);
}

void set_3d_close()
{
	uint8_t cmd[]={0xFF,0x01,0x00,0x07,0x00,0xF5,0xFD}; //6
	HAL_UART_Transmit(&huart2,cmd,7,1000);
}

void set_saved()
{
	uint8_t cmd[]={0xFF,0x01,0x00,0x07,0x00,0xF9,0x01}; //6
	HAL_UART_Transmit(&huart2,cmd,7,1000);
}

int Get_Tele_Code(uint16_t num)
{
    return Posi_Step * num;
}

void Tele_30(uint16_t num)
{
    int posi = Get_Tele_Code(num);

    uint8_t P = (uint8_t)((posi & 0xF000) >> 12);
    uint8_t Q = (uint8_t)((posi & 0x0F00) >> 8);
    uint8_t R = (uint8_t)((posi & 0x00F0) >> 4);
    uint8_t S = (uint8_t)(posi & 0x000F);

	uint8_t cmd[]={0x81,0x01,0x04,0x47,P,Q,R,S,0xFF};
    HAL_UART_Transmit(&huart2,cmd,9,1000);
}

