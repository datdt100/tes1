#ifndef modules_h
#define modules_h
#include "stdint.h"
#include "main.h"


typedef struct
{
	uint32_t rise[8];
	uint32_t pwm[8];
}Ecap;

typedef struct
{
	uint32_t ms;
	uint32_t us;
}Timestamp;
void Inc_TIM();
extern Ecap ecap;

uint16_t Get_channel(uint16_t channel);
#endif


