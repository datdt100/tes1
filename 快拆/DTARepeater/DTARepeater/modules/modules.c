#include "modules.h"
#include "main.h"
#include "stdint.h"
#include "string.h"

extern TIM_HandleTypeDef htim1;

static volatile uint32_t tim1_ms=0;
void Inc_TIM()
{
	tim1_ms++;
}
uint32_t Get_TIM()
{
	return tim1_ms;
}

Ecap ecap;
#if 1
void get_pwm(GPIO_TypeDef *GPIOx,uint16_t gpio_pin,uint16_t index,uint32_t value)
{
    if(HAL_GPIO_ReadPin(GPIOx,gpio_pin)==GPIO_PIN_SET)
    {
        ecap.rise[index]=value;
    }
    else
    {
        if(value>ecap.rise[index])
            ecap.pwm[index]=value-ecap.rise[index];
        else
            ecap.pwm[index]=4000+value-ecap.rise[index];
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	__disable_irq();
    uint32_t value=htim1.Instance->CNT;
    switch(GPIO_Pin)
    {
        case GPIO_PIN_8:
            get_pwm(GPIOA,GPIO_Pin,0,value);
            break;
        case GPIO_PIN_9:
            get_pwm(GPIOA,GPIO_Pin,1,value);
            break;
        case GPIO_PIN_10:
            get_pwm(GPIOA,GPIO_Pin,2,value);
            break;
        case GPIO_PIN_11:
            get_pwm(GPIOA,GPIO_Pin,3,value);
            break;
        case GPIO_PIN_6:
            get_pwm(GPIOA,GPIO_Pin,4,value);
            break;
        case GPIO_PIN_7:
            get_pwm(GPIOA,GPIO_Pin,5,value);
            break;
        case GPIO_PIN_0:
            get_pwm(GPIOB,GPIO_Pin,6,value);
            break;
        case GPIO_PIN_1:
            get_pwm(GPIOB,GPIO_Pin,7,value);
            break;
    }
    __enable_irq();
}



#else
void Init_Ecap() __enable_irq()
{
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_4);

	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_4);

	memset(&ecap,0,sizeof(ecap));
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	uint32_t value;
	if(htim==&htim1)
	{

		switch(htim->Channel)
		{
			case HAL_TIM_ACTIVE_CHANNEL_1:
				value=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
				if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8)==GPIO_PIN_SET)
				{
					ecap.rise[0]=value;
				}else
				{
					if(value>ecap.rise[0])
						ecap.fail[0]=value-ecap.rise[0];
					else
						ecap.fail[0]=value-ecap.rise[0]+4000;
				}
				break;
			case HAL_TIM_ACTIVE_CHANNEL_2:
				value=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
				if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9)==GPIO_PIN_SET)
				{
					ecap.rise[1]=value;
				}else
				{
					ecap.fail[1]=value;
				}
				break;
			case HAL_TIM_ACTIVE_CHANNEL_3:
				value=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_3);
				if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_10)==GPIO_PIN_SET)
				{
					ecap.rise[2]=value;
				}else
				{
					ecap.fail[2]=value;
				}
				break;
			case HAL_TIM_ACTIVE_CHANNEL_4:
				value=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_4);
				if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11)==GPIO_PIN_SET)
				{
					ecap.rise[3]=value;
				}else
				{
					ecap.fail[3]=value;
				}
				break;
			default:
					break;
				
		}
	}
	else if(htim==&htim3)
	{
		value=HAL_TIM_ReadCapturedValue(htim,htim->Channel);
		switch(htim->Channel)
		{
			case HAL_TIM_ACTIVE_CHANNEL_1:
						value=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
				if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)==GPIO_PIN_SET)
				{
					ecap.rise[4]=value;
				}else
				{
					ecap.fail[4]=value;
				}
				break;
			case HAL_TIM_ACTIVE_CHANNEL_2:
						value=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
				if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)==GPIO_PIN_SET)
				{
					ecap.rise[5]=value;
				}else
				{
					ecap.fail[5]=value;
				}
				break;
			case HAL_TIM_ACTIVE_CHANNEL_3:
				value=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_3);
				if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==GPIO_PIN_SET)
				{
					ecap.rise[6]=value;
				}else
				{
					ecap.fail[6]=value;
				}
				break;
			case HAL_TIM_ACTIVE_CHANNEL_4:
						value=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_4);
				if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==GPIO_PIN_SET)
				{
					ecap.rise[7]=value;
				}else
				{
					ecap.fail[7]=value;
				}
				break;
			default:
				break;
				
		}
	}
	
}
#endif


uint16_t Get_channel(uint16_t channel)
{
    if(channel >= 8)
    {
        return 0;
    }
    
    uint32_t value = ecap.pwm[channel];
    if(value < 988)
    {
        value = 988;
    }
    else if(value > 2012)
    {
        value = 2012;
    }

    return value;
}