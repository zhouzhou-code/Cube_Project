#include "motor.h"


void vMotorSetPWM(int16_t motor1Speed,int16_t motor2Speed)
{
	if(motor1Speed>100) motor1Speed=100;
	if(motor1Speed<-100) motor1Speed=-100;
	
	if(motor1Speed>0)
	{
	  AIN1(1);
		AIN2(0);
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,motor1Speed);
		
	}
	else
	{
		AIN1(0);
		AIN2(1);
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,-motor1Speed);
	
	}
	
	if(motor2Speed>0)
	{
	  BIN1(1);
		BIN2(0);
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2,motor2Speed);
		
	}
	else
	{
		BIN1(0);
		BIN2(1);
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2,-motor2Speed);
	
	}
	

}
