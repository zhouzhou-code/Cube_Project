#include "servo.h"

/*
*@breif  设置舵机的角度
*@param  Angle舵机角度  取值为0~180
* 
*/
void Set_Servo_Angle(float Angle)
{
   if(Angle<0) Angle=0;
	 if(Angle>180) Angle=180;
   SetPWMDuty(&htim2,TIM_CHANNEL_1,Angle / 180 * 2000 + 500);
	
}




