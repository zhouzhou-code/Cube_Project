//使用TIM3来进行精确地延时(SysTick为啥用不了?用作FreeRTOS时钟也应该能读取load,val等值来进行延时啊)
#include "delay.h"
static uint32_t  fac_us=72;							//1us计数fac_us个数(时钟为72Mhz,1us计72个数)		 

/*************************************************
*函数名：            Delay_us
*函数功能：          微秒级延时函数
*输入：              nus：延时nus微秒
注意：nus的取值为0~190887435(最大值即2^32/fac_us@fac_us=22.5),不会导致任务切换
*返回值：            无
**************************************************/	 
void Delay_us(uint32_t nus)
{		
	uint32_t ticks;
	uint32_t told,tnow,tcnt=0;             //旧的时间，现在的时间，计数值
	uint32_t reload=TIM3->ARR;				     //ARR的值(记录自动重装值)
	ticks=nus*fac_us; 						         //需要的节拍数(需要计数多少次) 
	told=TIM3->CNT;      				           //刚进入时的计数器值
	while(1)
	{
		tnow=TIM3->CNT;	                    //在while中不断获取CNT值
		if(tnow!=told)                       //CNT值发生改变     
		{	    
			if(tnow>told)tcnt+=tnow-told;	     //这里注意一下TIM7是一个递增的计数器就可以了
			else tcnt+=reload-(told-tnow); 	   //重装了，CNT发生了越界 
			told=tnow;
			if(tcnt>=ticks)break;			         //时间超过/等于要延迟的时间,则退出.
		}  
	};
}

/*************************************************
*函数名：            Delay_ms
*函数功能：          毫秒级延时函数
*输入：              nus：延时nus毫秒
*返回值：            无
**************************************************/	
void Delay_ms(uint16_t nms)
{
	uint32_t i;
	for(i=0;i<nms;i++) Delay_us(1000);
}


void delay_ms(uint16_t nms)
{
   Delay_ms(nms);
}
void delay_us(uint32_t nus)
{
   Delay_us(nus);
}

