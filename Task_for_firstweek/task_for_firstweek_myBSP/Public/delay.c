#if 0

#include "Delay.h"

//初始化延迟函数
//SYSTICK 的时钟固定为 AHB 时钟，基础例程里面 SYSTICK 时钟频率为 AHB/8
//这里为了兼容 FreeRTOS，所以将 SYSTICK 的时钟频率改为 AHB 的频率！
//SYSCLK:系统时钟频率
void delay_init()
{
	u32 reload;
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);//选择外部时钟 HCLK
	fac_us=SystemCoreClock/1000000; //不论是否使用 OS,fac_us 都需要使用
	reload=SystemCoreClock/1000000; //每秒钟的计数次数 单位为 M
	reload*=1000000/configTICK_RATE_HZ; //根据 configTICK_RATE_HZ 设定溢出
	//时间 reload 为 24 位寄存器,最大值:
	//16777216,在 72M 下,约合 0.233s 左右
	fac_ms=1000/configTICK_RATE_HZ; //代表 OS 可以延时的最少单位
	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk; //开启 SYSTICK 中断
	SysTick->LOAD=reload; //每 1/configTICK_RATE_HZ 秒中断
	//一次
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; //开启 SYSTICK
}



#include "Delay.h"

static uint32_t  fac_us=0;							//1us计数fac_us个数		   

/*******************************************************************************
* 函 数 名:          SysTick_Init
* 函数功能:          SysTick初始化函数
* 输    入:          SYSCLK:系统时钟频率
* 输    出:          无
*******************************************************************************/
void SysTick_Init(uint8_t SYSCLK)
{
 	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);//SysTick频率为HCLK 
	fac_us=SYSCLK;						//保存1us所需的计数次数  
}								    

/*************************************************
*函数名：            Delay_us
*函数功能：          微秒级延时函数
*输入：              nus：延时nus微秒
                     注意：nus的取值为0~190887435(最大值即2^32/fac_us@fac_us=22.5)
*返回值：            无
**************************************************/	 
void Delay_us(uint32_t nus)
{		
	uint32_t ticks;
	uint32_t told,tnow,tcnt=0;
	uint32_t reload=SysTick->LOAD;				//LOAD的值	    	 
	ticks=nus*fac_us; 						    //需要的节拍数 
	told=SysTick->VAL;        				//刚进入时的计数器值
	while(1)
	{
		tnow=SysTick->VAL;	//在while中不断获取Val值
		if(tnow!=told)      
		{	    
			if(tnow<told)tcnt+=told-tnow;	//这里注意一下SYSTICK是一个递减的计数器就可以了.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			//时间超过/等于要延迟的时间,则退出.
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
#else


//使用TIM7来进行精确地延时(SysTick为啥用不了?用作FreeRTOS时钟也应该能读取load,val等值来进行延时啊)
#include "Delay.h"
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
	uint32_t reload=TIM7->ARR;				     //ARR的值(记录自动重装值)
	ticks=nus*fac_us; 						         //需要的节拍数(需要计数多少次) 
	told=TIM7->CNT;      				           //刚进入时的计数器值
	while(1)
	{
		tnow=TIM7->CNT;	                    //在while中不断获取CNT值
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

#endif
