#include "greys.h"

uint16_t grey_Out_Read[5]={0};//保存红外对管的状态

float grey_pid_Out;//红外对管PID计算输出速度
float grey_pid_Out1;//电机1的循迹PID控制速度
float grey_pid_Out2;//电机2的循迹PID控制速度


void greys_Init(void)//灰度IO初始化，用cubeMX完成
{
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
//	
//	GPIO_InitTypeDef GPIO_InitStruct;
//  GPIO_InitStruct.GPIO_Mode= GPIO_Mode_IN_FLOATING;
//	GPIO_InitStruct.GPIO_Pin= GPIO_Pin_4 | GPIO_Pin_5| GPIO_Pin_8;
//  GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA,&GPIO_InitStruct);
//	
//	GPIO_InitStruct.GPIO_Pin= GPIO_Pin_0 | GPIO_Pin_1;
//	GPIO_Init(GPIOB,&GPIO_InitStruct);
}

/*
* @brief:通过参数选择是左优先还是右优先寻迹
* @param：WhatPriority 0表示左优先，1表示右优先
* @return:左右轮的差速值
*/
int Priority_Track(uint8_t WhatPriority)//左/右优先循迹
{  
    static int8_t ThisState;
    static int8_t LastState;
	float output_max = 50;//限幅最大值
	grey_Out_Read[0]=grey5;//读取红外对管状态  排布： 4 3 2 1 0
	grey_Out_Read[1]=grey4;
	grey_Out_Read[2]=grey3;
	grey_Out_Read[3]=grey2;
	grey_Out_Read[4]=grey1; 
	int flag=2;          //默认为2，直线的情况
	
	if(WhatPriority==0)//左优先循迹
	{
		for(int i=4;i>=0;i--)//从左向右读取红外对管状态，根据第一个探到黑线的红外对管，返回值
		{
			if(grey_Out_Read[i]==1)
			{
				flag=i;
				break;//查找到第一个探测到黑线的红外对管的编号后马上退出循环
			}		
		}
	}
	
	else if(WhatPriority==1) //右优先循迹
	{
		for(int i=0;i<5;i++)//从右向左读取红外对管状态，根据第一个探到黑线的红外对管，返回值
		{
			if(grey_Out_Read[i]==1)
			{
				flag=i;
				break;//查找到第一个探测到黑线的红外对管的编号后马上退出循环
			}		
		}
	
	}
  
	//根据不同的状态，返回不同值
	switch(flag)
		{
			case 4:
				ThisState=-10;break;
			case 3:
				ThisState=-3;break;
			case 2:
				ThisState=0;break;
			case 1:
				ThisState=3;break;
			case 0:
				ThisState=10;break;	
			
		}
		
	grey_pid_Out=PID_Position(&pidTracking,ThisState);
	//输出限幅
	if(grey_pid_Out>output_max)grey_pid_Out=output_max;
	if(grey_pid_Out<-output_max)grey_pid_Out=-output_max;
		
	LastState=ThisState;//保存上次状态
		
	return grey_pid_Out;
	
}

int16_t Public_Track(void)//列举有限情况寻迹
{
    
	/************************************************************************************************
	“小车状态”这个系统的输入是“两轮速度（差）”，目标是在黑线中央，其他实际状态是偏差状态,通过给不同的
	偏差状态赋值，将得到的偏差输入到循迹PID，循迹PID的输出作为速度环PID的输入
	*************************************************************************************************/
	static int8_t ThisState;
  static int8_t LastState;  
	grey_Out_Read[0]=grey5;//读取红外对管状态
	grey_Out_Read[1]=grey4;
	grey_Out_Read[2]=grey3;
	grey_Out_Read[3]=grey2;
	grey_Out_Read[4]=grey1;
	float output_max = 40;//限幅最大值
	
	                        //给小车的偏差状态赋值（没探到黑线，灯亮，返回0）
	if(grey_Out_Read[0]==0&&grey_Out_Read[1]==0&&grey_Out_Read[2]==1&&grey_Out_Read[3]==0&&grey_Out_Read[4]==0)  //00 1 00
	  ThisState=0;//应该前进
	
	else if(grey_Out_Read[0]==0&&grey_Out_Read[1]==1&&grey_Out_Read[2]==0&&grey_Out_Read[3]==0&&grey_Out_Read[4]==0)//01 0 00
		ThisState=1;//应该左转   轻微右偏		
     
	else if(grey_Out_Read[0]==1&&grey_Out_Read[1]==0&&grey_Out_Read[2]==0&&grey_Out_Read[3]==0&&grey_Out_Read[4]==0)//10 0 00
	  ThisState=2;//快速左转  中度右偏
	
	else if(grey_Out_Read[0]==1&&grey_Out_Read[1]==1&&grey_Out_Read[2]==1&&grey_Out_Read[3]==0&&grey_Out_Read[4]==0)// 11 1 00
	  ThisState=7;//快速左转   （小车到达左转弯处）

	else if(grey_Out_Read[0]==1&&grey_Out_Read[1]==1&&grey_Out_Read[2]==0&&grey_Out_Read[3]==0&&grey_Out_Read[4]==0)// 11 0 00
	  ThisState=10;//更快速左转  （小车超过左转弯处）
	
	else if(grey_Out_Read[0]==0&&grey_Out_Read[1]==0&&grey_Out_Read[2]==0&&grey_Out_Read[3]==1&&grey_Out_Read[4]==0)//00 0 10
		ThisState=-1;//应该右转    轻微左偏		
     
	else if(grey_Out_Read[0]==0&&grey_Out_Read[1]==0&&grey_Out_Read[2]==0&&grey_Out_Read[3]==0&&grey_Out_Read[4]==1)//00 0 01
	  ThisState=-2;//快速右转    中度左偏
	
	else if(grey_Out_Read[0]==0&&grey_Out_Read[1]==0&&grey_Out_Read[2]==1&&grey_Out_Read[3]==1&&grey_Out_Read[4]==1)// 00 1 11
	  ThisState=-7;//快速右转   （小车到达右转弯处）
	
	else if(grey_Out_Read[0]==0&&grey_Out_Read[1]==0&&grey_Out_Read[2]==0&&grey_Out_Read[3]==1&&grey_Out_Read[4]==1)// 00 0 11
	  ThisState=-10;//更快速右转   （小车超过右转弯处）
	
	
	//PID计算输出目标速度 ，“小车状态”这个系统的输入是两轮的速度（差），PID输出作用于两轮的期望速度上
	grey_pid_Out=PID_Position(&pidTracking,ThisState);
	//输出限幅
	if(grey_pid_Out>output_max)grey_pid_Out=output_max;
	if(grey_pid_Out<-output_max)grey_pid_Out=-output_max;
	
	LastState=ThisState;//保存上次状态
	
	return grey_pid_Out;//不能直接输出期望速度，因为要和其他PID输出并起来
	
}

/*
* @brief:通过灰度状态检测是否发生了状态切换
* @param：SwitchCounts 记录状态切换的次数
* @return:void
*/
void grey_state(uint8_t* SwitchCounts)
{
    static uint8_t ThisState=0;
    static uint8_t LastState=0;
    static uint8_t flag=0;
    static uint8_t  CNT=0; //连续状态计数器
    static uint8_t  N=15;//连续N次都是...状态
    
   	grey_Out_Read[0]=grey5;//读取红外对管状态
	grey_Out_Read[1]=grey4;
	grey_Out_Read[2]=grey3;
	grey_Out_Read[3]=grey2;
	grey_Out_Read[4]=grey1;

   if(grey_Out_Read[0]||grey_Out_Read[1]||grey_Out_Read[2]||grey_Out_Read[3]||grey_Out_Read[4])
      ThisState=1;//如果某一个灰度探到了黑线(返回高电平),令此次状态为1
   else
      ThisState=0; //否则令此次状态为0
    
   switch(flag)
   {
    case 0://无线时，判断是否连续N次都识别到了黑线
    {
      if(ThisState==1)//如果本次识别到了黑线,计数值加1
      {
         CNT++;
         LastState=ThisState;//将本次状态赋值为上次状态
         if(CNT>=N)
	     {
	       CNT=0;//重置计数器
           flag=1;//连续N次识别到了黑线，小车即将巡线，用另一个case判断
           (*SwitchCounts)++;//记录切换的次数
	     }
      }

      if(LastState!=ThisState)//如果相邻两次状态不同，说明不是连续的1,有噪声
	  {
		 CNT=0;//重置计数器
		 LastState=ThisState;//将本次状态赋值为上次状态
	  }
      break;
    }


    case 1://有线时判断是否连续N次都是没识别到黑线
    {
	if(ThisState==0)//如果本次识别到了无黑线,计数值加1
    {
       CNT++;
       LastState=ThisState;//将本次状态赋值为上次状态
       if(CNT>=N)
       {
	  	  CNT=0;//将CNT清零，为下次判断做准备
	  	  flag=0;//连续N次识别到了无黑线，小车即将陀螺仪导航,用另一个case判断
	  	  (*SwitchCounts)++;//记录切换的次数
       }
       
    }
  
    if(LastState!=ThisState)//如果相邻两次状态不同，说明不是连续的0,有噪声
    {
  	 CNT=0;//重置计数器
  	 LastState=ThisState;//将本次状态赋值为上次状态
    }
	break;

    }


   }

}
