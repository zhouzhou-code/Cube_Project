#include "pid.h"

//定义结构体变量
Pid_t pidMPU6050;
Pid_t pidTracking;
//给结构体变量赋初始值
void PID_Init(void)  
{
//	pid->actual_val=0;
//	pid->target_val=0;
//	pid->err=0;
//	pid->err_last=0;
//	pid->err_sum=0;
//	pid->Kd=0;
//	pid->Ki=0;
//	pid->Kp=0;
	pidMPU6050.actual_val=0;
  pidMPU6050.target_val=0;
	pidMPU6050.err_current=0;
	pidMPU6050.err_last=0;
	pidMPU6050.err_sum=0;
	pidMPU6050.Kp=0;
	pidMPU6050.Ki=0;
	pidMPU6050.Kd=0;
	
	pidTracking.actual_val=0;
  pidTracking.target_val=0;
	pidTracking.err_current=0;
	pidTracking.err_last=0;
	pidTracking.err_sum=0;
	pidTracking.Kp=3.6;
	pidTracking.Ki=0;
	pidTracking.Kd=0;
	pidTracking.err_sum_max=10;
}
/*********************************************************************************************
                                       位置式PID
**********************************************************************************************/
float PID_Position(Pid_t* pid,float actual_val)
{
	pid->actual_val=actual_val;//传递真实值
	pid->err_current=pid->target_val - pid->actual_val;            //当前误差＝目标值-真实值
	pid->err_sum+=pid->err_current;                                //累计误差=当前误差累计和
	I_Xianfu(	pid->err_sum,pid->err_sum_max);                      //积分项限幅
  //输出=Kp*当前误差+Ki*累计误差+Kd*两次误差之差
	pid->output_val=pid->Kp*pid->err_current + pid->Ki*pid->err_sum +pid->Kd*(pid->err_current-pid->err_last);
	pid->err_last=pid->err_current;                                //本次误差作为下次的“上次误差”
	
	return pid->output_val;
}

/*********************************************************************************************
                                       增量式PID
**********************************************************************************************/
float PID_Incremental(Pid_t* pid,float actual_val)
{
	pid->actual_val=actual_val;//传递真实值
	pid->err_current=pid->target_val - pid->actual_val;//计算当前误差
	pid->output_val+=pid->Kp*(pid->err_current-pid->err_last)+ pid->Ki*pid->err_current+ pid->Kd*(pid->err_current-2*pid->err_last+pid->err_previous);
	//增量式PID公式
	pid->err_previous=pid->err_last;//更新前次误差
	pid->err_last=pid->err_current;//更新上次误差
	
	return pid->output_val;
}

void I_Xianfu(float err_sum,int err_sum_max)//对积分项I进行限幅(位置式PID需要)
{
  if(err_sum>err_sum_max) err_sum=err_sum_max;
  if(err_sum<-err_sum_max) err_sum=-err_sum_max;
}
/*********************************************************************************************
                             拓展:根据串口调参
**********************************************************************************************/


float Get_Data_fromVOFA(const uint8_t *DataBuff)
{
	float data_return = 0.0f;
	
	sscanf((const char*)DataBuff, "%*[^=]=%f!", &data_return);
	return data_return;
}

/*
* @param1：Pid_t *pid PID变量的指针
* @param2：保存vofa+上位机发的调参数据的缓冲数组DataBuff
* @param3: 
*/
void Update_PID_Params( Pid_t *pid,const uint8_t *DataBuff)
{
	float data_Get=Get_Data_fromVOFA(DataBuff);
	
	switch (DataBuff[0]) {
		case 'P':
			pid->Kp = data_Get;
			break;
		case 'I':
			pid->Ki = data_Get;
			break;
		case 'D':
			pid->Kd = data_Get;
			break;
		case 'T':
			pid->target_val = data_Get;
			break;
		default:
			// 未知字符，不做处理
			break;
	}
}

//void USART_PID_Adjust(const uint8_t *DataBuff,Pid_t *pid) 
//{
//    if (DataBuff != NULL && DataBuff[0] != '\0')
//    {
//        float data_Get = Get_Data_fromVOFA(DataBuff);//调用函数获得浮点数数据
//        Update_PID_Params(pid, DataBuff, data_Get);
//    }
//}



