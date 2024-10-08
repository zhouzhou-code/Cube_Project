#ifndef __PID_H__
#define __PID_H__

#include "main.h"//包含了标准库函数和hal库定义
#include "usart.h"//包含了从VOFA+获取浮点数据的函数

//PID控制器结构体
typedef struct
{
	float target_val;//目标值
	float actual_val;//实际值
	float err_current;//当前误差
	float err_sum;//累计误差值
	float err_last;//上次误差
	float err_previous;//上上次误差(增量式PID用)
	float Kp,Ki,Kd;//PID系数
	float output_val;//PID输出值
	int err_sum_max;//I限幅(位置式PID用)
} Pid_t;
 //声明函数
void PID_Init(void);
float PID_Position(Pid_t* pid,float actual_val);
float PID_Incremental(Pid_t* pid,float actual_val);
void I_Xianfu(float err_sum,int err_sum_max);
float Get_Data_fromVOFA(const uint8_t *DataBuff);
void Update_PID_Params( Pid_t *pid,const uint8_t *DataBuff);
//void Update_PID_Params( Pid_t *pid,const uint8_t *DataBuff,float data_Get);
//void USART_PID_Adjust(const uint8_t *DataBuff,Pid_t *pid) ;


//自己的.c文件定义的全局变量在此处声明
extern Pid_t pidMPU6050;
extern Pid_t pidTracking;
#endif
