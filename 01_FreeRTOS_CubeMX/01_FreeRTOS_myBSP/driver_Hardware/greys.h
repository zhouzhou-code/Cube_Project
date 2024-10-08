/***************************************************************************************
																	红外/灰度循迹
									 此模块用到的io口是   PA4,PA5,PA8，PB0,PB1
***************************************************************************************/

#ifndef __HW_TRACKING_H__
#define __HW_TRACKING_H__

#include "main.h"
#include "string.h" 
#include  "sys.h"
#include "PID.h"
#include "motor.h"

void greys_Init(void);//灰度IO初始化，用cubeMX完成
int Priority_Track(uint8_t WhatPriority);//左/右优先循迹
int16_t Public_Track(void);//列举有限情况的寻迹
void grey_state(uint8_t* SwitchCounts);//通过灰度状态检测是否发生了状态切换，并记录状态切换的次数




//宏定义红外对管需要的IO口
#define grey1   PBin(0)
#define grey2   PBin(1)
#define grey3   PAin(4)
#define grey4   PAin(5)
#define grey5   PAin(6)

#define HW_R2    grey_Out_Read[0] 
#define HW_R1    grey_Out_Read[1] 
#define HW_C     grey_Out_Read[2] 
#define HW_L2    grey_Out_Read[3] 
#define HW_L1    grey_Out_Read[4] 


#endif
