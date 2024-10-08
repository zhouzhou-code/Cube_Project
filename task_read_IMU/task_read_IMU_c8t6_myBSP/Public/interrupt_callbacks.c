#include "interrupt_callbacks.h"
#include "usart.h"
#include "pid.h"
#include "string.h"
#include "stdio.h"
extern TimerHandle_t xKeyFilteringTimer;
KeyStatus keyStatus = NO_KEY;

//外部中断回调函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if(GPIO_Pin == K3_Pin) {
        keyStatus = KEY_K3;
        xTimerResetFromISR(xKeyFilteringTimer, NULL);
    }
    
    if(GPIO_Pin == K4_Pin) {
        keyStatus = KEY_K4;
        xTimerResetFromISR(xKeyFilteringTimer, NULL);
    }
}


/******************************************************************************
 * 函  数： HAL_UARTEx_RxEventCallback
 * 功  能： DMA+空闲中断回调函数              接收到一帧完整的数据后调用此函数处理数据
 * 参  数： UART_HandleTypeDef  *huart      // 触发的串口
 *          uint16_t Size                // 接收字节
 * 返回值： 无
 * 备  注： 
   1：这个是回调函数，不是中断服务函数。技巧：使用CubeMX生成的工程中，中断服务函数已被CubeMX安排妥当，我们只管重写回调函数
   2：触发条件：当DMA接收到指定字节数时，或产生空闲中断时，硬件就会自动调用本回调函数，无需进行人工调用;
   3：必须使用这个函数名称，因为它在CubeMX生成时，已被写好了各种函数调用、函数弱定义(在stm32xx_hal_uart.c的底部); 
      不要在原弱定义中增添代码，而是重写本函数                  
   4：无需进行中断标志的清理，它在被调用前，已有清中断的操作   
   5：生成的所有DMA+空闲中断服务函数，都会统一调用这个函数，以引脚编号作参数
   6：判断参数传进来的引脚编号，即可知道是哪个串口接收收了多少字节
******************************************************************************/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart == &huart1)																	 // 判断串口
	{ 
		printf("进入回调函数次数\n");
		
		__HAL_UNLOCK(huart);																 // 解锁串口状态
               
		xUSART1.ReceiveNum	= Size; 														 // 把接收字节数，存入结构体xUSART1.ReceiveNum，以备使用
		memset(xUSART1.ReceiveData, 0, sizeof(xUSART1.ReceiveData));						 // 清0前一帧的接收数据
		memcpy(xUSART1.ReceiveData, xUSART1.BuffTemp, Size);								 // 把新数据，从临时缓存中，复制到xUSART1.ReceiveData[], 以备使用
		
		printf("接收到了:%s\n,float:%.2f",xUSART1.ReceiveData,Get_Data_fromVOFA(xUSART1.ReceiveData));
		
    
		
#if 0
		
		//接收到的参数用来调参
		Update_PID_Params(&pidTracking,xUSART1.ReceiveData);
		//查看曲线
		printf("p=%.2f\n i=%.2f\n d=%.2f\n output=%.2f\n",pidTracking.Kp,pidTracking.Ki,pidTracking.Kd,pidTracking.output_val);
		OLED_Printf(4,1,"p=%.1f d=%.2f",pidTracking.Kp,pidTracking.Kd);
#else
		
		//接收到的参数用来调参
		Update_PID_Params(&pidMPU6050,xUSART1.ReceiveData);
		//查看曲线
		printf("p=%.2f\n i=%.2f\n d=%.2f\n output=%.2f\n",pidMPU6050.Kp,pidMPU6050.Ki,pidMPU6050.Kd,pidMPU6050.output_val);
		OLED_Printf(4,1,"p=%.1f d=%.2f",pidMPU6050.Kp,pidMPU6050.Kd);
		
		
#endif
        xUSART1.ReceiveNum = 0; 
    
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, xUSART1.BuffTemp,  sizeof(xUSART1.BuffTemp));	 // 再次开启DMA空闲中断; 每当接收完指定长度，或者产生空闲中断时，就会来到这个
		  
	}
}

/******************************************************************************
 * 函  数： HAL_UART_ErrorCallback
 * 功  能： 串口接收错误回调函数
 * 参  数： UART_HandleTypeDef  *huart   // 触发的串口
 * 返回值： 无
 * 备  注： 1：这个是回调函数，不是中断服务函数。技巧：使用CubeMX生成的工程中，中断服务函数已被CubeMX安排妥当，我们只管重写回调函数
*          2：触发条件：当串口收发出现错误，例如波特率等不匹配时
 *          3：必须使用这个函数名称，因为它在CubeMX生成时，已被写好了各种函数调用、函数弱定义(在stm32xx_hal_uart.c的底部); 不要在原弱定义中增添代码，而是重写本函数
 *          4：无需进行中断标志的清理，它在被调用前，已有清中断的操作;
            5:当某一次发生错误时，无法进入接收完成或DMA+空闲中断回调函数中，也就无法再次开启接收，即使下一次发送的波特率等参数
             都设置正确，也无法接收，所以发生错误后，我们需要在串口接收错误回调函数中重新开启接收
******************************************************************************/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) 
{
 if (huart == &huart1)																	 // 判断串口
 {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, xUSART1.BuffTemp,  sizeof(xUSART1.BuffTemp));
 
 }


}
