#ifndef _SERIAL_H
#define _SERIAL_H

#include "stm32f1xx_hal.h"
#include "main.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "list.h"

/*-----------------------------------------------------------------------*/
/*C库的函数*/
#include "stdio.h" 
#include "stdarg.h" 
/*---------------------------------------------------------------------------*/

void Serial_SendByte(USART_TypeDef* USARTx,int Byte);//发送一个字节
void Serial_SendString(USART_TypeDef* USARTx,char* str); //发送字符串
void Serial_printf(USART_TypeDef* USARTx,char* format,...);//自己封装的Printf函数


#endif

