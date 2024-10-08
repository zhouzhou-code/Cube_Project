#include "serial.h"		 

//发送一个字节
void Serial_SendByte(USART_TypeDef* USARTx,int Byte) 
{
	while ((USARTx->SR & (1<<7)) == 0);  //等待状态寄存器为空
	USARTx->DR = Byte;                    //向DR寄存器中写入Byte
}


//发送一个字符串
void Serial_SendString(USART_TypeDef* USARTx,char* str) 
{
   int16_t i=0;
	while(str[i]!='\0')
	{
		Serial_SendByte(USARTx,str[i]);
        i++;   
	}

}

//自己封装的printf函数
void Serial_printf(USART_TypeDef* USARTx,char* format,...)
{ 
   char str[100];//定义字符数组
   va_list arg;  //定义可变参数列表类型的变量arg
   va_start(arg,format); //从format开始接收参数列表到arg变量
   vsprintf(str,format,arg); //使用vsprintf打印格式化字符串和参数列表到字符数组中
   va_end(arg);
   Serial_SendString(USARTx,str);
}

int fputc( int ch, FILE *f )// //函数默认的，在使用printf函数时自动调用,重定向只能使用某一个串口，这里使用了串口1
{
	USART_TypeDef* USARTx = USART1;
	while ((USARTx->SR & (1<<7)) == 0);
	USARTx->DR = ch;
	return ch;
}


/*******************************************************************************
* 函 数 名         : USART1_IRQHandler
* 函数功能		   : USART1中断函数
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/ 
