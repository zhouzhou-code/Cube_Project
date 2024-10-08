#ifndef INTERRUPT_CALLBACKS_
#define INTERRUPT_CALLBACKS_

#include "stm32f1xx_hal.h"
#include "main.h"
#include "delay.h"
#include "stdbool.h"
#include "usart.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "cmsis_os.h"

extern osSemaphoreId_t BinarySem_RecHandle; //信号量句柄  定义在freertos.c 
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle);



#endif
