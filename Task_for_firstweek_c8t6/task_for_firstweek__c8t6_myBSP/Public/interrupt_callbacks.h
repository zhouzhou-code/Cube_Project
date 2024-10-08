#ifndef INTERRUPT_CALLBACKS_
#define INTERRUPT_CALLBACKS_

#include "stm32f1xx_hal.h"
#include "main.h"
#include "delay.h"
#include "stdbool.h"
#include "usart.h"
#include "string.h"



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);



#endif
