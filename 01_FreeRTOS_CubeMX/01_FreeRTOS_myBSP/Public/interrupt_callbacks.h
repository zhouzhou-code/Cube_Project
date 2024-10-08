#ifndef INTERRUPT_CALLBACKS_
#define INTERRUPT_CALLBACKS_

#include "stm32f1xx_hal.h"
#include "main.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "list.h"
#include "stdbool.h"
#include "usart.h"
#include "oled.h"
#include "string.h"
#include "pid.h"


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);





typedef enum {
    NO_KEY,
    KEY_K3,
    KEY_K4
} KeyStatus;//记录按键的状态

extern KeyStatus keyStatus;
extern uint8_t ifKeyTouched;



#endif

