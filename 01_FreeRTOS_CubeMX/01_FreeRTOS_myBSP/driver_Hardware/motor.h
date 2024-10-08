#ifndef _MOTOR_H
#define _MOTOR_H

#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stm32f1xx_hal.h"
#include "main.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "list.h"
#include "stdbool.h"

#define AIN1(x)   HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,(GPIO_PinState)x)
#define AIN2(x)   HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,(GPIO_PinState)x)
#define BIN1(x)   HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,(GPIO_PinState)x)
#define BIN2(x)   HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,(GPIO_PinState)x)


void vMotorSetPWM(int16_t motor1Speed,int16_t motor2Speed);

#endif
