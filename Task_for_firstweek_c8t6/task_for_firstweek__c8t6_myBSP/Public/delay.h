#ifndef DELAY_H_
#define DELAY_H_

#include "stm32f1xx_hal.h"

void SysTick_Init(uint8_t SYSCLK); //SysTick定时器初始化函数
void Delay_us(uint32_t nus);       //微秒级延时函数
void Delay_ms(uint16_t nms);       //毫秒级延时函数
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);

//#define delay_ms    Delay_ms
//#define delay_us    Delay_us


#endif
