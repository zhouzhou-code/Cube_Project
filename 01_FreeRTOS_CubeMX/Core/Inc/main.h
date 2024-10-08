/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "stm32_hal_legacy.h"
#include "string.h"
#include "stdio.h"
#include "stdarg.h"


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#ifndef USE_HAL_LEGACY
#define  USE_HAL_LEGACY
#endif

#define RXBUFFER_SIZE   1

typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;
 
typedef const int32_t sc32;
typedef const int16_t sc16;
typedef const int8_t sc8;
 
typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;
 
typedef __I int32_t vsc32;
typedef __I int16_t vsc16;
typedef __I int8_t vsc8;
 
typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;
 
typedef const uint32_t uc32;
typedef const uint16_t uc16;
typedef const uint8_t uc8;
 
typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;
 
typedef __I uint32_t vuc32;
typedef __I uint16_t vuc16;
typedef __I uint8_t vuc8;

typedef struct                         // 声明一个结构体，方便管理变量
{
    uint16_t  ReceiveNum;              // 接收字节数
    uint8_t   ReceiveData[512];        // 接收到的数据
    uint8_t   BuffTemp[512];           // 接收缓存; 注意：这个数组，只是一个缓存，用于DMA逐个字节接收，
                                       // 当接收完一帧后，数据在回调函数中，转存到 ReceiveData[ ] 存放。
                                       // 即：双缓冲，有效减少单缓冲的接收过程新数据覆盖旧数据
} xUSATR_TypeDef;
 
extern  xUSATR_TypeDef xUSART1;      // 定义结构体，方便管理变量。也可以不用结构体，用单独的变量


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

extern u8 error;                    //DMP库初始化错误号
extern float pitch,roll,yaw; 		//DMP库输出的欧拉角
extern float BaseSpeed;
extern uint8_t mode;                //小车模式 0表示停止，1走圈，2走八字
extern uint8_t SwitchCounts;        //记录小车状态切换的次数



/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
double calculate_shortest_path(double current_angle, double target_angle);
void safe_printf(const char *format, ...) ;
void safe_printf_FromISR(const char *format, ...) ;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define grey3_Pin GPIO_PIN_4
#define grey3_GPIO_Port GPIOA
#define grey4_Pin GPIO_PIN_5
#define grey4_GPIO_Port GPIOA
#define grey5_Pin GPIO_PIN_6
#define grey5_GPIO_Port GPIOA
#define grey1_Pin GPIO_PIN_0
#define grey1_GPIO_Port GPIOB
#define grey2_Pin GPIO_PIN_1
#define grey2_GPIO_Port GPIOB
#define mpuiic_SCL_Pin GPIO_PIN_10
#define mpuiic_SCL_GPIO_Port GPIOB
#define mpuiic_SDA_Pin GPIO_PIN_11
#define mpuiic_SDA_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_12
#define AIN1_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_13
#define AIN2_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_14
#define BIN1_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_15
#define BIN2_GPIO_Port GPIOB
#define K3_Pin GPIO_PIN_8
#define K3_GPIO_Port GPIOC
#define K3_EXTI_IRQn EXTI9_5_IRQn
#define K4_Pin GPIO_PIN_9
#define K4_GPIO_Port GPIOC
#define K4_EXTI_IRQn EXTI9_5_IRQn
#define LED2_Pin GPIO_PIN_8
#define LED2_GPIO_Port GPIOA
#define SCL_Pin GPIO_PIN_8
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_9
#define SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

 
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
