/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "queue.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for myTask_LED */
osThreadId_t myTask_LEDHandle;
const osThreadAttr_t myTask_LED_attributes = {
  .name = "myTask_LED",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for myTask_UART */
osThreadId_t myTask_UARTHandle;
const osThreadAttr_t myTask_UART_attributes = {
  .name = "myTask_UART",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};
/* Definitions for Mutex_UART_Lock */
osMutexId_t Mutex_UART_LockHandle;
const osMutexAttr_t Mutex_UART_Lock_attributes = {
  .name = "Mutex_UART_Lock"
};
/* Definitions for BinarySem_Rec */
osSemaphoreId_t BinarySem_RecHandle;
const osSemaphoreAttr_t BinarySem_Rec_attributes = {
  .name = "BinarySem_Rec"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void breathing_light(float frequency, uint8_t max_brightness);//呼吸灯
/* USER CODE END FunctionPrototypes */

void StartLED_Task(void *argument);
void StartTask_UART(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of Mutex_UART_Lock */
  Mutex_UART_LockHandle = osMutexNew(&Mutex_UART_Lock_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of BinarySem_Rec */
  BinarySem_RecHandle = osSemaphoreNew(1, 1, &BinarySem_Rec_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of myTask_LED */
  myTask_LEDHandle = osThreadNew(StartLED_Task, NULL, &myTask_LED_attributes);

  /* creation of myTask_UART */
  myTask_UARTHandle = osThreadNew(StartTask_UART, NULL, &myTask_UART_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartLED_Task */
/**
  * @brief  Function implementing the myTask_LED thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLED_Task */
void StartLED_Task(void *argument)
{
  /* USER CODE BEGIN StartLED_Task */
	float frequency=0.5;
  uint8_t  max_brightness=100;
	
  /* Infinite loop */
  for(;;)
  {
		
		  breathing_light(frequency,max_brightness);
    
  }
  /* USER CODE END StartLED_Task */
}

/* USER CODE BEGIN Header_StartTask_UART */
/**
* @brief Function implementing the myTask_UART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_UART */
void StartTask_UART(void *argument)
{
  /* USER CODE BEGIN StartTask_UART */
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake(BinarySem_RecHandle,portMAX_DELAY);//获取信号量
		
		xSemaphoreTake(Mutex_UART_LockHandle,portMAX_DELAY);//获取互斥锁	
		printf("收到: %s\n",xUSART1.ReceiveData);	
		xSemaphoreGive(Mutex_UART_LockHandle);//释放互斥锁
		
  }
  /* USER CODE END StartTask_UART */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/*
*@brief  LED呼吸灯，可通过入口参数调整最大亮度和频率
*@param1 frequency 呼吸灯频率 0.1~2之间比较合适
*@param2 max_brightness 呼吸灯最大亮度  取值0~100
*/
void breathing_light(float frequency, uint8_t max_brightness)
{
	  
	  float T =1000.0/frequency;//单位ms
	
    // 计算每个阶段的延迟（单位：毫秒）
    uint32_t delay_time_ms = T/(2*max_brightness);
    
    // 逐渐增加亮度
    for (uint8_t brightness = 0; brightness <= max_brightness; brightness++)
	  {
        SetPWMDuty(&htim2,TIM_CHANNEL_1,brightness);
			
        vTaskDelay(pdMS_TO_TICKS(delay_time_ms));
    }

    // 逐渐减少亮度
    for (uint8_t brightness = max_brightness; brightness > 0; brightness--)
		{
        SetPWMDuty(&htim2,TIM_CHANNEL_1,brightness);
			  
        vTaskDelay(pdMS_TO_TICKS(delay_time_ms));
    }
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
