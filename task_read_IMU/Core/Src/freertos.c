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
#include "OLED.h"
#include "delay.h"
#include "mpu6050.h"
#include "inv_mpu.h"
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
/* Definitions for myTask_read_IMU */
osThreadId_t myTask_read_IMUHandle;
const osThreadAttr_t myTask_read_IMU_attributes = {
  .name = "myTask_read_IMU",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartmyTask_read_IMU(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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
  /* creation of myTask_read_IMU */
  myTask_read_IMUHandle = osThreadNew(StartmyTask_read_IMU, NULL, &myTask_read_IMU_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartmyTask_read_IMU */
/**
  * @brief  Function implementing the myTask_read_IMU thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartmyTask_read_IMU */
void StartmyTask_read_IMU(void *argument)
{
  /* USER CODE BEGIN StartmyTask_read_IMU */
	//初始化MPU6050和dmp库
	float pitch,roll,yaw; 		//欧拉角
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
 
  /* Infinite loop */
  for(;;)
  {
		
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);//获取三轴角速度
    MPU_Get_Accelerometer(&aacx,&aacy,&aacz);//获取三轴加速度
		mpu_dmp_get_data(&pitch,&roll,&yaw);     //获取欧拉角
		
		OLED_Printf(1,1,"ax:%d",aacx);
		OLED_Printf(2,1,"ay:%d",aacy);
		OLED_Printf(3,1,"az:%d",aacz);
		OLED_Printf(1,9,"gx:%d",gyrox);
		OLED_Printf(2,10,"gy:%d",gyroy);
		OLED_Printf(3,10,"gz:%d",gyroz);
		
    vTaskDelay(10);
  }
  /* USER CODE END StartmyTask_read_IMU */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
