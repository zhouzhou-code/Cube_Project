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


#include "OLED.h"
#include "delay.h"
#include "mpu6050.h"
#include "inv_mpu.h"

#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float freq;
    float max_lighting;
    //可扩展更多参数
} LED_Params_t;//LED调参结构体，为下面的parseParameters函数服务
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
xQueueHandle Queue_Trams_param_handle; //队列
/* USER CODE END Variables */
/* Definitions for myTask_read_IMU */
osThreadId_t myTask_read_IMUHandle;
const osThreadAttr_t myTask_read_IMU_attributes = {
  .name = "myTask_read_IMU",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask_LED */
osThreadId_t myTask_LEDHandle;
const osThreadAttr_t myTask_LED_attributes = {
  .name = "myTask_LED",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask_UART */
osThreadId_t myTask_UARTHandle;
const osThreadAttr_t myTask_UART_attributes = {
  .name = "myTask_UART",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
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
bool parseParameters(const char *input, LED_Params_t *params);//参数解析函数
/* USER CODE END FunctionPrototypes */

void StartmyTask_read_IMU(void *argument);
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
	
	Queue_Trams_param_handle=xQueueCreate(2,sizeof(LED_Params_t));//传递LED参数的队列LED
	
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of myTask_read_IMU */
  myTask_read_IMUHandle = osThreadNew(StartmyTask_read_IMU, NULL, &myTask_read_IMU_attributes);

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

/* USER CODE BEGIN Header_StartLED_Task */
/**
* @brief Function implementing the myTask_LED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLED_Task */
void StartLED_Task(void *argument)
{
  /* USER CODE BEGIN StartLED_Task */
	LED_Params_t LED_Params={0.5,100};
  /* Infinite loop */
  for(;;)
  {
    
		//如果传参队列里面有数据，获取;如果无数据，立即返回，不等待
	 xQueueReceive(Queue_Trams_param_handle,&LED_Params,0);
		
   breathing_light(LED_Params.freq,LED_Params.max_lighting);
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
		xSemaphoreTake(BinarySem_RecHandle,portMAX_DELAY);//获取信号量(串口有数据来了)
		
		xSemaphoreTake(Mutex_UART_LockHandle,portMAX_DELAY);//获取互斥锁		
		printf("收到: %s\n",xUSART1.ReceiveData);	
		xSemaphoreGive(Mutex_UART_LockHandle);//释放互斥锁
		
		LED_Params_t LED_Params={0.5,100};
		
		parseParameters((char*)xUSART1.ReceiveData,&LED_Params);//解析参数(xUSART1.ReceiveData已经加上了‘\0’，可以强转为(char*))
		
		xQueueSend(Queue_Trams_param_handle,&LED_Params,portMAX_DELAY);//通过队列将参数结构体发给LED任务
		
		
   
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


/*
*@brief  参数解析函数
*@param1 从串口助手收到的字节数组,格式为{freq=11;max=11;}，(若有其他参数需求，可以修改判断分支和参数结构体)
*@retval true:成功  false；失败
*@注意：本函数用到了很多局部变量，需要较多堆栈
*/
bool parseParameters(const char *input, LED_Params_t *params) 
{
    // 初始化参数为默认值
    *params = (LED_Params_t){0.5, 100};

    // 检查输入格式
    if (input[0] != '{' || input[strlen(input) - 1] != '}')
		{
      return false;  // 格式不正确
    }

    // 去掉大括号
    char buffer[100];  //这个地方可能需要更多的空间
    strncpy(buffer, input + 1, strlen(input) - 2);
    buffer[strlen(input) - 2] = '\0';  // 结束字符串

    // 分割参数
    char *token = strtok(buffer, ";");
    while (token != NULL)
		{
        char key[10];
        char value[10];

        // 分割键值对
        if(sscanf(token, "%[^=]=%s", key, value) == 2)// %[^=]读取=之前的所有东西，放在key里；%s读取表示读取接下来的所有非空白字符，直到遇到空格或字符串结束,存储到 value 变量中。
				{
            if (strcmp(key, "freq") == 0) //如果键等于freq
						{
              params->freq = atof(value);  // 转换为 float
            } 
						else if (strcmp(key, "max") == 0) //如果键等于max
						{
              params->max_lighting = atof(value);  // 转换为 float
            } 
        }
        token = strtok(NULL, ";");//继续分割字符串
    }

    return true;  // 解析成功
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
