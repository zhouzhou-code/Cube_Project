/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "string.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "list.h"
#include "serial.h"
#include "interrupt_callbacks.h"
#include "OLED.h"
#include "delay.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32_hal_legacy.h"//�ɿ�
#include "MPU9250.h"                          //MPU9250(6050)
#include "inv_mpu.h"                          //
#include "inv_mpu_dmp_motion_driver.h"        //DMP输出库
#include "pid.h"
#include "greys.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
xUSATR_TypeDef xUSART1={0} ;
u8 error;      //DMP库初始化错误号
float pitch,roll,yaw; 		//DMP库输出的欧拉角
float BaseSpeed=20;
uint8_t mode=0;                //小车模式 0表示停止，1走圈，2走八字
uint8_t SwitchCounts;      //记录小车状态切换的次数



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
TaskHandle_t xTaskTrackHandle;
TaskHandle_t xTaskGoblankHandle;
TaskHandle_t xTaskCheckHandle;
TimerHandle_t xKeyFilteringTimer;
SemaphoreHandle_t xPrintfMutex;      //UART互斥量句柄


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void TaskTrackFun(void*param);//巡线任务
static void TaskGoblankFun(void*param);//导航走直线任务
static void xTaskCheckFun(void*param);//检测小车状态切换的任务
void safe_printf(const char *format, ...) ;
void safe_printf_FromISR(const char *format, ...) ;
void vKeyFilteringTimerFunc( TimerHandle_t xTimer );
void my_prvHardwareSetup(void);//自己的硬件初始化函数
void SetAngle(float current_angle, float target_angle);//设置出发角度

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
 
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM7_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /* 注意DMA初始化要在UART之前*/	
	/*-------初始化自己的硬件代码(放在CubeMX的初始化之后)-----------------*/
	my_prvHardwareSetup();
 

	/*---------------------FreeRTOS的相关代码--------------------------------*/
	xKeyFilteringTimer=xTimerCreate("TimerKey",20,pdFALSE,0,vKeyFilteringTimerFunc);
	xPrintfMutex=xSemaphoreCreateMutex();
    xTaskCreate(TaskTrackFun,"Tracking",128,NULL,1,&xTaskTrackHandle);
    xTaskCreate(TaskGoblankFun,"Go Straight",128,NULL,1,&xTaskGoblankHandle);
	xTaskCreate(xTaskCheckFun,"Check the State",128,NULL,2,&xTaskCheckHandle);	
    vTaskStartScheduler();//调度器不打开则无法运行任务和开启中断
	/*---------------------end--------------------------------*/
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {    
		  
//		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
//      Delay_ms(100);
//			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
//      Delay_ms(600);
//		  	mpu_dmp_get_data(&pitch,&roll,&yaw);
//	      OLED_Printf(2,1,"yaw=%.3f",yaw);
//		HAL_UART_Transmit(&huart1, (uint8_t *)RxBuffer, 1,10);
//		delay_ms(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void TaskTrackFun(void*param)
{
	
  int CNT=0,differential=0;
   while(1)
  {   
	
   if (mode == 0)
   {
      vMotorSetPWM(0, 0);//停车
      safe_printf("TaskTrack is Suspended\r\n");
	  OLED_Printf(3,11,"mode%d",mode);
	  vTaskSuspend(NULL);//模式0，任务挂起 
   }  
   else
   {
	   taskENTER_CRITICAL();//进入临界区
			
	   differential= Priority_Track(0);//调用巡线任务，获取差速值
	   vMotorSetPWM(BaseSpeed+differential,BaseSpeed-differential);//ͨ通过差速值控制PWM
		 
	   taskEXIT_CRITICAL();//退出临界区
	   
		// safe_printf("vleft=%.2f\n vright=%.2f\n",BaseSpeed+differential,BaseSpeed-differential);
		 
		 OLED_Printf(1,1,"vl=%.1f  vr=%.1f",BaseSpeed+differential,BaseSpeed-differential);

		 //safe_printf("TaskTrack is running:%d \r\n",CNT++);
        vTaskDelay(pdMS_TO_TICKS(10));
   }
   

  }

}
static void TaskGoblankFun(void*param)
{ 
	
    int CNT=0;
    while(1)
    {  
    
       if (mode == 0)
	   {
	      
	      vMotorSetPWM(0, 0);//停车
	     safe_printf("TaskGoblank is Suspended\r\n");
		  vTaskSuspend(NULL);//模式0，任务挂起 
		  OLED_Printf(3,11,"mode%d",mode);
	      
	   }          
       else if(mode == 1) 
       {
         taskENTER_CRITICAL();//进入临界区
         
         mpu_dmp_get_data(&pitch,&roll,&yaw);//获取陀螺仪欧拉角
    	 if(SwitchCounts%4==0)//SwitchCounts==0,4,8,12,16......
    	    SetAngle(yaw,0);
    	 else if(SwitchCounts%2==0)//SwitchCounts==2,6,10,14,18......
    	    SetAngle(yaw,180);
         		 
         taskEXIT_CRITICAL();//退出临界区
			 
		//safe_printf("taskGoblank is running mode%d\r\n ", mode);
		OLED_Printf(2,1,"yaw=%.2f",yaw);
        
    	 vTaskDelay(pdMS_TO_TICKS(10));
    	 
       }
       else if(mode==2)
       {
         taskENTER_CRITICAL();//进入临界区:获取当前角度和用当前角度控制电机必须同步
         
         mpu_dmp_get_data(&pitch,&roll,&yaw);//获取陀螺仪欧拉角
         if(SwitchCounts%4==0)//SwitchCounts==0,4,8,12,16......
    		 SetAngle(yaw,37);
    	 else if(SwitchCounts%2==0)//SwitchCounts==2,6,10,14,18......
    	    SetAngle(yaw,-143); 
		 
         taskEXIT_CRITICAL();//退出临界区
			 
   		 //safe_printf("taskGoblank is running mode%d\r\n ", mode);  
         OLED_Printf(2,1,"yaw=%.1f",yaw);
    	 vTaskDelay(pdMS_TO_TICKS(10));
         
       }
      else if(mode == 4) //陀螺仪调试模式
      {
         taskENTER_CRITICAL();//进入临界区:获取当前角度和用当前角度控制电机必须同步
         
         mpu_dmp_get_data(&pitch,&roll,&yaw);//获取陀螺仪欧拉角
         if(SwitchCounts%4==0)//SwitchCounts==0,4,8,12,16......
    		 SetAngle(yaw,0);
    	 else if(SwitchCounts%2==0)//SwitchCounts==2,6,10,14,18......
    	    SetAngle(yaw,180);
		 
    	 taskEXIT_CRITICAL();//退出临界区
			  //safe_printf("taskGoblank is running mode%d\r\n ", mode); 
    	 OLED_Printf(2,1,"yaw=%.2f",yaw);
	
		 
    	 vTaskDelay(pdMS_TO_TICKS(10));
    	 
      }
		
		 }
		 
}
static void xTaskCheckFun(void*param)
{
    int CNT=0;
  
    while(1)
	{   
     if (mode == 0)
     {
          
         vMotorSetPWM(0, 0);//停车
         safe_printf("TaskCheck is Suspended\r\n");
		 vTaskSuspend(NULL);//模式0，任务挂起
		 OLED_Printf(3,11,"mode%d",mode);
     }  
     else
     {
	     taskENTER_CRITICAL();//进入临界区
	     //printf("%.2f,%.2f,%.2f\n",sin(a++),b++,c++);//test vofa+
	     
	     grey_state(&SwitchCounts);//判断小车当前状态
	     if(SwitchCounts%2==0) //SwitchCounts==0,2,4,6.....走直线
	     {
	       vTaskSuspend(xTaskTrackHandle);//挂起巡线任务
	       vTaskResume(xTaskGoblankHandle);//唤醒陀螺仪走直线任务
	       
	     }
	     else //SwitchCounts==1,3,5,7.....巡线
	     {
	       vTaskSuspend(xTaskGoblankHandle);//挂起陀螺仪走直线任务
	       vTaskResume(xTaskTrackHandle);//唤醒巡线任务
	     }
		 
	     taskEXIT_CRITICAL();//退出临界区
			 
	    //safe_printf("TaskCheck is running:%d\r\n",CNT++);
	    OLED_Printf(3,1,"Switch=%d",SwitchCounts);
		
	     vTaskDelay(pdMS_TO_TICKS(20));//每20ms运行一次判断任务

     }
     
		 
    }
  
}

//按键消抖后的处理函数
void vKeyFilteringTimerFunc(TimerHandle_t xTimer) 
{   
	 #if 1
    static uint8_t pressCount = 0;    
    
    switch(keyStatus) 
    {
        case KEY_K3:
        {
	         if (HAL_GPIO_ReadPin(K3_GPIO_Port,K3_Pin)==GPIO_PIN_SET)//读取按键电平
	         {  
	             pressCount=(pressCount+1)%5;//pressCount++,mode++
	             safe_printf("mode%d\r\n",pressCount);            
	             OLED_Printf(3,11 ,"mode%d",pressCount);
            
             } 
	         
            break;
        }
           
        case KEY_K4:   
        {
            if (HAL_GPIO_ReadPin(K4_GPIO_Port,K4_Pin)==GPIO_PIN_SET) 
            {  
               taskENTER_CRITICAL();//进入临界区
               
               mode=pressCount;
               if(mode==1|| mode==2)
               {
                 vTaskResume(xTaskGoblankHandle);
				 vTaskResume(xTaskCheckHandle);
				 vTaskResume(xTaskTrackHandle);
			   }
               else if(mode==3)//寻迹调试模式
               {
                 vTaskSuspend(xTaskGoblankHandle); 
                 vTaskSuspend(xTaskCheckHandle); 
                 vTaskResume(xTaskTrackHandle);                 
               }
               else if(mode==4)//陀螺仪调试模式
               {
                 SwitchCounts=0;
                 vTaskSuspend(xTaskTrackHandle); 
                 vTaskSuspend(xTaskCheckHandle); 
                 vTaskResume(xTaskGoblankHandle);                 
               }               
			   
               taskEXIT_CRITICAL();//退出临界区
							 
               safe_printf("mode%d is ready\r\n",mode);
             //  OLED_Printf(1,1 ,"mode%d is ready",mode);
            }
            break;
        }
        
        default:
            //其他情况，退出
            break;
    }
    
    keyStatus = NO_KEY;  //重置keyStatus
		#endif
		
}


/**
  * @brief  自己的硬件的初始化
  * @param  无
  * @retval 无
  */
void my_prvHardwareSetup(void)
{   
	#if 1
    HAL_TIM_Base_Start(&htim2);    //开启时基，cubeMX初始化并不会开启定时器，需要我们手动打开使能定时器
    HAL_TIM_Base_Start(&htim7); //开启时基，cubeMX初始化并不会开启，需要我们手动打开使能定时器
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);//开启定时器2的PWM通道1的输出
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);//开启定时器2的PWM通道2的输出
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, xUSART1.BuffTemp, sizeof(xUSART1.BuffTemp));    //开启DMA接收完成中断，串口空闲中断 
	
    PID_Init();                                  //PID初始化
	OLED_Init();                                 //OLED初始化
	MPU_Init();                                   
	
	if(MPU_Init() == 0)			                 //初始化MPU9250(6050)
		printf("ID读取正常\r\n");                    //ID读取正常
	else
		printf("ID读取不正常\r\n");                  //ID读取不正常
	while(1)                                       
	{
		
		error = mpu_dmp_init();                      //初始化mpu_dmp库
		printf("ERROR:%d\r\n",error);                //串口显示初始化错误值
		switch (error)                               //对不同错误值类型判断
		{
			case 0:printf("DMP库初始化正常\r\n");break;
			case 1:printf("设置传感器失败\r\n");break;
			case 2:printf("设置FIFO失败\r\n");break;
			case 3:printf("设置采样率失败\r\n");break;
			case 4:printf("加载dmp固件失败\r\n");break;
			case 5:printf("设置陀螺仪方向失败\r\n");break;
			case 6:printf("设置dmp功能失败\r\n");break;
			case 7:printf("设置DMP输出速率失败\r\n");break;
			case 8:printf("自检失败\r\n");break;
			case 9:printf("使能DMP失败\r\n");break;
			case 10:printf("初始化MPU6050失败\r\n");break;
			default :printf("未知错误\r\n");break;
		}
		
		if(error == 0)break;               //没有错误，退出循环                          
		delay_ms(200);                            
		OLED_Printf(1,1,"    ERROR:%d",error);
		
 } 
	delay_ms(999); //MPU6050DMP初始化必须的延时
  #endif
}


/**
  * @brief  计算两个角度(-180~+180)之间的最短路径
  * @param  当前角度
  * @param  目标角度
  * @retval 当前角度到目标角度之间的最短路径
  */
double calculate_shortest_path(double current_angle, double target_angle) 
{
    
    current_angle = fmod(current_angle + 180, 360) - 180;
    
    
    double angle_difference = target_angle - current_angle;

   
    if (fabs(angle_difference) < 180) {
        
        return angle_difference;
    } else {
        
        return (angle_difference > 0) ? -(360 - fabs(angle_difference)) : (360 - fabs(angle_difference));
    }
}

/**
  * @brief  设置出发时的目标角度
  * @param  current_angle 当前角度，由陀螺仪得出
  * @param  目标角度
  * @retval 无
  */
void SetAngle(float current_angle, float target_angle)
{ 
  float pid_output_max=30;//输出限幅
	
  float shortest_path=calculate_shortest_path(current_angle,target_angle);//计算最短路径
  int differential=PID_Position(&pidMPU6050, shortest_path);
	
  if(differential>pid_output_max)	differential=pid_output_max;
	if(differential<-pid_output_max)	differential=-pid_output_max;
	
  vMotorSetPWM(BaseSpeed+differential,BaseSpeed-differential);//设置PWM差速
  
}
// 封装的 printf 函数，线程安全
void safe_printf(const char *format, ...) 
{
    xSemaphoreTake(xPrintfMutex, portMAX_DELAY);//获得互斥量
		
    va_list args;
    va_start(args, format);
    vprintf(format, args);  // 通过 vprintf 打印
    va_end(args);
	
    xSemaphoreGive(xPrintfMutex);//释放互斥量
   
}
void safe_printf_FromISR(const char *format, ...) 
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
    xSemaphoreTakeFromISR(xPrintfMutex,&xHigherPriorityTaskWoken);
	
    va_list args;
    va_start(args, format);
    vprintf(format, args);  // 通过 vprintf 打印
    va_end(args);
	
    xSemaphoreGiveFromISR(xPrintfMutex,&xHigherPriorityTaskWoken);
		 
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);//任务切换

}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
