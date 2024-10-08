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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*--------------------------------------------readme---------------------------------------------------------------------------------------------*/
/*              
                     作者:华南理工大学 周名顺
                     时间:2024.9.15
                     QQ:2031145985

本工程详细介绍了使用DMA+串口空闲中断接收不定长数据的方法，使用一个结构体+一个HAL库函数+两个回调函数，
完美实现了各种情况下的串口接收
无论是蓝牙，无线串口还是，wifi等，只要用到了串口，都可以使用本工程的方法。

                                                            其中有两个易错点: 
1，关于初始化顺序的问题： HAL库DMA初始化必须放在UART的初始化之前，因为UART初始化中也有关于DMA通道的配置，
 但是DMA的时钟开启却是在DMA初始化函数中完成的。如果DMA初始化在后，会导致UART中关于DMA的初始化失效

2，关于处理错误的回调函数：当串口接收发生错误，比如串口助手的波特率，数据位，停止位的参数设置和串口不同时，会触发错误
无法进入DMA+空闲中断回调函数开启下一次接收，所以即使下一次串口助手的参数设置正确，也会导致单片机无法接收。
所以解决办法是在串口错误回调函数中开启DMA接收*/
/*-------------------------------------------------------------------------------------------------------------------------------------------------*/
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int fputc( int ch, FILE *f );
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

 xUSATR_TypeDef xUSART1 = {0};          // 定义结构体，方便管理变量。也可以不用结构体，用单独的变量

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
    /* 用户代码，必须写在配对的BEGIN与END之间 */
    
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, xUSART1.BuffTemp, sizeof(xUSART1.BuffTemp));  // 开启DMA空闲中断    
    
		static char  strTem[100] = "Hello World!\r";                            // 定义一个数组，也可以是其它的数据，如结构体等
    HAL_UART_Transmit (&huart1, (uint8_t*)strTem, strlen(strTem), 0xFFFF);  // 发送方式2：HAL_UART_Transmit(), 不推荐使用; 阻塞式发送，当调用后，程序会一直死等，不干其它事了(中断除外)，直到发送完毕
    HAL_UART_Transmit_IT (&huart1, (uint8_t*)strTem, strlen(strTem));       // 发送方式3：HAL_UART_Transmit_IT(), 推荐使用; 利用中断发送，非阻塞式，大大减少资源占用; 注意：当上次的调用还没完成发送，下次的调用会直接返回(放弃)，所以，要想连接发送，两行调用间，要么判断串口结构体gState的值，要么调用延时HAL_Delay(ms), ms值要大于前一帧发送用时, 用时计算：1/(波特率*11*前一帧字节数) 
    while((&huart1)->gState != HAL_UART_STATE_READY);                       // 等待上条发送结束; 也可以用HAL_Delay延时法，但就要计算发送用时; 两种方法都是死等法，程序暂时卡死不会往下运行; 如果两次发送间隔时间大，如，大于100ms, 就不用判断语句了。
    HAL_UART_Transmit_DMA (&huart1,(uint8_t*)strTem, strlen(strTem));       // 发送方式4：HAL_UART_Transmit_DMA()，推荐使用; 利用DMA发送，非阻塞式，最大限度减少资源占用; 注意：当上次的调用还没完成发送，下次的调用会直接返回(放弃); 所以，要想连接发送，两行调用间，要么判断串口结构体gState的值，要么调用延时HAL_Delay(ms), ms值要大于前一帧发送用时，用时计算：1/(波特率*11*前一帧字节数)
 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (xUSART1.ReceiveNum)                                                  // 判断字节数
		{
				printf("\r\n<<<<< USART1 接收到一帧数据 \r\n");                  // 提示
				printf("字节数：%d \r", xUSART1.ReceiveNum);             // 显示字节数
				printf("ASCII : %s\r", (char *)xUSART1.ReceiveData);    // 显示数据，以ASCII方式显示，即以字符串的方式显示
				printf("16进制: ");                                                              // 显示数据，以16进制方式，显示每一个字节的值
				for (uint16_t i = 0; i < xUSART1.ReceiveNum; i++)          // 逐个字节输出
						printf("0x%X ", xUSART1.ReceiveData[i]);                   // 以16进制显示
				printf("\r\r");                                                                       // 显示换行

				xUSART1.ReceiveNum = 0;                                             // 清0接收标记
		}

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
int fputc( int ch, FILE *f )
{
	USART_TypeDef* USARTx = USART1;
	while ((USARTx->SR & (1<<7)) == 0);
	USARTx->DR = ch;
	return ch;
}

 
/******************************************************************************
 * 函  数： HAL_UARTEx_RxEventCallback
 * 功  能： DMA+空闲中断回调函数
 * 参  数： UART_HandleTypeDef  *huart   // 触发的串口
 *          uint16_t             Size    // 接收字节
 * 返回值： 无
 * 备  注： 1：这个是回调函数，不是中断服务函数。技巧：使用CubeMX生成的工程中，中断服务函数已被CubeMX安排妥当，我们只管重写回调函数
 *          2：触发条件：当DMA接收到指定字节数时，或产生空闲中断时，硬件就会自动调用本回调函数，无需进行人工调用;
 *          2：必须使用这个函数名称，因为它在CubeMX生成时，已被写好了各种函数调用、函数弱定义(在stm32xx_hal_uart.c的底部); 不要在原弱定义中增添代码，而是重写本函数
 *          3：无需进行中断标志的清理，它在被调用前，已有清中断的操作;
 *          4：生成的所有DMA+空闲中断服务函数，都会统一调用这个函数，以引脚编号作参数
 *          5：判断参数传进来的引脚编号，即可知道是哪个串口接收收了多少字节
******************************************************************************/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart1)                                                                    // 判断串口
    {
        __HAL_UNLOCK(huart);                                                                 // 解锁串口状态
 
        xUSART1.ReceiveNum  = Size;                                                          // 把接收字节数，存入结构体xUSART1.ReceiveNum，以备使用
        memset(xUSART1.ReceiveData, 0, sizeof(xUSART1.ReceiveData));                         // 清0前一帧的接收数据
        memcpy(xUSART1.ReceiveData, xUSART1.BuffTemp, Size);                                 // 把新数据，从临时缓存中，复制到xUSART1.ReceiveData[], 以备使用
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, xUSART1.BuffTemp, sizeof(xUSART1.BuffTemp));   // 再次开启DMA空闲中断; 每当接收完指定长度，或者产生空闲中断时，就会来到这个
    }
}


/******************************************************************************
 * 函  数： HAL_UART_ErrorCallback
 * 功  能： 串口接收错误回调函数
 * 参  数： UART_HandleTypeDef  *huart   // 触发的串口
 * 返回值： 无
 * 备  注： 1：这个是回调函数，不是中断服务函数。技巧：使用CubeMX生成的工程中，中断服务函数已被CubeMX安排妥当，我们只管重写回调函数
*          2：触发条件：当串口收发出现错误，例如波特率等不匹配时
 *          3：必须使用这个函数名称，因为它在CubeMX生成时，已被写好了各种函数调用、函数弱定义(在stm32xx_hal_uart.c的底部); 不要在原弱定义中增添代码，而是重写本函数
 *          4：无需进行中断标志的清理，它在被调用前，已有清中断的操作;
            5:作用：当某一次发生错误时，无法进入接收完成或DMA+空闲中断回调函数中，也就无法再次开启接收，即使下一次发送的波特率等参数
             都设置正确，也无法接收，所以发生错误后，我们需要在串口接收错误回调函数中重新开启接收
******************************************************************************/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) 
{
 if (huart == &huart1)																	 // 判断串口
 {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, xUSART1.BuffTemp,  sizeof(xUSART1.BuffTemp));
 
 }


}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
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
