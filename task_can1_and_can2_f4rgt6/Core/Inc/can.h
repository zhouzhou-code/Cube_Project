/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */
typedef struct {
    CAN_RxHeaderTypeDef RxHeader;    // 接收头（包含ID，数据长度等信息）
	  uint8_t RxDataBuf[8];            //数据缓冲区，设置其为最大长度8，实际接收的数据长度由DLC确定
	  uint8_t RxDataLength;           // 接收数据长度(可以去掉，从头中读取数据,为了更直观的显示接收到的字节数，选择保留这个成员)

    CAN_TxHeaderTypeDef TxHeader;      // 发送头（包含ID，数据长度等信息）
    uint8_t TxDataBuf[8];              // 数据缓冲区，设置其为最大长度8，实际发送的数据长度由DLC确定
    
} CAN_Message_t;                     //使用结构体管理头和数据

/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */


void send_can_message(CAN_HandleTypeDef* hcan,CAN_Message_t* CAN_Message_Tx);
uint8_t receive_can_message(CAN_HandleTypeDef* hcan,CAN_Message_t* CAN_Message_Rx);


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
