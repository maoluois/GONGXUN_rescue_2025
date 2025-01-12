/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart5;

extern UART_HandleTypeDef huart8;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_UART5_Init(void);
void MX_UART8_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
typedef struct
{
  uint16_t ReceiveNum;  // 接收字节数，在中断回调中自动赋值，只要字节数>0即为接受到新的一帧数据
  uint8_t ReceiveData[XBOX_BUFFER_SIZE];  // 接收到的数据
  uint8_t BuffTemp[XBOX_BUFFER_SIZE];  // 临时缓存
} xUART_TypeDef;

typedef struct
{
  uint16_t ReceiveNum;  // 接收字节数，在中断回调中自动赋值，只要字节数>0即为接受到新的一帧数据
  uint8_t ReceiveData[Oran_BUFFER_SIZE];  // 接收到的数据
  uint8_t BuffTemp[Oran_BUFFER_SIZE];  // 临时缓存
} OranUART_TypeDef;
  // 声明外部变量
extern xUART_TypeDef xUSART1;
extern xUART_TypeDef xUSART2;
extern xUART_TypeDef xUSART3;
extern xUART_TypeDef xUART4;
extern OranUART_TypeDef xUART5;
extern xUART_TypeDef xUSART6;
extern xUART_TypeDef xUART7;
extern xUART_TypeDef xUART8;

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

