/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IMU_TX_Pin GPIO_PIN_2
#define IMU_TX_GPIO_Port GPIOA
#define IMU_RX_Pin GPIO_PIN_3
#define IMU_RX_GPIO_Port GPIOA
#define Motor1_Pin GPIO_PIN_0
#define Motor1_GPIO_Port GPIOB
#define Motor2_Pin GPIO_PIN_1
#define Motor2_GPIO_Port GPIOB
#define E1A_Pin GPIO_PIN_9
#define E1A_GPIO_Port GPIOE
#define E1B_Pin GPIO_PIN_11
#define E1B_GPIO_Port GPIOE
#define AIN2_Pin GPIO_PIN_12
#define AIN2_GPIO_Port GPIOE
#define BIN1_Pin GPIO_PIN_13
#define BIN1_GPIO_Port GPIOE
#define BIN2_Pin GPIO_PIN_14
#define BIN2_GPIO_Port GPIOE
#define AIN1_Pin GPIO_PIN_15
#define AIN1_GPIO_Port GPIOE
#define Daplink_TX_Pin GPIO_PIN_14
#define Daplink_TX_GPIO_Port GPIOB
#define Daplink_RX_Pin GPIO_PIN_15
#define Daplink_RX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define convert_param (30.0f*28.0f*4.0f) // 从脉冲转到转速的转换参数 转速 = 脉冲数 / (线数 * 减速比 * 4)
#define wheel_distance 0.23156f // 单位：m
#define wheel_radius 0.0325f // 单位：m
#define COUNTERNUM1 ((float)__HAL_TIM_GET_COUNTER(&htim1))
#define COUNTERNUM2 ((float)__HAL_TIM_GET_COUNTER(&htim2))
#define RELOADVALUE 20000
#define fliter_mean_sample1 6
#define fliter_buffer_size 100


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
