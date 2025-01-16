/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "memorymap.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "retarget.h"
#include "control.h"
#include "filter.h"
#include "Algorithm.h"
#include "pid.h"
#include <stdio.h>
#include <string.h>
#include "jy901s.h"
#include "Xbox.h"
#include "delay.h"


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

/* USER CODE BEGIN PV */
// controller PV
int mode = 0;                 // 控制模式
int task = 0;                 // 任务
int state = 0;                // 状态
int flag = 0;

// encoder PV
int32_t totalAngle1 = 0;      // 总的角度
int32_t totalAngle2 = 0;
int32_t lastAngle = 0;        // 上一次的角度
// int16_t loopNum1 = 0;      // 防超上限
// int16_t loopNum2 = 0;
float wheel1_speed = 0;       // 轮子速度 （单位：cm/s）
float wheel2_speed = 0;
float wheel1_position = 0;    // 轮子位置
float wheel2_position = 0;
float wheel1_total_position = 0;       // 轮子总位移
float wheel2_total_position = 0;       // 轮子总位移
float last_wheel1_speed = 0;  // 上一次的轮子速度
float last_wheel2_speed = 0;
float wheel1_speedF = 0;      // 滤波后的轮子速度
float wheel2_speedF = 0;
float SetSpeed1 = 0;          // 设置目标速度（单位：cm/s）
float SetSpeed2 = 0;
float SetSpeed6 = 0;

// car centre PV
float CurrentDistance = 0;
float linear_speed = 0;       // 线速度
float SpeedY = 0;         // 目标速度（单位：cm/s）
float angular_speed = 0;      // 角速度

// usart PV
xUART_TypeDef xUSART1 = {0};  // 串口1;
xUART_TypeDef xUSART2 = {0};  // 串口2;
xUART_TypeDef xUSART3 = {0};  // 串口3;
xUART_TypeDef xUART4 = {0};  // 串口4;
OranUART_TypeDef xUART5 = {0};  // 串口5;
xUART_TypeDef xUSART6 = {0};  // 串口6;
xUART_TypeDef xUART7 = {0};  // 串口7;
xUART_TypeDef xUART8 = {0};  // 串口8;
    // Imu JY901s PV
    JY_USART JY901s = {0};   // 本例中具有JY901s使用串口2
    // ########################################################################
    #define Receiveing 1
    #define Free 0
    uint8_t recieve_flag = 0 ;

    uint8_t buf ;
    uint8_t str_data[100];
    uint8_t str_index = 0;

    float accelerate_x ;
    float accelerate_y ;
    float accelerate_z ;

    float angel_velocity_x ;
    float angel_velocity_y ;
    float angel_velocity_z ;

    float angle_x ;
    float angle_y ;
    float angle_z ;

    uint8_t data_ok = 0;

    uint8_t Test_data(uint8_t* str , uint8_t lenth)
    {
        uint8_t result = 0 ;
        for( uint8_t i = 0 ; i < lenth ; i++ )
        {
            result = result + str[i];

        }
        return result;
    }
    // ###############################################################################
    float yaw = 0;
    float yawF = 0;

    // Xbox PV
    uint16_t XboxData[4]  = {9, 9, 0 ,0};

    // camera PV
    volatile class_Ogpi red_ball = {0};
    volatile class_Ogpi blue_ball = {0};
    volatile class_Ogpi yellow_ball = {0};
    volatile class_Ogpi black_ball = {0};
    volatile class_Ogpi blue_aim = {0};
    volatile class_Ogpi red_aim = {0};
    volatile class_Ogpi blue_base = {0};
    volatile class_Ogpi red_base = {0};
    volatile uint8_t class = 0;
    float biasX = 0;
    float biasY = 0;

// fliter PV
float mean_buff1[fliter_buffer_size];             // 滤波缓冲
float mean_buff2[fliter_buffer_size];
float mean_buff3[fliter_buffer_size];
int buff_index1 = 0;                // 滤波缓冲区索引
int buff_index2 = 0;

// DMA PV
// extern uint8_t Rx_data8[BUFFER_SIZE];    // 接收数组
// extern uint8_t Rx_len8;    // 接收长度
// extern volatile uint8_t Rx_flag; // 接收标志

// struct PID
PID_ControllerTypeDef motor1PID_V = {0};
PID_ControllerTypeDef motor2PID_V = {0};
PID_ControllerTypeDef motor1PID_P = {0};
PID_ControllerTypeDef motor2PID_P = {0};
PID_ControllerTypeDef distancePID = {0};
PID_ControllerTypeDef anglePID = {0};
PID_ControllerTypeDef ImuPID = {0};
float pidOutputV1 = 0;
float pidOutputV2 = 0;
float pidOutputYaw = 0;
float pid_out_position1 = 0;
float pid_out_position2 = 0;
float Position_out_Y = 0;
float Position_pid_outer1 = 0;
float Position_pid_outer2 = 0;
// float pidOutputBc = 0;
// float pid_end = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
// vofa串口调试函数
void USART_PID_Adjust(uint8_t Motor_n,PID_ControllerTypeDef *pid);
float Get_Data(void);

// 任务执行函数
void CalculateWheelSpeeds(class_Ogpi ball, float* motor1_speed, float* motor2_speed, float* biasX, float* biasY);
void InitializeAll() {
    InitializeOgpi(&red_ball, RED_BALL);
    InitializeOgpi(&blue_ball, BLUE_BALL);
    InitializeOgpi(&yellow_ball, YELLOW_BALL);
    InitializeOgpi(&black_ball, BLACK_BALL);
    InitializeOgpi(&blue_aim, BLUE_AIM);
    InitializeOgpi(&red_aim, RED_AIM);
    InitializeOgpi(&blue_base, BLUE_BASE);
    InitializeOgpi(&red_base, RED_BASE);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM17_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_UART8_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

  RetargetInit(&huart1);
  delay_init(480);
  //JY901s_Init(&JY901s);  // 初始化JY901串口
  PID_Init(&motor1PID_V, 8.8f, 0.066f, 39.9f, 0);
  PID_Init(&motor2PID_V, 8.1f, 0.066f, 38.8f, 0);
  PID_Init(&motor1PID_P, 0.60f, 0, 0, 0);
  PID_Init(&motor2PID_P, 0.66f, 0, 0, 0);
  PID_Init(&distancePID, -0.048f, 0, 0, 0);
  PID_Init(&anglePID, 0.0018f, 0, 0, 0);
  HAL_TIM_Base_Init(&htim3);
  HAL_TIM_Base_Init(&htim4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim17);
  HAL_UART_Receive_IT(&huart1, xUSART1.BuffTemp, 1);
  HAL_UART_Receive_IT( &huart2 , (uint8_t *)&buf ,1);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart8, xUART8.BuffTemp, sizeof(xUART8.BuffTemp));
  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, xUART5.BuffTemp, sizeof(xUART5.BuffTemp));
  //HAL_UARTEx_ReceiveToIdle_DMA(&huart2, JY901s.BuffTemp, sizeof(JY901s.BuffTemp));
  InitializeAll();
  HAL_Delay(500); // 初始化的编码器有误差 需要延时，等编码器稳定后，将位置归0
  wheel1_total_position = 0;
  wheel2_total_position = 0;
  Set_servo1(open);
  HAL_Delay(1000);
  printf("Init OK!\n");

  while (xUART5.ReceiveNum == 0);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // 调试使用
      // Set_motor1(100);
      // Set_servo1(close);
      // HAL_Delay(1000);
      // Set_servo1(205);
      // HAL_Delay(1000);
      // yaw = JY901s.angle.angle[2];

      // 角度调参
      // printf("%f,%f,%f,%f\n",ImuPID.Kp, angle_z, ImuPID.setpoint, angel_velocity_z);

      // 速度环使用
      // printf("%f,%f,%f,%f,%f,%f,%f,%f\n" ,motor1PID_V.Kp, motor2PID_V.Kp, wheel1_speed, wheel1_speedF, wheel2_speed, wheel2_speedF, motor1PID_V.setpoint, motor2PID_V.setpoint);

      // 位置环使用
      // printf("%f,%f,%f,%f,%f,%f,%f,%f\n" ,motor1PID_P.Kp, motor2PID_P.Kp, wheel1_total_position, wheel2_total_position, motor1PID_P.setpoint, motor2PID_P.setpoint, wheel1_speedF, wheel2_speedF);

      // HAL_Delay(2);
      // 获取角度
      // printf("%d,%d,%d,%d\n", XboxData[0], XboxData[1], XboxData[2], XboxData[3]);
      printf("%d\n", state);
      // // ********************************************************************************************
      // 任务代码
    if (task == 0)
    {
        switch (state)
        {
        case 0: // 只夹蓝球
            if (class == BLUE_BALL)
            {
                CalculateWheelSpeeds(blue_ball, &motor1PID_V.setpoint, &motor2PID_V.setpoint, &biasX, &biasY);
                if (biasX < 100 && biasX > -100 && biasY > -100 && biasY < 100)
                {
                    Set_servo1(close);
                    delay_ms(500); // 等待夹取完成
                    state = 1;
                }
                else
                {
                    state = 0;
                }
            }
            break;
        case 1:
            CalculateWheelSpeeds(blue_aim, &motor1PID_V.setpoint, &motor2PID_V.setpoint, &biasX, &biasY);
            if (biasX < 300 && biasX > -300 && biasY > -300 && biasY < 300)
            {
                Set_servo1(open);
                delay_ms(1000); // 等待夹取完成
                state = 2;
            }
            else
            {
                state = 1;
            }
            break;

        case 2:
            CalculateWheelSpeeds(yellow_ball, &motor1PID_V.setpoint, &motor2PID_V.setpoint, &biasX, &biasY);
            if (biasX < 100 && biasX > -100 && biasY > -100 && biasY < 100)
            {
                Set_servo1(close);
                delay_ms(500); // 等待夹取完成
                InitializeOgpi(&blue_aim, BLUE_AIM);
                state = 3;
            }
            else
            {
                back_forward();
                delay_ms(10000);
                state = 2;
            }
            break;

        case 3:
            CalculateWheelSpeeds(blue_aim, &motor1PID_V.setpoint, &motor2PID_V.setpoint, &biasX, &biasY);
            if (biasX < 300 && biasX > -300 && biasY > -300 && biasY < 300)
            {
                Set_servo1(open);
                HAL_Delay(1000); // 等待夹取完成
                state = 4;
            }
            else
            {
                state = 3;
            }
            break;

        case 4:
            CalculateWheelSpeeds(yellow_ball, &motor1PID_V.setpoint, &motor2PID_V.setpoint, &biasX, &biasY);
            if (biasX < 100 && biasX > -100 && biasY > -100 && biasY < 100)
            {
                Set_servo1(close);
                HAL_Delay(500); // 等待夹取完成
                InitializeOgpi(&blue_aim, BLUE_AIM);
                state = 5;
            }
            else
            {
                state = 4;
            }
            break;

        case 5:
            CalculateWheelSpeeds(blue_aim, &motor1PID_V.setpoint, &motor2PID_V.setpoint, &biasX, &biasY);
            if (biasX < 300 && biasX > -300 && biasY > -300 && biasY < 300)
            {
                Set_servo1(open);
                HAL_Delay(1000); // 等待夹取完成
                state = 6;
            }
            else
            {
                state = 5;
            }
            break;

        case 6:
            CalculateWheelSpeeds(black_ball, &motor1PID_V.setpoint, &motor2PID_V.setpoint, &biasX, &biasY);
            if (biasX < 100 && biasX > -100 && biasY > -100 && biasY < 100)
            {
                Set_servo1(close);
                HAL_Delay(500); // 等待夹取完成
                InitializeOgpi(&blue_aim, BLUE_AIM);
                state = 7;
            }
            else
            {
                state = 6;
            }
            break;

        case 7:
            CalculateWheelSpeeds(blue_aim, &motor1PID_V.setpoint, &motor2PID_V.setpoint, &biasX, &biasY);
            if (biasX < 300 && biasX > -300 && biasY > -300 && biasY < 300)
            {
                Set_servo1(open);
                HAL_Delay(1000); // 等待夹取完成
                state = 8;
            }
            else
            {
                state = 7;
            }
            break;

        case 8:
            CalculateWheelSpeeds(black_ball, &motor1PID_V.setpoint, &motor2PID_V.setpoint, &biasX, &biasY);
            if (biasX < 100 && biasX > -100 && biasY > -100 && biasY < 100)
            {
                Set_servo1(close);
                HAL_Delay(500); // 等待夹取完成
                InitializeOgpi(&blue_aim, BLUE_AIM);
                state = 9;
            }
            else
            {
                state = 8;
            }
            break;

        case 9:
            CalculateWheelSpeeds(blue_aim, &motor1PID_V.setpoint, &motor2PID_V.setpoint, &biasX, &biasY);
            if (biasX < 300 && biasX > -300 && biasY > -300 && biasY < 300)
            {
                Set_servo1(open);
                HAL_Delay(1000); // 等待夹取完成
                state = 10;
            }
            else
            {
                state = 9;
            }
            break;

        case 10:
            CalculateWheelSpeeds(blue_ball, &motor1PID_V.setpoint, &motor2PID_V.setpoint, &biasX, &biasY);
            if (biasX < 100 && biasX > -100 && biasY > -100 && biasY < 100)
            {
                Set_servo1(close);
                HAL_Delay(500); // 等待夹取完成
                InitializeOgpi(&blue_aim, BLUE_AIM);
                state = 11;
            }
            else
            {
                state = 10;
            }
            break;

        case 11:
            CalculateWheelSpeeds(blue_aim, &motor1PID_V.setpoint, &motor2PID_V.setpoint, &biasX, &biasY);
            if (biasX < 300 && biasX > -300 && biasY > -300 && biasY < 300)
            {
                Set_servo1(open);
                HAL_Delay(1000); // 等待夹取完成
                state = 12;
            }
            else
            {
                state = 11;
            }
            break;

        case 12:
            CalculateWheelSpeeds(blue_ball, &motor1PID_V.setpoint, &motor2PID_V.setpoint, &biasX, &biasY);
            if (biasX < 100 && biasX > -100 && biasY > -100 && biasY < 100)
            {
                Set_servo1(close);
                HAL_Delay(500); // 等待夹取完成
                InitializeOgpi(&blue_aim, BLUE_AIM);
                state = 13;
            }
            else
            {
                state = 12;
            }
            break;

        case 13:
            CalculateWheelSpeeds(blue_aim, &motor1PID_V.setpoint, &motor2PID_V.setpoint, &biasX, &biasY);
            if (biasX < 300 && biasX > -300 && biasY > -300 && biasY < 300)
            {
                Set_servo1(open);
                HAL_Delay(1000); // 等待夹取完成
                state = 14;
            }
            else
            {
                state = 13;
            }
            break;

        case 14:
            CalculateWheelSpeeds(blue_ball, &motor1PID_V.setpoint, &motor2PID_V.setpoint, &biasX, &biasY);
            if (biasX < 100 && biasX > -100 && biasY > -100 && biasY < 100)
            {
                Set_servo1(close);
                HAL_Delay(500); // 等待夹取完成
                InitializeOgpi(&blue_aim, BLUE_AIM);
                state = 15;
            }
            else
            {
                state = 14;
            }
            break;

        case 15:
            CalculateWheelSpeeds(blue_aim, &motor1PID_V.setpoint, &motor2PID_V.setpoint, &biasX, &biasY);
            if (biasX < 300 && biasX > -300 && biasY > -300 && biasY < 300)
            {
                Set_servo1(open);
                HAL_Delay(1000); // 等待夹取完成
                state = 16;
            }
            else
            {
                state = 15;
            }
            break;

        default:
            break;
        }
    }

      // ********************************************************************************************

      if (XboxData[0] != 0 && XboxData[0] != 1 && XboxData[1] != 0 && XboxData[1] != 1)  // xbox没连接时是9,9,0,0
      {

          // motor1PID_V.setpoint = 0;
          // motor2PID_V.setpoint = 0;   // 罪魁祸首
          // printf("xbox not connected\n");
          // mode = 1;  // 位置环模式
      }
      else
      {
          // printf("xbox connected\n");
          mode = 0;  // 遥控（速度环）模式
          task = 1;
          calculate_target_speeds(XboxData[2], XboxData[3], &SpeedY, &angular_speed);
          if (XboxData[0] == 1)
          {
               Set_servo1(close);
          }
          if (XboxData[1] == 1)
          {
               Set_servo1(open);
          }
          if (SpeedY < 3.99 && SpeedY > -3.99)   // 死区防止静止时抖动
          {
              SpeedY = 0;
              // printf("%f\n", SpeedY);
          }
          if (angular_speed < 0.05 && angular_speed > -0.05)
          {
              angular_speed = 0;
              // printf("%f\n", angular_speed);
          }
          InverseKinematics_differential(SpeedY, angular_speed, WheelDistance, &motor1PID_V.setpoint, &motor2PID_V.setpoint);
      }




    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      // if (JY901s.ReceiveNum)
      // {
      //     JY901s_Process();
      //     printf("ASCII : %f", JY901s.angle.angle[2]);    // 显示数据，以ASCII方式显示，即以字符串的方式显示
      //     JY901s.ReceiveNum = 0;
      // }

      // 有一帧数据就解析Xbox数据

      if (xUART8.ReceiveNum)                                                  // 判断字节数
      {
          Get_Data_Xbox(xUART8.ReceiveData);// 解析Xbox数据
          // printf("%s\n", (char *)xUART8.ReceiveData);    // 显示数据，以ASCII方式显示，即以字符串的方式显示
          // printf("%d,%d,%d,%d\n", XboxData[0], XboxData[1], XboxData[2], XboxData[3]);// 显示换行
          xUART8.ReceiveNum = 0;                                             // 清0接收标记

      }

      if (xUART5.ReceiveNum)// 判断字节数
      {
          Get_Data_Ogpi(xUART5.ReceiveData);                                 // 解析Ogpi数据
          // printf("%s\n", (char *)xUART5.ReceiveData);    // 显示数据，以ASCII方式显示，即以字符串的方式显示
          // printf("%f,%f,%f,%f\n", linear_speed, angular_speed, motor1PID_V.setpoint, motor2PID_V.setpoint);// 显示换行
          printf("%d,%f,%f,%f,%f\n", blue_aim.class, blue_aim.x, blue_aim.y, motor1PID_V.setpoint, motor2PID_V.setpoint);// 显示换行
          xUART5.ReceiveNum = 0;                                             // 清0接收标记

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void CalculateWheelSpeeds(class_Ogpi ball, float* motor1_speed, float* motor2_speed, float* biasX, float* biasY) {
    distancePID.setpoint = Camera_centerY;
    anglePID.setpoint = Camera_centerX;
    *biasX = Camera_centerX - ball.x;
    *biasY = ball.y - Camera_centerY;
    float linear_speed = PID_Compute(&distancePID, ball.y);
    float angular_speed = PID_Compute(&anglePID, ball.x);
    InverseKinematics_differential(linear_speed, angular_speed, WheelDistance, motor1_speed, motor2_speed);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 定时10ms(240000000 / 2400 / 500 = 200)
    if (htim->Instance == htim17.Instance)
    {

        // 获取脉冲
        int16_t pluse1 = COUNTERNUM1;
        int16_t pluse2 = COUNTERNUM2;

        totalAngle1 = pluse1;
        totalAngle2 = pluse2;

        // 计算速度
        wheel1_speed = -((float)(totalAngle1 - RELOADVALUE / 2.0) / ConvertParam) * 200 * WheelCircumference;  // 单位：厘米/秒 （AB相反）
        wheel2_speed = ((float)(totalAngle2 - RELOADVALUE / 2.0) / ConvertParam) * 200 * WheelCircumference;  // 单位：厘米/秒
        // printf("%f,%f,%f,%f\n", wheel1_speed, wheel2_speed, SetSpeed1, SetSpeed2);

        // 计算位置
        wheel1_position = (float)(totalAngle1 - RELOADVALUE / 2.0) / ConvertParam * WheelCircumference;  // 单位：厘米
        wheel1_total_position += wheel1_position;
        wheel2_position = -(float)(totalAngle2 - RELOADVALUE / 2.0) / ConvertParam * WheelCircumference;  // 单位：厘米
        wheel2_total_position += wheel2_position;

        // 均值滤波
        mean_buff1[buff_index1] = wheel1_speed;
        mean_buff2[buff_index1] = wheel2_speed;
        buff_index1 ++;
        mean_buff3[buff_index2++] = JY901s.angle.angle[2];
        wheel1_speedF = mean_fliter(mean_buff1, buff_index1);
        wheel2_speedF = mean_fliter(mean_buff2, buff_index1);
        yawF = mean_fliter(mean_buff3, buff_index2);

        // // 滑动窗口均值滤波
        // wheel1_speed = Filter_SlidingWindowAvg(motor1, wheel1_speed);
        // wheel2_speed = Filter_SlidingWindowAvg(motor2, wheel2_speed);

        // 索引更新
        if (buff_index1 >= fliter_mean_sample1)
        {
            buff_index1 = 0;
            // // 记录上一次的速度
            // last_wheel1_speed = wheel1_speed;
        }
         if (buff_index2 >= fliter_mean_sample1)
        {
            buff_index2 = 0;
            // // 记录上一次的速度
            // last_wheel2_speed = wheel2_speed;
        }

//      printf("%d, %d\n", totalAngle, lastAngle);                                // 调试使用
//      printf("%f, %f\n", (float)(totalAngle - lastAngle), speed);               // 调试使用
//      lastAngle = totalAngle;
        //
        // // 速度PID
        // pidOutputV1 = PID_Velocity(&motor1PID_V, wheel1_speedF);
        // pidOutputV2 = PID_Velocity(&motor2PID_V, wheel2_speedF);
        //
        // // 输出PWM（用于调试速度PID)
        // Set_motor1(pidOutputV1);
        // Set_motor2(pidOutputV2);

        if (mode == 1)
        {
            // 位置PID ??? 有问题
            pid_out_position1 = PID_Position(&motor1PID_P, wheel1_total_position);
            pid_out_position2 = PID_Position(&motor2PID_P, wheel2_total_position);

            Position_out_Y = (pid_out_position1 + pid_out_position2) / 2.0;

            // 角度PID
            pidOutputYaw = PID_Turn(&ImuPID, angle_z, angel_velocity_z);

            InverseKinematics_differential(Position_out_Y, pidOutputYaw, WheelDistance, &Position_pid_outer1, &Position_pid_outer2);

            // 速度PID
            pidOutputV1 = PID_Velocity(&motor1PID_V, Position_pid_outer1);
            pidOutputV2 = PID_Velocity(&motor2PID_V, Position_pid_outer2);

            Set_motor1(pidOutputV1);
            Set_motor2(pidOutputV2);
        }
        else
        {
            // 速度PID
            pidOutputV1 = PID_Velocity(&motor1PID_V, wheel1_speedF);
            pidOutputV2 = PID_Velocity(&motor2PID_V, wheel2_speedF);

            // 输出PWM（用于调试速度PID)
            Set_motor1(pidOutputV1);
            Set_motor2(pidOutputV2);
        }
        // printf("%f,%f,%f\n", pidOutputV1, pidOutputV2, SetSpeed1);

        // // 计算角度PID
        // pidOutputYaw = PID_Turn(&ImuPID, yaw, fGyro[2]);
        //
        // // 输出PWM（用于调试角度PID）
        // if (pidOutputYaw > 0)           // 方向不一定正确，需要根据实际情况调整
        // {
        //     Set_pulse1(pidOutputYaw);
        //     Set_pulse2(-pidOutputYaw);
        // }
        // else
        // {
        //     Set_pulse1(-pidOutputYaw);
        //     Set_pulse2(pidOutputYaw);
        // }

        // printf("%f,%f,%f,%f,%f,%f,%f,%f,%f\n" ,motor1PID_V.Kp, wheel1_speed, wheel2_speed, SetSpeed1, SetSpeed2, pitch, SetSpeed6, COUNTERNUM1, COUNTERNUM2);

        // 重置计数器 （重置到重装值的中间值，也可以重置到0，不过反转得到的数需要取补码）
        __HAL_TIM_SetCounter(&htim1, RELOADVALUE / 2);
        __HAL_TIM_SetCounter(&htim2, RELOADVALUE / 2);

    //  printf("%f, %f, %f, %f, %f, %f\n", motor1PID_V.Kp, motor1PID_V.Ki, motor1PID_V.Kd, speed, SetSpeed, (float)(totalAngle - RELOADVALUE / 2.0)); // 调试使用
    }

}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // printf("enter\r\n");
    // 未调试
    // if (huart == &huart1)  // 判断是否是串口1产生的中断
    // {
    //     __HAL_UNLOCK(huart);
    //     xUSART1.ReceiveNum ++;                        // 每接收到一个数据，接收长度加1
    //     xUSART1.ReceiveData[xUSART1.ReceiveNum - 1] = xUSART1.BuffTemp[0];  // 将接收到的数据存入缓存数组
    //
    //     if (xUSART1.BuffTemp[0] == '!')         // 判断是否接收到结束标志（这里以0x21为例，可以根据实际情况修改）
    //     {
    //         // printf("RXLen=%d\r\n", xUSART1.ReceiveNum);  // 输出接收到的指令长度
    //         // for (int i = 0; i < xUSART1.ReceiveNum; i++)
    //         //    printf("UART xUSART1.ReceiveData[%d] = %c\r\n", i, xUSART1.ReceiveData[i]);  // 输出接收到的完整指令
    //
    //         USART_PID_Adjust(1, &motor1PID_V);  // 解析指令并赋值到对应变量（这里示例传入参数1，可根据实际情况修改）
    //         USART_PID_Adjust(2, &motor2PID_V);  // 解析指令并赋值到对应变量（这里示例传入参数1，可根据实际情况修改）
    //         USART_PID_Adjust(6, &ImuPID);  // 解析指令并赋值到对应变量（这里示例传入参数1，可根据实际情况修改）
    //         memset(xUSART1.ReceiveData, 0, sizeof(xUSART1.ReceiveData));  // 清空接收缓存
    //         xUSART1.ReceiveNum = 0;  // 重置接收长度计数
    //     }
    //     xUSART1.BuffTemp[0] = 0;  // 清空接收缓冲
    //     HAL_UARTEx_ReceiveToIdle_DMA(&huart1, xUSART1.BuffTemp, sizeof(xUSART1.BuffTemp));
    // }
    // 未调试
    // if (huart == &huart2)
    // {
    //    __HAL_UNLOCK(huart);
    //     JY901s.ReceiveNum  = Size;                                                          // 把接收字节数，存入结构体
    //     memset(JY901s.angle.angle, 0, sizeof(JY901s.angle.angle));                       // 清0前一帧的接收数据
    //     memset(JY901s.w.w, 0, sizeof(JY901s.w.w));
    //     memset(JY901s.acc.a, 0, sizeof(JY901s.acc.a));
    //     JY901s_Process();
    //     printf("ASCII : %f", JY901s.angle.angle[2]);    // 显示数据，以ASCII方式显示，即以字符串的方式显示
    //     HAL_UARTEx_ReceiveToIdle_DMA(&huart2, JY901s.BuffTemp, sizeof(JY901s.BuffTemp));   // 再次开启DMA空闲中断; 每当接收完指定长度，或者产生空闲中断时，就会来到这个
    // }

    if (huart == &huart5)                                                                    // 判断串口
    {
        __HAL_UNLOCK(huart);                                                                 // 解锁串口状态
        xUART5.ReceiveNum  = Size;                                                          // 把接收字节数，存入结构体xUSART8.ReceiveNum，以备使用
        memset(xUART5.ReceiveData, 0, sizeof(xUART5.ReceiveData));                         // 清0前一帧的接收数据
        memcpy(xUART5.ReceiveData, xUART5.BuffTemp, Size);                                 // 把新数据，从临时缓存中，复制到xUSART8.ReceiveData[], 以备使用
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, xUART5.BuffTemp, sizeof(xUART5.BuffTemp));   // 再次开启DMA空闲中断; 每当接收完指定长度，或者产生空闲中断时，就会来到这个
        // 其实，在CubeMX配置中，DMA有一个选项 ：Mode的circular, 可以让DMA进行连续地的工作，接收完成后，无需在回调函数里再次开启DMA 。但是，目前的CubeMX版本(V6.10），这个参数的选择，会使我们上面的DMA接收与发送，相冲突。那我们二选一好了，自行手工调用。
    }

    if (huart == &huart8)                                                                    // 判断串口
    {
        __HAL_UNLOCK(huart);                                                                 // 解锁串口状态
        xUART8.ReceiveNum  = Size;                                                          // 把接收字节数，存入结构体xUSART8.ReceiveNum，以备使用
        memset(xUART8.ReceiveData, 0, sizeof(xUART8.ReceiveData));                         // 清0前一帧的接收数据
        memcpy(xUART8.ReceiveData, xUART8.BuffTemp, Size);                                 // 把新数据，从临时缓存中，复制到xUSART8.ReceiveData[], 以备使用
        HAL_UARTEx_ReceiveToIdle_DMA(&huart8, xUART8.BuffTemp, sizeof(xUART8.BuffTemp));   // 再次开启DMA空闲中断; 每当接收完指定长度，或者产生空闲中断时，就会来到这个
// 其实，在CubeMX配置中，DMA有一个选项 ：Mode的circular, 可以让DMA进行连续地的工作，接收完成后，无需在回调函数里再次开启DMA 。但是，目前的CubeMX版本(V6.10），这个参数的选择，会使我们上面的DMA接收与发送，相冲突。那我们二选一好了，自行手工调用。
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    if (UartHandle->Instance == USART1)  // 判断是否是串口1产生的中断
    {
        xUSART1.ReceiveNum ++;                        // 每接收到一个数据，接收长度加1
        xUSART1.ReceiveData[xUSART1.ReceiveNum - 1] = xUSART1.BuffTemp[0];  // 将接收到的数据存入缓存数组

        if (xUSART1.BuffTemp[0] == '!')         // 判断是否接收到结束标志（这里以0x21为例，可以根据实际情况修改）
        {
            // printf("RXLen=%d\r\n", xUSART1.ReceiveNum);  // 输出接收到的指令长度
            // for (int i = 0; i < xUSART1.ReceiveNum; i++)
            //    printf("UART xUSART1.ReceiveData[%d] = %c\r\n", i, xUSART1.ReceiveData[i]);  // 输出接收到的完整指令

            // USART_PID_Adjust(1, &motor1PID_V);  // 解析指令并赋值到对应变量（这里示例传入参数1，可根据实际情况修改）
            // USART_PID_Adjust(2, &motor2PID_V);  // 解析指令并赋值到对应变量（这里示例传入参数1，可根据实际情况修改）
            USART_PID_Adjust(6, &ImuPID);  // 解析指令并赋值到对应变量（这里示例传入参数1，可根据实际情况修改）
            USART_PID_Adjust(8, &motor1PID_P);  // 解析指令并赋值到对应变量（这里示例传入参数1，可根据实际情况修改）
            USART_PID_Adjust(9, &motor2PID_P);  // 解析指令并赋值到对应变量（这里示例传入参数1，可根据实际情况修改）
            memset(xUSART1.ReceiveData, 0, sizeof(xUSART1.ReceiveData));  // 清空接收缓存
            xUSART1.ReceiveNum = 0;  // 重置接收长度计数
        }
        xUSART1.BuffTemp[0] = 0;  // 清空接收缓冲
        HAL_UART_Receive_IT(&huart1, (uint8_t *)xUSART1.BuffTemp, 1);  // 重新启动串口中断接收下一个字符
    }

    if(UartHandle->Instance == USART2)
    {

        if( buf == 0x55 && recieve_flag == Free )
        {
            str_data[str_index] = buf ;
            str_index++;
            recieve_flag = Receiveing ;
        }
        else if( recieve_flag == Receiveing )
        {
            str_data[str_index] = buf ;
            str_index++;
            if( str_index > 70 )
            {
                str_index = 0 ;
            }
            if( str_index == 11 )
            {
                uint8_t item = Test_data( str_data , 10 ) ;
                if( item == str_data[10] )
                {
                    data_ok = 1 ;
                    switch ( str_data[1])
                    {
                    case 0x51:

                        accelerate_x = (float)((((short)str_data[3]<<8)|str_data[2])*1.0/32768*156.8);
                        accelerate_y = (float)((((short)str_data[5]<<8)|str_data[4])*1.0/32768*156.8);
                        accelerate_z = (float)((((short)str_data[7]<<8)|str_data[6])*1.0/32768*156.8);
                        break;

                    case 0x52:
                        angel_velocity_x = (float)( ((short)str_data[3]<<8|str_data[2])*1.0/32768*2000 );
                        angel_velocity_y = (float)( ((short)str_data[5]<<8|str_data[4])*1.0/32768*2000 );
                        angel_velocity_z = (float)( ((short)str_data[7]<<8|str_data[6])*1.0/32768*2000 );
                        break;

                    case 0x53:
                        angle_x = (float)( ((short)str_data[3]<<8|str_data[2])*1.0/32768*180 );
                        angle_y = (float)( ((short)str_data[5]<<8|str_data[4])*1.0/32768*180 );
                        angle_z = (float)( ((short)str_data[7]<<8|str_data[6])*1.0/32768*180 );
                        if ((angle_z > 358 && angle_z < 360) || (angle_z < 2 && angle_z > 0))
                        {
                            angle_z = 0;
                        }
                        break;

                    default:
                        break;
                    }
                    recieve_flag = Free ;

                }
                str_index = 0 ;
                recieve_flag = Free ;
            }
        }

        HAL_UART_Receive_IT(&huart2 , (uint8_t *)&buf ,1);
    }


}

// 解析从指令缓存中提取数据
float Get_Data(void)
{
    float Decimal = 0;            // 小数数据
    float Integer = 0;            // 整数数据
    uint8_t data_Decimal_len = 0; // 小数数据长度
    uint8_t data_Integer_len = 0; // 整数数据长度
    uint8_t data_Point_Num = 0;   // 小数点位置
    uint8_t data_Start_Num = 0;   // 数据位开始位置
    uint8_t data_End_Num = 0;     // 数据位结束位置
    uint8_t minus_Flag = 0;       // 负数标志
    float data_return = 0;        // 解析得到的数据
    // 查找等号、小数点和感叹号的位置
    for (uint8_t i = 0; i < 200; i++)
    {
        if (xUSART1.ReceiveData[i] == '=')
            data_Start_Num = i + 1;  // 找到等号后面的位置作为数据起始位
        if (xUSART1.ReceiveData[i] == '.')
            data_Point_Num = i;
        if (xUSART1.ReceiveData[i] == '!')
        {
            data_End_Num = i - 1;  // 找到感叹号前面的位置作为数据结束位
            break;
        }
    }

    // 判断数据是否为负数
    if (xUSART1.ReceiveData[data_Start_Num] == '-')
    {
        data_Start_Num += 1;  // 如果是负数，数据起始位后移一位
        minus_Flag = 1;       // 设置负数标志
    }
    // 计算整数长度
    data_Integer_len = data_Point_Num - data_Start_Num;
    // 计算小数长度
    data_Decimal_len = data_End_Num - data_Point_Num;

    // 计算整数数据
    Integer = data_Integer_calculate(data_Integer_len, data_Start_Num, xUSART1.ReceiveData);

    // 计算小数数据
    Decimal = (data_Decimal_calculate(data_Decimal_len, data_Point_Num, xUSART1.ReceiveData));

    data_return = Integer + Decimal;
    if (minus_Flag == 1)
        data_return = -data_return;  // 如果是负数，取负值

    // printf("data_return:%lf\n", data_return);

    return data_return;  // 返回解析得到的数据
}

// 根据接收到的指令内容进行PID参数调整
void USART_PID_Adjust(uint8_t Motor_n, PID_ControllerTypeDef *pid)
{
    float data_Get = Get_Data();  // 解析得到的数据
    // 根据指令内容赋值到对应的PID参数或目标变量
    if (Motor_n == 1)  // 电机1
    {

        if (xUSART1.ReceiveData[0] == 'P' && xUSART1.ReceiveData[1] == '1')
            pid->Kp = data_Get;     // 速度环P参数
        else if (xUSART1.ReceiveData[0] == 'I' && xUSART1.ReceiveData[1] == '1')
            pid->Ki = data_Get;     // 速度环I参数
        else if (xUSART1.ReceiveData[0] == 'D' && xUSART1.ReceiveData[1] == '1')
            pid->Kd = data_Get;     // 速度环D参数
        else if ((xUSART1.ReceiveData[0] == 'S' && xUSART1.ReceiveData[1] == 'p') && xUSART1.ReceiveData[2] == 'e')
            pid->setpoint = data_Get;     // 目标速度
            SetSpeed1 = pid->setpoint;
    }

    if (Motor_n == 2)  // 电机2
    {
        if (xUSART1.ReceiveData[0] == 'P' && xUSART1.ReceiveData[1] == '2')
            pid->Kp = data_Get;     // 速度环P参数
        else if (xUSART1.ReceiveData[0] == 'I' && xUSART1.ReceiveData[1] == '2')
            pid->Ki = data_Get;     // 速度环I参数
        else if (xUSART1.ReceiveData[0] == 'D' && xUSART1.ReceiveData[1] == '2')
            pid->Kd = data_Get;     // 速度环D参数
        else if ((xUSART1.ReceiveData[0] == 'S' && xUSART1.ReceiveData[1] == 'p') && xUSART1.ReceiveData[2] == 'e')
            pid->setpoint = data_Get;     // 目标速度
            SetSpeed2 = pid->setpoint;
    }

    if (Motor_n == 6)  // IMU
    {
        if (xUSART1.ReceiveData[0] == 'P' && xUSART1.ReceiveData[1] == '6')
            pid->Kp = data_Get;     // 速度环P参数
        else if (xUSART1.ReceiveData[0] == 'D' && xUSART1.ReceiveData[1] == '6')
            pid->Kd = data_Get;     // 速度环D参数
        else if ((xUSART1.ReceiveData[0] == 'S' && xUSART1.ReceiveData[1] == 'p') && xUSART1.ReceiveData[2] == 'e')
            pid->setpoint = data_Get;  // 目标速度
            SetSpeed6 = pid->setpoint;
    }

    if (Motor_n == 8)  // 电机1位置环
    {
        if (xUSART1.ReceiveData[0] == 'P' && xUSART1.ReceiveData[1] == '8')
            pid->Kp = data_Get;     // 位置环P参数
        else if (xUSART1.ReceiveData[0] == 'I' && xUSART1.ReceiveData[1] == '8')
            pid->Ki = data_Get;     // 位置环I参数
        else if (xUSART1.ReceiveData[0] == 'D' && xUSART1.ReceiveData[1] == '8')
            pid->Kd = data_Get;     // 位置环D参数)
        else if ((xUSART1.ReceiveData[0] == 'P' && xUSART1.ReceiveData[1] == 'o') && xUSART1.ReceiveData[2] == 's')
            pid->setpoint = data_Get;  // 目标速度)
    }

    if (Motor_n == 9)  // 电机2位置环
    {
        if (xUSART1.ReceiveData[0] == 'P' && xUSART1.ReceiveData[1] == '9')
            pid->Kp = data_Get;     // 位置环P参数
        else if (xUSART1.ReceiveData[0] == 'I' && xUSART1.ReceiveData[1] == '9')
            pid->Ki = data_Get;     // 位置环I参数
        else if (xUSART1.ReceiveData[0] == 'D' && xUSART1.ReceiveData[1] == '9')
            pid->Kd = data_Get;     // 位置环D参数)
        else if ((xUSART1.ReceiveData[0] == 'P' && xUSART1.ReceiveData[1] == 'o') && xUSART1.ReceiveData[2] == 's')
            pid->setpoint = data_Get;  // 目标速度)
    }
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
