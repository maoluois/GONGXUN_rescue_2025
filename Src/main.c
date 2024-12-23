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
#include "memorymap.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "retarget.h"
#include "control.h"
#include "fliter.h"
#include "Algorithm.h"
#include "pid.h"
#include "wit_c_sdk.h"
#include <stdio.h>
#include <string.h>

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

// encoder PV
int32_t totalAngle1 = 0;       // 总的角度
int32_t totalAngle2 = 0;
int32_t lastAngle = 0;        // 上一次的角度
// int16_t loopNum1 = 0;          // 防超上限
// int16_t loopNum2 = 0;
float wheel1_speed = 0;              // 轮子速度 （单位：m/s）
float wheel2_speed = 0;

// usart PV
uint8_t RxBuffer[1];          // 串口接收缓冲
uint16_t RxLine = 0;          // 指令长度
uint8_t DataBuff[200];        // 指令内容
float SetSpeed1 = 0;          // 设置目标速度（单位：m/s）
float SetSpeed2 = 0;
float SetSpeed6 = 0;

// fliter PV
float mean_buff1[fliter_buffer_size];             // 滤波缓冲
float mean_buff2[fliter_buffer_size];
float mean_buff3[fliter_buffer_size];
int buff_index1 = 0;                // 滤波缓冲区索引
int buff_index2 = 0;

// Imu JY901s PV
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
// const uint32_t c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
float fAcc[3], fGyro[3], fAngle[3];
float pitch = 0, roll = 0, yaw = 0;

// struct PID
PID_ControllerTypeDef motor1PID;
PID_ControllerTypeDef motor2PID;
PID_ControllerTypeDef ImuPID;
float pidOutputV1 = 0;
float pidOutputV2 = 0;
float pidOutputYaw = 0;
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

// JY901s配置函数
// static void CmdProcess(void);
static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);

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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM17_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  RetargetInit(&huart1);
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  WitSerialWriteRegister(SensorUartSend);
  WitRegisterCallBack(SensorDataUpdata);
  WitDelayMsRegister(Delayms);
  AutoScanSensor();
  HAL_TIM_Base_Init(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim17);
  HAL_UART_Receive_IT(&huart1, RxBuffer, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      Set_pulse1(-300);
      Set_pulse2(100);
      // 获取角度
      // CmdProcess();
      if(s_cDataUpdate)
      {
          for(int i = 0; i < 3; i++)
          {
              fAcc[i] = (float)sReg[AX+i] / 32768.0f * 16.0f;
              fGyro[i] = (float)sReg[GX+i] / 32768.0f * 2000.0f;
              fAngle[i] = (float)sReg[Roll+i] / 32768.0f * 180.0f;
          }
          if(s_cDataUpdate & ACC_UPDATE)
          {
              //printf("acc:%.3f %.3f %.3f\r\n", fAcc[0], fAcc[1], fAcc[2]);
              s_cDataUpdate &= ~ACC_UPDATE;
          }
          if(s_cDataUpdate & GYRO_UPDATE)
          {
              //printf("gyro:%.3f %.3f %.3f\r\n", fGyro[0], fGyro[1], fGyro[2]);
              s_cDataUpdate &= ~GYRO_UPDATE;
          }
          if(s_cDataUpdate & ANGLE_UPDATE)
          {
              //printf("angle:%.3f %.3f %.3f\r\n", fAngle[0], fAngle[1], fAngle[2]);
              // printf("%.3f,%.3f,%.3f\n", fAngle[0], fAngle[1], fAngle[2]);
              s_cDataUpdate &= ~ANGLE_UPDATE;
          }
          if(s_cDataUpdate & MAG_UPDATE)
          {
              //printf("mag:%d %d %d\r\n", sReg[HX], sReg[HY], sReg[HZ]);
              s_cDataUpdate &= ~MAG_UPDATE;
          }

      }
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
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 定时1ms(480000000 / 480 / 1000 = 10000)
    if (htim->Instance == htim17.Instance)
    {
        // 获取脉冲
        int16_t pluse1 = COUNTERNUM1;
        int16_t pluse2 = COUNTERNUM2;
        //    printf("%f,%f\n", COUNTERNUM1, COUNTERNUM2);

        totalAngle1 = pluse1;
        totalAngle2 = pluse2;

        // 计算速度
        wheel1_speed = ((float)(totalAngle1 - RELOADVALUE / 2.0) / convert_param) * 1000 * wheel_circumference;  // 单位：米/秒
        wheel2_speed = ((float)(totalAngle2 - RELOADVALUE / 2.0) / convert_param) * 1000 * wheel_circumference;  // 单位：米/秒


        // 滤波
        mean_buff1[buff_index1++] = wheel1_speed;
        mean_buff2[buff_index1++] = wheel2_speed;
        mean_buff3[buff_index2++] = fAngle[2];
        wheel1_speed = mean_fliter(mean_buff1, buff_index1);
        wheel2_speed = mean_fliter(mean_buff2, buff_index1);
        yaw = mean_fliter(mean_buff3, buff_index2);


        // 索引更新
        if (buff_index1 >= fliter_mean_sample1)
        {
            buff_index1 = 0;
        }
         if (buff_index2 >= fliter_mean_sample1)
        {
            buff_index2 = 0;
        }
//      printf("%d, %d\n", totalAngle, lastAngle);                                // 调试使用
//      printf("%f, %f\n", (float)(totalAngle - lastAngle), speed);               // 调试使用
//      lastAngle = totalAngle;

        // 计算速度PID
        pidOutputV1 = PID_Velocity(&motor1PID, wheel1_speed);
        pidOutputV2 = PID_Velocity(&motor2PID, wheel2_speed);

        // 输出PWM（用于调试速度PID）
        Set_pulse1(pidOutputV1);
        Set_pulse2(pidOutputV2);

        // // 计算角度PID
        // pidOutputYaw = PID_Turn_Calc(&ImuPID, yaw, fGyro[2]);
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


        // if ((SetSpeed - speed1) > 0.01 || (SetSpeed - speed1) < -0.01) {
        //           printf("哈哈我又来啦");
        // Set_pulse1(pidoutput1);
        // }
        //
        // if ((SetSpeed - speed2) > 0.01 || (SetSpeed - speed2) < -0.01) {
        //           printf("哈哈我又来啦");
        // Set_pulse2(pidoutput2);
        // }

        // 重置计数器 （重置到重装值的中间值，也可以重置到0，不过反转得到的数需要取补码）
        __HAL_TIM_SetCounter(&htim1, RELOADVALUE / 2);
        __HAL_TIM_SetCounter(&htim2, RELOADVALUE / 2);

    //  printf("%f, %f, %f, %f, %f, %f\n", motor1PID.Kp, motor1PID.Ki, motor1PID.Kd, speed, SetSpeed, (float)(totalAngle - RELOADVALUE / 2.0)); // 调试使用

    }

}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    if (UartHandle->Instance == USART1)  // 判断是否是串口1产生的中断
    {

        RxLine++;                        // 每接收到一个数据，接收长度加1
        DataBuff[RxLine - 1] = RxBuffer[0];  // 将接收到的数据存入缓存数组

        if (RxBuffer[0] == '!')         // 判断是否接收到结束标志（这里以0x21为例，可以根据实际情况修改）
        {
            // printf("RXLen=%d\r\n", RxLine);  // 输出接收到的指令长度
            // for (int i = 0; i < RxLine; i++)
            //    printf("UART DataBuff[%d] = %c\r\n", i, DataBuff[i]);  // 输出接收到的完整指令

            USART_PID_Adjust(1, &motor1PID);  // 解析指令并赋值到对应变量（这里示例传入参数1，可根据实际情况修改）
            USART_PID_Adjust(2, &motor2PID);  // 解析指令并赋值到对应变量（这里示例传入参数1，可根据实际情况修改）
            USART_PID_Adjust(6, &ImuPID);  // 解析指令并赋值到对应变量（这里示例传入参数1，可根据实际情况修改）

            memset(DataBuff, 0, sizeof(DataBuff));  // 清空接收缓存
            RxLine = 0;  // 重置接收长度计数
        }

        RxBuffer[0] = 0;  // 清空接收缓冲
        HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuffer, 1);  // 重新启动串口中断接收下一个字符
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
        if (DataBuff[i] == '=')
            data_Start_Num = i + 1;  // 找到等号后面的位置作为数据起始位
        if (DataBuff[i] == '.')
            data_Point_Num = i;
        if (DataBuff[i] == '!')
        {
            data_End_Num = i - 1;  // 找到感叹号前面的位置作为数据结束位
            break;
        }
    }

    // 判断数据是否为负数
    if (DataBuff[data_Start_Num] == '-')
    {
        data_Start_Num += 1;  // 如果是负数，数据起始位后移一位
        minus_Flag = 1;       // 设置负数标志
    }
    // 计算整数长度
    data_Integer_len = data_Point_Num - data_Start_Num;
    // 计算小数长度
    data_Decimal_len = data_End_Num - data_Point_Num;

    // 计算整数数据
    if (data_Integer_len != 0) // 为两位数
    {
        if (data_Integer_len == 1)
            Integer = (float)(DataBuff[data_Start_Num] - 48);
        else if (data_Integer_len == 2)
            Integer = (float)(DataBuff[data_Start_Num] - 48) * 10 + (float)(DataBuff[data_Start_Num + 1] - 48);
        else if (data_Integer_len == 3)
            Integer = (float)(DataBuff[data_Start_Num] - 48) * 100 + (float)(DataBuff[data_Start_Num + 1] - 48) * 10 +
            (float)(DataBuff[data_Start_Num + 2] - 48);
        else if (data_Integer_len == 4)
            Integer = (float)(DataBuff[data_Start_Num] - 48) * 1000 + (float)(DataBuff[data_Start_Num + 1] - 48) * 100 +
            (float)(DataBuff[data_Start_Num + 3] - 48) * 10 + (float)(DataBuff[data_Start_Num + 4] - 48);
    }

    // 计算小数数据
    if (data_Decimal_len != 0) // 为个位数
    {
        if (data_Decimal_len == 1)
            Decimal = (float)(DataBuff[data_End_Num] - 48) * 0.1f;
        else if (data_Decimal_len == 2)
            Decimal = (float)(DataBuff[data_End_Num - 1] - 48) * 0.1f + (float)(DataBuff[data_End_Num] - 48) * 0.01f;
        else if (data_Decimal_len == 3)
            Decimal = (float)(DataBuff[data_End_Num - 2] - 48) * 0.1f + (float)(DataBuff[data_End_Num - 1] - 48) * 0.01f +
                      (float)(DataBuff[data_End_Num] - 48) * 0.001f;
        else if (data_Decimal_len == 4)
            Decimal = (float)(DataBuff[data_End_Num - 3] - 48) * 0.1f + (float)(DataBuff[data_End_Num - 2] - 48) * 0.01f +
                      (float)(DataBuff[data_End_Num - 1] - 48) * 0.001f + (float)(DataBuff[data_End_Num] - 48) * 0.0001f;
        else if (data_Decimal_len == 5)
            Decimal = (float)(DataBuff[data_End_Num - 4] - 48) * 0.1f + (float)(DataBuff[data_End_Num - 3] - 48) * 0.01f +
                      (float)(DataBuff[data_End_Num - 2] - 48) * 0.001f + (float)(DataBuff[data_End_Num - 1] - 48) * 0.0001f +
                      (float)(DataBuff[data_End_Num] - 48) * 0.00001f;
        else if (data_Decimal_len == 6)
            Decimal = (float)(DataBuff[data_End_Num - 5] - 48) * 0.1f + (float)(DataBuff[data_End_Num - 4] - 48) * 0.01f +
                      (float)(DataBuff[data_End_Num - 3] - 48) * 0.001f + (float)(DataBuff[data_End_Num - 2] - 48) * 0.0001f +
                      (float)(DataBuff[data_End_Num - 1] - 48) * 0.00001f + (float)(DataBuff[data_End_Num] - 48) * 0.000001f;
    }
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

        if (DataBuff[0] == 'P' && DataBuff[1] == '1')
            pid->Kp = data_Get;     // 速度环P参数
        else if (DataBuff[0] == 'I' && DataBuff[1] == '1')
            pid->Ki = data_Get;     // 速度环I参数
        else if (DataBuff[0] == 'D' && DataBuff[1] == '1')
            pid->Kd = data_Get;     // 速度环D参数
        else if ((DataBuff[0] == 'S' && DataBuff[1] == 'p') && DataBuff[2] == 'e')
            pid->setpoint = data_Get;     // 目标速度
            SetSpeed1 = pid->setpoint;
    }

    if (Motor_n == 2)  // 电机2
    {
        if (DataBuff[0] == 'P' && DataBuff[1] == '2')
            pid->Kp = data_Get;     // 速度环P参数
        else if (DataBuff[0] == 'I' && DataBuff[1] == '2')
            pid->Ki = data_Get;     // 速度环I参数
        else if (DataBuff[0] == 'D' && DataBuff[1] == '2')
            pid->Kd = data_Get;     // 速度环D参数
        else if ((DataBuff[0] == 'S' && DataBuff[1] == 'p') && DataBuff[2] == 'e')
            pid->setpoint = data_Get;     // 目标速度
            SetSpeed2 = pid->setpoint;
    }

    if (Motor_n == 6)  // IMU
    {
        if (DataBuff[0] == 'P' && DataBuff[1] == '6')
            pid->Kp = data_Get;     // 速度环P参数
        else if (DataBuff[0] == 'D' && DataBuff[1] == '6')
            pid->Kd = data_Get;     // 速度环D参数
        else if ((DataBuff[0] == 'S' && DataBuff[1] == 'p') && DataBuff[2] == 'e')
            pid->setpoint = data_Get;  // 目标速度
            SetSpeed6 = pid->setpoint;
    }
}

void CopeCmdData(unsigned char ucData)
{
	 static unsigned char s_ucData[50], s_ucRxCnt = 0;

	 s_ucData[s_ucRxCnt++] = ucData;
	 if(s_ucRxCnt<3)return;										//Less than three data returned
	 if(s_ucRxCnt >= 50) s_ucRxCnt = 0;
	 if(s_ucRxCnt >= 3)
	 {
		 if((s_ucData[1] == '\r') && (s_ucData[2] == '\n'))
		 {
		  	s_cCmd = s_ucData[0];
//			  printf("%c", s_cCmd);
			  memset(s_ucData,0,50);
			  s_ucRxCnt = 0;
	   }
		 else
		 {
			 s_ucData[0] = s_ucData[1];
			 s_ucData[1] = s_ucData[2];
			 s_ucRxCnt = 2;
			}
	  }
}

// static void ShowHelp(void)
// {
// 	printf("\r\n************************	 WIT_SDK_DEMO	************************");
// 	printf("\r\n************************          HELP           ************************\r\n");
// 	printf("UART SEND:a\\r\\n   Acceleration calibration.\r\n");
// 	printf("UART SEND:m\\r\\n   Magnetic field calibration,After calibration send:   e\\r\\n   to indicate the end\r\n");
// 	printf("UART SEND:U\\r\\n   Bandwidth increase.\r\n");
// 	printf("UART SEND:u\\r\\n   Bandwidth reduction.\r\n");
// 	printf("UART SEND:B\\r\\n   Baud rate increased to 115200.\r\n");
// 	printf("UART SEND:b\\r\\n   Baud rate reduction to 9600.\r\n");
// 	printf("UART SEND:R\\r\\n   The return rate increases to 10Hz.\r\n");
// 	printf("UART SEND:r\\r\\n   The return rate reduction to 1Hz.\r\n");
// 	printf("UART SEND:C\\r\\n   Basic return content: acceleration, angular velocity, angle, magnetic field.\r\n");
// 	printf("UART SEND:c\\r\\n   Return content: acceleration.\r\n");
// 	printf("UART SEND:h\\r\\n   help.\r\n");
// 	printf("******************************************************************************\r\n");
// }

// static void CmdProcess(void)
// {
// 	switch(s_cCmd)
// 	{
// 		case 'a':
// //			printf("yes");
// 			if(WitStartAccCali() != WIT_HAL_OK)
// 				printf("\r\nSet AccCali Error\r\n");
// 			break;
// 		case 'm':
// 			if(WitStartMagCali() != WIT_HAL_OK)
// 				printf("\r\nSet MagCali Error\r\n");
// 			break;
// 		case 'e':
// 			if(WitStopMagCali() != WIT_HAL_OK)
// 				printf("\r\nSet MagCali Error\r\n");
// 			break;
// 		case 'u':
// 			if(WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK)
// 				printf("\r\nSet Bandwidth Error\r\n");
// 			break;
// 		case 'U':
// 			if(WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK)
// 				printf("\r\nSet Bandwidth Error\r\n");
// 			break;
// 		case 'B':
// 			if(WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK)
// 				printf("\r\nSet Baud Error\r\n");
// 			else
// 				MX_USART2_UART_Init(115200);
// 			break;
// 		case 'b':
// 			if(WitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK)
// 				printf("\r\nSet Baud Error\r\n");
// 			else
// 				MX_USART2_UART_Init(9600);
// 			break;
// 		case 'R':
// 			if(WitSetOutputRate(RRATE_10HZ) != WIT_HAL_OK)
// 				printf("\r\nSet Rate Error\r\n");
// 			break;
// 		case 'r':
// 			if(WitSetOutputRate(RRATE_1HZ) != WIT_HAL_OK)
// 				printf("\r\nSet Rate Error\r\n");
// 			break;
// 		case 'C':
// 			if(WitSetContent(RSW_ACC|RSW_GYRO|RSW_ANGLE|RSW_MAG) != WIT_HAL_OK)
// 				printf("\r\nSet RSW Error\r\n");
// 			break;
// 		case 'c':
// 			if(WitSetContent(RSW_ACC) != WIT_HAL_OK)
// 				printf("\r\nSet RSW Error\r\n");
// 			break;
// 		case 'h':
// 			ShowHelp();
// 			break;
// 	}
// 	s_cCmd = 0xff;
// }

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
	Uart2Send(p_data, uiSize);
}

static void Delayms(uint16_t ucMs)
{
	HAL_Delay(ucMs);
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}

static void AutoScanSensor(void)
{
	int i, iRetry;

	for(i = 1; i < 10; i++)
	{
		iRetry = 2;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			HAL_Delay(100);
			if(s_cDataUpdate != 0)
			{
				printf("*************************************baud find sensor*************************************\r\n\r\n");
				// ShowHelp();
				return ;
			}
			iRetry--;
		}while(iRetry);
	}
	printf("can not find sensor\r\n");
	printf("please check your connection\r\n");
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
