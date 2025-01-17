//
// Created by Administrator on 24-9-23.
//
#include "tim.h"
#include "control.h"
#include <math.h>
#include <stdint.h>
#include "pid.h"

void Set_motor1(float speed)
{
    if (speed > 0)
    {
        // Set the motor to move forward
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed);
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    }
    else
    {
        // Set the motor to move backward
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, -speed);
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);

    }

}
void Set_motor2(float speed)
{
    if (speed > 0)
    {
        // Set the motor to move forward
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, speed);
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    }
    else
    {
        // Set the motor to move backward
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, -speed);
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);

    }
}

void Set_servo1(float angle)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, angle);
}

void Set_postionY(float target_position)
{
    motor1PID_P.setpoint = -target_position;
    motor2PID_P.setpoint = -target_position;
}

void turn_around(void)
{
    motor1PID_V.setpoint = 10;
    motor2PID_V.setpoint = -10;
}

void back_forward(void)
{
   motor1PID_V.setpoint = 30;
   motor2PID_V.setpoint = 30;
}

void stop(void)
{
    motor1PID_V.setpoint = 0;
    motor2PID_V.setpoint = 0;
}
