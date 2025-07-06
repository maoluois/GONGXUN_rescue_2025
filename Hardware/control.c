#include "control.h"
/**
  * @brief  设置两个电机速度
  * @param  输入pwm
  * @retval 无
  */
void Set_Motor(float speedl, float speedr)
{
	if (speedl > 0)
    {
        // Set the motor to move forward
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speedl);
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    }
    else
    {
        // Set the motor to move backward
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, -speedl);
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    }
    
    if (speedr > 0)
    {
        // Set the motor to move forward
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, speedr);
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    }
    else
    {
        // Set the motor to move backward
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, -speedr);
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    }
}
