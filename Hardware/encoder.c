#include "tim.h"

/**
  * @brief  获取两个编码值，并把计数值清零
  * @param  两个编码指的指针
  * @retval 无
  */

  //正负测一下
void Get_Encoder(float* countennum1, float* countennum2)
{
    *countennum1  = __HAL_TIM_GET_COUNTER(&htim1);
    __HAL_TIM_SetCounter(&htim1, 0);
    *countennum2  = __HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SetCounter(&htim2, 0);
}
 