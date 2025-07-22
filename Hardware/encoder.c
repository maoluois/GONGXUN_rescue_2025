#include "tim.h"


//max 100   min -100
/**
  * @brief  获取两个编码值，并把计数值清零
  * @param  两个编码指的指针
  * @retval 无
  */
void Get_Encoder(short* countennuml, short* countennumr)
{
    *countennuml  = -(__HAL_TIM_GET_COUNTER(&htim1));
    *countennumr  = __HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SetCounter(&htim1, 0);
    __HAL_TIM_SetCounter(&htim2, 0);
}

