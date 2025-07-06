#include "Xbox.h"

uint16_t XboxData[4];
/**
  * @brief  获取XB发送的数据
  * @param  接收到XB数据的数组指针
  * @retval 无
  */
void Get_Data_Xbox(uint8_t *Rx_data) {
  uint8_t up_Start_Num = 0, up_End_Num = 0;
  uint8_t down_Start_Num = 0, down_End_Num = 0;
  uint8_t x_Start_Num = 0, x_End_Num = 0;
  uint8_t y_Start_Num = 0, y_End_Num = 0;
  uint8_t flag = 0;          // 开始寻找标志位

  // 查找等号、小数点和感叹号的位置
  for (uint8_t i = 0; i < 200; i++) {
    if (Rx_data[i] == '\n') {
      flag = 1;  // 找到换行符，标志接收开始
    }
    if (Rx_data[i] == 'u' && flag == 1) {
      up_Start_Num = i + 1;
    }
    if (Rx_data[i] == 'd' && flag == 1) {
      up_End_Num = i - 1;
      down_Start_Num = i + 1;
    }
    if (Rx_data[i] == 'x' && flag == 1) {
      down_End_Num = i - 1;
      x_Start_Num = i + 1;
    }
    if (Rx_data[i] == 'y' && flag == 1) {
      x_End_Num = i - 1;
      y_Start_Num = i + 1;
    }
    if (Rx_data[i] == '!' && flag == 1) {
      y_End_Num = i - 1;
      break;
    }
  }

  // 计算整数数据的长度
  uint8_t up_Integer_len = up_End_Num - up_Start_Num + 1;
  uint8_t down_Integer_len = down_End_Num - down_Start_Num + 1;
  uint8_t x_Integer_len = x_End_Num - x_Start_Num + 1;
  uint8_t y_Integer_len = y_End_Num - y_Start_Num + 1;

  // 计算返回值
  XboxData[0] = data_Integer_calculate(up_Integer_len, up_Start_Num, Rx_data);
  XboxData[1] = data_Integer_calculate(down_Integer_len, down_Start_Num, Rx_data);
  XboxData[2] = data_Integer_calculate(x_Integer_len, x_Start_Num, Rx_data);
  XboxData[3] = data_Integer_calculate(y_Integer_len, y_Start_Num, Rx_data);
}


/**
  * @brief  将值映射到目标范围的函数
  * @param  要映射的值，当前值的上下限，映射后的上下限
  * @retval 无
  */
float map(float value, float in_min, float in_max, float out_min, float out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/**
  * @brief  将XB接收的速度和角速度经行映射
  * @param  XB接两个收值，目标速度、角度
  * @retval 无
  */
void calculate_target_speeds(uint16_t x, uint16_t y, float* w, float* v) {
    // 将速度和角度映射到目标范围
    *v = -(int8_t)map(y, 0, 65535, XB_V_MIN, XB_V_MAX);
    *w = -(int16_t)map(x, 0, 65535, XB_W_MIN, XB_W_MAX);
}




