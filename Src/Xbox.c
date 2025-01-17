#include "Xbox.h"
#include "Algorithm.h"


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

void Get_Data_Ogpi(uint8_t *Rx_data) {
  uint8_t x_Start_Num = 0, x_End_Num = 0;
  uint8_t y_Start_Num = 0, y_End_Num = 0;
  uint8_t x_Point_Num = 0, y_Point_Num = 0;
  uint8_t flag = 0;          // 开始寻找标志位
  uint8_t data_class = 0;    // 数据类别

  // 查找等号、小数点和感叹号的位置
  for (uint8_t i = 0; i < 200; i++) {
    if (Rx_data[i] == '\n') {
      flag = 1;  // 找到换行符，标志接收开始
    }
    if (Rx_data[i] == 'c' && flag == 1) {
      data_class = Rx_data[i + 1] - '0';  // 获取数据类别
    }
    if (Rx_data[i] == 'x' && flag == 1) {
      x_Start_Num = i + 1;
    }
    if (Rx_data[i] == 'y' && flag == 1) {
      x_End_Num = i - 1;
      y_Start_Num = i + 1;
    }
    if (Rx_data[i] == '.' && flag == 1) {
      if (x_Start_Num > 0 && x_Point_Num == 0) {
        x_Point_Num = i;
      } else if (y_Start_Num > 0 && y_Point_Num == 0) {
        y_Point_Num = i;
      }
    }
    if (Rx_data[i] == '!' && flag == 1) {
      y_End_Num = i - 1;
      break;
    }
  }

  // 计算整数和小数数据的长度
  uint8_t x_Integer_len = x_Point_Num - x_Start_Num;
  uint8_t x_Decimal_len = x_End_Num - x_Point_Num;
  uint8_t y_Integer_len = y_Point_Num - y_Start_Num;
  uint8_t y_Decimal_len = y_End_Num - y_Point_Num;

  // 计算返回值
  float x_value = data_Integer_calculate(x_Integer_len, x_Start_Num, Rx_data) +
                  data_Decimal_calculate(x_Decimal_len, x_Point_Num, Rx_data);
  float y_value = data_Integer_calculate(y_Integer_len, y_Start_Num, Rx_data) +
                  data_Decimal_calculate(y_Decimal_len, y_Point_Num, Rx_data);

  // 根据数据类别处理数据
  switch (data_class) {
  case 0:
    blue_ball.x = x_value;
    blue_ball.y = y_value;
    break;
  case 1:
    red_ball.x = x_value;
    red_ball.y = y_value;
    break;
  case 2:
    black_ball.x = x_value;
    black_ball.y = y_value;
    break;
  case 3:
    yellow_ball.x = x_value;
    yellow_ball.y = y_value;
    break;
  case 4:
    blue_base.x = x_value;
    blue_base.y = y_value;
    break;
  case 5:
    blue_aim.x = x_value;
    blue_aim.y = y_value;
    break;
  case 6:
    red_base.x = x_value;
    red_base.y = y_value;
    break;
  case 7:
    red_aim.x = x_value;
    red_aim.y = y_value;
    break;
  default:
    // 处理未知类别的数据
    break;
  }

  class = data_class;
}

// 初始化香橙派数据接收对象
void InitializeOgpi(volatile class_Ogpi *obj, int category)
{
  obj -> x = 0;
  obj -> y = Camera_centerY;
  obj -> class = category;
}