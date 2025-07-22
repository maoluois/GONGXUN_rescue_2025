#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <stdint.h>
#include "main.h"
#include "Rescue_Car.h"

// 差速轮运动学正逆解
void InverseKinematics_differential(float linear_speed, float angular_speed, float wheel_distance, float *wheel1_speed, float *wheel2_speed);
uint16_t data_Integer_calculate(uint8_t data_Integer_len, uint8_t data_Start_Num, uint8_t *DataBuff);
double data_Decimal_calculate(uint8_t data_Decimal_len, uint8_t data_Point_Num, uint8_t *Data);

#endif //ALGORITHM_H
