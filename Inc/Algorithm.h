#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <stdint.h>

// 差速轮运动学正逆解
void Kinematics_differential(float wheel1_speed, float wheel2_speed, float wheel_distance, float *linear_speed, float *angular_speed);
void InverseKinematics_differential(float linear_speed, float angular_speed, float wheel_distance, float *wheel1_speed, float *wheel2_speed);
void calculate_target_speeds(uint16_t x, uint16_t y, float* v_f, float* w);
#endif //ALGORITHM_H
