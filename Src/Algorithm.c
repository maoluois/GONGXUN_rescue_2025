#include "Algorithm.h"
#include "main.h"
#include <math.h>

void Kinematics_differential(float wheel1_speed, float wheel2_speed, float wheel_distance, float *linear_speed, float *angular_speed)
{
    *linear_speed = (wheel1_speed + wheel2_speed) / 2.0f;
    *angular_speed = (wheel2_speed - wheel1_speed) / wheel_distance;
}

void InverseKinematics_differential(float linear_speed, float angular_speed, float wheel_distance, float *wheel1_speed, float *wheel2_speed)
{
    *wheel1_speed = linear_speed - angular_speed * wheel_distance / 2.0f;
    *wheel2_speed = linear_speed + angular_speed * wheel_distance / 2.0f;
}

// 将值映射到目标范围的函数
float map(float value, float in_min, float in_max, float out_min, float out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// 解算函数
void calculate_target_speeds(uint16_t x, uint16_t y, float* v_f, float* w) {

    // 计算方向角度并归一化到角速度
    float angle = atan2(x, y); // 角度范围为[-π, π]

    // 将速度和角度映射到目标范围
    *v_f = (int8_t)map(y, 0, 65535, V_F_MIN, V_F_MAX); // 假设归一化输入速度范围为[0, 1]
    *w = (int8_t)map(angle, -M_PI, M_PI, W_MIN, W_MAX);
}