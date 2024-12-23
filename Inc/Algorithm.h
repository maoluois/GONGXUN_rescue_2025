#ifndef ALGORITHM_H
#define ALGORITHM_H
// 差速轮运动学正逆解
void Kinematics_differential(float wheel1_speed, float wheel2_speed, float wheel_distance);
void InverseKinematics_differential(float linear_speed, float angular_speed, float wheel_distance);

#endif //ALGORITHM_H
