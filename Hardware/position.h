#ifndef __POSITION_H
#define __POSITION_H

#include "jy901s.h"
#include "filter.h"
#include "math.h"

#define ConvertParam (13.0f*30.0f*4.0f/100)  // 从脉冲转到转速的转换参数 转速 = 脉冲数 / (线数 * 减速比 * 4) (转/10毫秒)
#define WheelDistance 23.156f // 单位：cm
#define WheelRadius 3.25f // 单位：cm
#define WheelCircumference 20.42f // 单位：cm

void Get_LinearSpeed(short enl, short enr, imudata angle, float* linear_speed);
void Get_Position(short enl, short enr, imudata angle, float* x, float* y);

#endif

