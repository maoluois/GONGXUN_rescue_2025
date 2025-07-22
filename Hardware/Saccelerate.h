#ifndef __SACCELERATE_H
#define __SACCELERATE_H

#include "math.h"
#include <stm32h7xx.h>
#include "Rescue_Car.h"

//s形加速变量封装结构体
typedef struct sacc_typedef
{
    float j_max;
    float a_max;
    float a_supplement;
    float a0;
    float a;
    float a_pre;
    float target;
    float current;
    float delta;
    float threshold;
    float vnew;
    float mininum;
    float t0, t1, t2, t3, t;
    int8_t time_flag;
    float tick;
}sacc_typedef; 


float fast_sqrt(float a);
void Sacc_Init(sacc_typedef* sacc, float j_max, float a_max, float a_supplement, float minimun);
void time_calculate(sacc_typedef* sacc);
void acc_calculate(sacc_typedef* sacc);
int sign(float a);
void Angle_Constrain(sacc_typedef* sacc);


#endif
