#ifndef __RESCUE_CAR_H
#define __RESCUE_CAR_H

#include "FreeRTOS.h"
#include "task.h"
#include "encoder.h"
#include "jy901s.h"
#include "Xbox.h"
#include "Algorithm.h"
#include "pid.h"
#include "control.h"

void Rescue_Car_Init(void);

#define XB_V_MAX 50    // 遥控速度范围 // 极限值为+-127
#define XB_V_MIN -50
#define XB_W_MAX -3   // 极限值为+-10
#define XB_W_MIN 3
#define PWM_MAX 4900
#define PWM_MIN -4900

#endif

