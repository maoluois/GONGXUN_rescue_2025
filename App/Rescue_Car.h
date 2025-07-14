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
#include "string.h"
#include "stdlib.h"
#include "position.h"
#include "Saccelerate.h"

#define XB_V_MAX 70    // 遥控速度范围 // 极限值为+-100
#define XB_V_MIN -70
#define XB_W_MAX 10  // 极限值为+-10
#define XB_W_MIN -10
#define PWM_MAX 4900
#define PWM_MIN -4900

void Rescue_Car_Init(void);
extern PID_ControllerTypeDef velocity_pid;
extern char debugrxdata[30];
#endif

