#ifndef PID_H
#define PID_H

#include "math.h"
#include <stm32h7xx.h>

//直立环的机械中值
#define Middle_angle 0

typedef struct{
    float Kp;
    float Ki;
    float Kd;
    float setpoint;
    float lastError;
    float lastLastError;
    float integral;
    float output;

}PID_ControllerTypeDef;


extern uint8_t positionflag;
extern float xset, yset;

void PID_Init(PID_ControllerTypeDef *pid,float kp, float ki, float kd, float setpoint);
float PID_Clamp(float value, float min, float max);
float PID_Incremental(PID_ControllerTypeDef *pid, float currentSpeed);
float PID_Velocity(PID_ControllerTypeDef *pid, float currentSpeed);
float PID_Velocity2(PID_ControllerTypeDef *pid, float currentSpeedLeft, float currentSpeedRight, float angle);
float PID_Position(PID_ControllerTypeDef *pid, float x, float y, float xset, float yset);
float PID_Balance(PID_ControllerTypeDef *pid, float Angle);
float PID_Turn(PID_ControllerTypeDef *pid, float yaw);
float PID_Gyro(PID_ControllerTypeDef *pid, float gyro);
float PID_Compute(PID_ControllerTypeDef *pid, float measurement);
#endif //PID_H
