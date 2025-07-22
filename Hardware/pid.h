#ifndef PID_H
#define PID_H

#include "math.h"
#include <stm32h7xx.h>
#include "Rescue_Car.h"

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


void PID_Init(PID_ControllerTypeDef *pid,float kp, float ki, float kd, float setpoint);
float PID_Clamp(float value, float min, float max);
float PID_Velocity(PID_ControllerTypeDef *pid, float currentSpeed);
//float PID_Position(PID_ControllerTypeDef *pid, float mileage);
float PID_Position(PID_ControllerTypeDef *pid, float tx, float ty, float x, float y);
//float PID_Balance(PID_ControllerTypeDef *pid, float Angle);
float PID_Turn(PID_ControllerTypeDef *pid, float yaw);
float PID_Gyro(PID_ControllerTypeDef *pid, float gyro);
float PID_Compute(PID_ControllerTypeDef *pid, float measurement);

extern  float current;
extern PID_ControllerTypeDef velocity_pid;
extern PID_ControllerTypeDef gyro_pid;
extern PID_ControllerTypeDef turn_pid;
extern PID_ControllerTypeDef position_pid;
    
#endif //PID_H
