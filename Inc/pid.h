//
// Created by Administrator on 24-9-21.
//

#ifndef PID_H
#define PID_H

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

extern PID_ControllerTypeDef motor1PID_V;
extern PID_ControllerTypeDef motor2PID_V;
extern PID_ControllerTypeDef motor1PID_P;
extern PID_ControllerTypeDef motor2PID_P;
extern PID_ControllerTypeDef distancePID;
extern PID_ControllerTypeDef anglePID;
extern PID_ControllerTypeDef ImuPID;


void PID_Init(PID_ControllerTypeDef *pid,float kp, float ki, float kd, float setpoint);
float PID_Clamp(float value, float min, float max);
float PID_Incremental(PID_ControllerTypeDef *pid, float currentSpeed);
float PID_Velocity(PID_ControllerTypeDef *pid, float currentSpeed);
float PID_Velocity2(PID_ControllerTypeDef *pid, float currentSpeedLeft, float currentSpeedRight, float angle);
float PID_Position(PID_ControllerTypeDef *pid, float currentPos);
float PID_Balance(PID_ControllerTypeDef *pid, float Angle);
float PID_Turn(PID_ControllerTypeDef *pid, float Angle, float Gyro);
float PID_Compute(PID_ControllerTypeDef *pid, float measurement);
#endif //PID_H
