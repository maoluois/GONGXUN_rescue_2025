#include "position.h"


void Get_LinearSpeed(short enl, short enr, imudata angle, float* linear_speed)
{
    float wheell_speed, wheelr_speed;
    
    wheell_speed = (float)enl/ConvertParam*WheelCircumference;
    wheelr_speed = (float)enr/ConvertParam*WheelCircumference;
    *linear_speed = (wheelr_speed + wheell_speed) / 2.0f;
    
}

void Get_Position(short enl, short enr, imudata angle, float* x, float* y)
{
    float p_yaw;
    float linear_speed;
    
    p_yaw = angle.yaw*0.01745;
    Get_LinearSpeed(enl, enr, angle, &linear_speed);
    *y += linear_speed*cosf(p_yaw)*sampletime;
    *x -= linear_speed*sinf(p_yaw)*sampletime;
}


void InverseKinematics_differential(float linear_speed, imudata angle, float wheel_distance, float *wheel1_speed, float *wheel2_speed)
{
    *wheel1_speed = linear_speed - angle.gz * wheel_distance / 2.0f;
    *wheel2_speed = linear_speed + angle.gz * wheel_distance / 2.0f;
}


