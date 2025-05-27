#include "math.h"
#include "PID.h"



/**
  * @brief  初始化PID结构体
  * @param  结构体指针，pid参数，目标值
  * @retval 无
  */
void PID_Init(PID_ControllerTypeDef *pid,float kp, float ki, float kd, float setpoint) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->setpoint = setpoint;
    pid->lastError = 0.0f;
    pid->lastLastError = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

/**
  * @brief  计算输出值
  * @param  pid结构体，当前测量值
  * @retval 无
  */
float PID_Compute(PID_ControllerTypeDef *pid, float measurement) {
    float error = pid->setpoint - measurement;

    // Define the dead zone range
    float dead_zone = 0.1;

    // Apply dead zone
    if (fabs(error) < dead_zone) {
        error = 0;
    }

    pid->integral += error;
    float derivative = error - pid->lastError;
    pid->lastError = error;
    return pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
}

/**
  * @brief  速度环
  * @param  pid结构体，编码值
  * @retval 无
  */
float PID_Velocity(PID_ControllerTypeDef *pid, float currentSpeed) {
    // 计算PID控制量
    float integral;
    float error = pid->setpoint - currentSpeed;
    integral = pid->integral + pid->Ki * error;
    float proportional = pid->Kp * error;
    
    pid->output = proportional + integral;

    // 更新积分项和记录上一次误差
    pid->integral = integral;

    return pid->output;
}


/**
  * @brief  角速度环
  * @param  pid结构体，编码值
  * @retval 无
  */
float PID_Gyro(PID_ControllerTypeDef *pid, float gyro) 
{

    // 2. 计算角速度误差
    float bias = gyro - pid->setpoint; // 假设目标角速度为0（稳定状态）
    // 3. 计算PID输出
    float proportional = pid->Kp * bias;
    float integral = pid->integral + pid->Ki * bias;
    integral>3500 ? integral = 3500  : 0;
    integral<-3500? integral = -3500 : 0;
    pid->output = proportional + integral;
    // 4. 更新上一次的角度误差
    pid->integral = integral;
    
    return pid->output;
}

float PID_Turn(PID_ControllerTypeDef *pid, float yaw) 
{
    // 2. 计算角速度误差
    float bias = yaw - pid->setpoint; // 假设目标角速度为0（稳定状态）
    // 3. 计算PID输出
    float proportional = pid->Kp * bias;
    float integral = pid->integral + pid->Ki * bias;
    float derivative = bias - pid->lastError;
    pid->output = proportional + integral + derivative;
    
    
    // 4. 更新上一次的角度误差
    pid->integral = integral;
    pid->lastError = bias;
    
    return pid->output;
}

float PID_Velocity2(PID_ControllerTypeDef *pid, float currentSpeedLeft, float currentSpeedRight, float angle) {
    float error = pid->setpoint - (currentSpeedLeft + currentSpeedRight) / 2;
    // 计算PID控制量
    float proportional = pid->Kp * error;
    float integral = pid->integral + pid->Ki * error;
    float derivative = pid->Kd * (error - pid->lastError);

    pid->output = proportional + integral + derivative;

    // 限制PID输出在合理范围内
    pid->output = PID_Clamp(pid->output, -1000, 1000);

    // 更新积分项和记录上一次误差
    if (angle > 25 || angle < -25) {
        pid->integral = 0;
    }
    else pid->integral = integral;

    pid->lastError = error;

    return pid->output;
}

// 位置环
float PID_Position(PID_ControllerTypeDef *pid, float currentPosition) {
    // 计算位置误差
    float positionError = pid->setpoint - currentPosition;

    // 计算PID控制量
    float proportional = pid->Kp * positionError;
    float integral = pid->integral + pid->Ki * positionError;
    float derivative = pid->Kd * (positionError - pid->lastError);

    pid->output = proportional + integral + derivative;

    // 限制PID输出在合理范围内
    pid->output = PID_Clamp(pid->output, -1000, 1000);

    // 更新积分项和记录上一次误差
    pid->integral = integral;
    pid->lastError = positionError;

    return pid->output;
}

//// 直立环
//float PID_Balance(PID_ControllerTypeDef *pid, float Angle)
//{
//    float Angle_bias, Gyro_bias;
//    Angle_bias = Middle_angle - Angle;                    			//求出平衡的角度中值 和机械相关
//    float Gyro = Angle - pid->lastError;                          	//求出角速度
//    Gyro_bias = 0 - Gyro;
//    pid->output= -pid->Kp * Angle_bias - Gyro_bias * pid->Kd ;      //计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数
//    pid->lastError = Angle;                                			//记录角度

//    return pid->output;
//}

/**
  * @brief  输出限幅
  * @param  输出值，上下限
  * @retval 无
  */
float PID_Clamp(float value, float min, float max) {
    if (value > max) {
        return max;
    }
    else if (value < min) {
        return min;
    }
    else {
        return value;
    }
}
