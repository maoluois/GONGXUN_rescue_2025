#ifndef __JY901S_H
#define __JY901S_H
#include "jy901s_register.h"
#include "usart.h"
#include "string.h"

#define sampletime 0.01

typedef struct
{
    short acc[3];
    short gyro[3];
    short angle[3];  
}jydata;

typedef struct 
{
    double ax, ay, az;
    double gx, gy, gz;
    double pitch, roll, yaw;
}imudata;


void JY901S_GetData(jydata* initdata);
void JY901S_DataConverse(imudata* data);

extern char imu_buffer[100];
extern uint8_t imurxflag;
extern uint8_t imuerrflag;
extern char jytempdata[44];
extern uint8_t imurxsize;
#endif


