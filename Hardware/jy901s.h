#ifndef __JY901S_H
#define __JY901S_H
#include "jy901s_register.h"
#include "usart.h"
#include "string.h"

typedef struct
{
    short acc[3];
    short gyro[3];
    short mag[3];
    short angle[3];  
}jydata;

void JY901S_GetData(jydata* intialdata);

extern char rxbuffer[100];
extern uint8_t rxflag;

#endif


