#ifndef XBOX_H
#define XBOX_H

#include <stdint.h>
#include <main.h>
#include "Rescue_Car.h"


#define XBOX_BUFFER_SIZE  88 //XBOX 数组容量大小

void Get_Data_Xbox(uint8_t *Rx_data);

void calculate_target_speeds(uint16_t x, uint16_t y, float* v_f, float* w);

void datadeal(void);
    
extern uint16_t XboxData[4];
extern volatile uint8_t class;

    
typedef struct
{
  uint8_t class;
  float x;
  float y;
  float area;
} class_Ogpi;

#endif //XBOX_H
