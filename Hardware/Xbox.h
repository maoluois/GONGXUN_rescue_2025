//
// Created by Administrator on 24-12-29.
//

#ifndef XBOX_H
#define XBOX_H
#include <stdint.h>
#include <main.h>
void Get_Data_Xbox(uint8_t *Rx_data);

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
