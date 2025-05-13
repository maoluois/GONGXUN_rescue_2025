//
// Created by Administrator on 24-12-29.
//

#ifndef XBOX_H
#define XBOX_H
#include <stdint.h>
#include <main.h>
void Get_Data_Xbox(uint8_t *Rx_data);
void Get_Data_Ogpi(uint8_t *Rx_data);
extern uint16_t XboxData[4];
extern volatile uint8_t class;

typedef struct
{
  uint8_t class;
  float x;
  float y;
  float area;
} class_Ogpi;

extern volatile class_Ogpi red_ball;
extern volatile class_Ogpi blue_ball;
extern volatile class_Ogpi yellow_ball;
extern volatile class_Ogpi black_ball;
extern volatile class_Ogpi blue_aim;
extern volatile class_Ogpi red_aim;
extern volatile class_Ogpi blue_base;
extern volatile class_Ogpi red_base;

void InitializeOgpi(volatile class_Ogpi *obj, int category);

#endif //XBOX_H
