//
// Created by Administrator on 24-12-29.
//

#ifndef XBOX_H
#define XBOX_H
#include <stdint.h>
void Get_Data_Xbox(uint8_t *Rx_data);
void Get_Data_Ogpi(uint8_t *Rx_data);

uint16_t XboxData[4] = {9, 9, 0, 0};
extern uint16_t XboxData[4];

uint8_t class;
extern uint8_t class;
typedef struct
{
  uint8_t class;
  float x;
  float y;
  float area;
} class_Ogpi;

extern class_Ogpi red_ball;
extern class_Ogpi blue_ball;
extern class_Ogpi yellow_ball;
extern class_Ogpi black_ball;
extern class_Ogpi blue_aim;
extern class_Ogpi red_aim;
extern class_Ogpi blue_base;
extern class_Ogpi red_base;

#endif //XBOX_H
