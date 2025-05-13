#ifndef __MYI2C_H
#define __MYI2C_H

#include "gpio.h"
#include "Delay.h"

void MyI2C_Start(void);
void MyI2C_Stop(void);
void MyI2C_SendByte(uint8_t Byte);
uint16_t MyI2C_ReceiveByte(void);
void MyI2C_SendAck(uint8_t AckBit);
uint8_t MyI2C_ReceiveAck(void);
void I2C_WriteReg(uint8_t address, uint8_t reg, uint8_t data);
uint16_t I2C_ReadReg(uint8_t address, uint8_t reg);

#endif
