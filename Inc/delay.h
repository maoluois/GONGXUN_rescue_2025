//
// Created by Administrator on 2024/4/10.
//
#include "main.h"
#ifndef DELAY_H
#define DELAY_H

void delay_init(uint16_t sysclk);           /* 初始化延迟函数 */
void delay_ms(uint16_t nms);                /* 延时nms */
void delay_us(uint32_t nus);                /* 延时nus */

#if (!SYS_SUPPORT_OS)                       /* 没有使用Systick中断 */
void HAL_Delay(uint32_t Delay);         /* HAL库的延时函数，SDIO等需要用到 */
#endif

#endif

