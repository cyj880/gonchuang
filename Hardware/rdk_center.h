#ifndef __RDK_CENTER_H
#define __RDK_CENTER_H

#include "stm32f4xx.h"

/* RDK圆心链路: USART3, PD8=TX / PD9=RX, 115200-8-N-1. */
void RDK_Center_Init(uint32_t baud);
void RDK_Center_IRQHandler(void);
void RDK_Center_Enable(uint8_t enable);
void RDK_Center_Clear(void);
void RDK_Center_SendLocate(uint8_t c1, uint8_t c2, uint8_t c3);
void RDK_Center_SendIdle(void);
void RDK_Center_Poll(void);
uint8_t RDK_CenterValid(void);
int16_t RDK_CenterX(void);
int16_t RDK_CenterY(void);
uint32_t RDK_CenterAgeMs(void);
uint32_t RDK_CenterSequence(void);

#endif
