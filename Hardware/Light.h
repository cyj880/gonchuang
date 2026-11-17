#ifndef LIGHT_H
#define LIGHT_H

#include "stm32f4xx.h"

/* PI6 = TIM8_CH2(AF3)，1kHz高电平有效PWM；上电默认60%占空比。 */
void Light_Init(void);
void Light_SetPercent(uint8_t percent); /* 0~100%，超范围限制到100%。 */

#endif
