#ifndef __LED_H__
#define __LED_H__

#include "stm32f4xx.h"

void LED_Init(void);
void LED_On(uint8_t num);   //点亮第num颗LED（num = 1~8，PG1~PG8）
void LED_Off(uint8_t num);  //熄灭第num颗LED（num = 1~8）
void LED_Green_On(void);    //点亮绿色指示灯（PF14）
void LED_Green_Off(void);   //熄灭绿色指示灯（PF14）
void LED_Red_On(void);      //点亮红色指示灯（PE11）
void LED_Red_Off(void);     //熄灭红色指示灯（PE11）
void LED_Red_Toggle(void);  //翻转红色指示灯（PE11）

#endif
