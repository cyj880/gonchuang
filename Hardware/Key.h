#ifndef __KEY_H__
#define __KEY_H__

#include "stm32f4xx.h"

#define KEY1_GPIO_Port	GPIOB			//A板用户按键接在PB2（手册1.7节）
#define KEY1_Pin		GPIO_Pin_2

typedef struct
{
	BitAction Previous;					//上一次扫描到的电平
	BitAction Current;					//本次扫描到的电平

	GPIO_TypeDef *GPIO_Port;			//按键所在的GPIO端口
	uint16_t GPIO_Pin;					//按键的引脚号

	void (*ClickedCallback)(void);		//按键松开时触发的回调函数

} KeyHandle_TypeDef;

void Key_Init(KeyHandle_TypeDef *Handle);
void Key_Scan(KeyHandle_TypeDef *Handle);

#endif
