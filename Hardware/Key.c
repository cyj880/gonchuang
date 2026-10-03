#include "stm32f4xx.h"                  // Device header
#include "Key.h"

/**
  * A板用户按键驱动（源自 STM32F407 例程 Key.c 移植，适配 PB2）
  *
  * 硬件差异：407 例程按键按下为低电平（上拉输入）；
  *           A板白色用户按键直接连 PB2，按下为高电平（手册1.7节），
  *           故配置为下拉输入，松开=低、按下=高。
  * 扫描逻辑：松开沿（高→低）触发 ClickedCallback，避免长按重复触发。
  */

//按键初始化函数（配置引脚为下拉输入，并复位扫描状态）
void Key_Init(KeyHandle_TypeDef *Handle)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	//PB2浮空输入，按键按下为高电平
	GPIO_InitStructure.GPIO_Pin = KEY1_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(KEY1_GPIO_Port, &GPIO_InitStructure);

	Handle->Previous = Bit_RESET;
	Handle->Current = Bit_RESET;
}

//按键扫描函数（需周期性调用，检测到按键松开时触发回调）
void Key_Scan(KeyHandle_TypeDef *Handle)
{
	Handle->Current = (BitAction)GPIO_ReadInputDataBit(Handle->GPIO_Port, Handle->GPIO_Pin);

	//A板按键按下为高：Previous=按下(SET)、Current=松开(RESET) → 松开沿
	if(Handle->Previous == Bit_SET && Handle->Current == Bit_RESET)
	{
		Handle->ClickedCallback();
	}

	Handle->Previous = Handle->Current;
}
