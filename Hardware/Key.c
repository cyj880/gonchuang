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

//按键初始化函数（配置引脚为浮空输入，并复位扫描状态；时钟按所在端口自动使能）
void Key_Init(KeyHandle_TypeDef *Handle)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*按句柄端口使能对应GPIO时钟(2026-11-14支持多端口: PB2板载键/PI7外接启动键)*/
	switch ((uint32_t)Handle->GPIO_Port)
	{
	case GPIOA_BASE: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); break;
	case GPIOB_BASE: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); break;
	case GPIOC_BASE: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); break;
	case GPIOE_BASE: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); break;
	case GPIOF_BASE: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); break;
	case GPIOI_BASE: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE); break;
	default: break;
	}

	GPIO_InitStructure.GPIO_Pin = Handle->GPIO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;   /*外接模块自带5V上拉, 板载PB2按下为高*/
	GPIO_Init(Handle->GPIO_Port, &GPIO_InitStructure);

	Handle->Previous = Handle->Current = Handle->Candidate =
		(BitAction)GPIO_ReadInputDataBit(Handle->GPIO_Port, Handle->GPIO_Pin);
	Handle->StableScans = 0;
}

//按键扫描函数（需周期性调用，检测到按键松开时触发回调）
void Key_Scan(KeyHandle_TypeDef *Handle)
{
	BitAction sample = (BitAction)GPIO_ReadInputDataBit(Handle->GPIO_Port, Handle->GPIO_Pin);
	if (sample != Handle->Candidate)
	{
		Handle->Candidate = sample;
		Handle->StableScans = 1;
		return;
	}
	if (Handle->StableScans < 3u) Handle->StableScans++;
	if (Handle->StableScans < 3u) return;
	Handle->Current = sample;

	//A板按键按下为高：Previous=按下(SET)、Current=松开(RESET) → 松开沿
	if(Handle->Previous == Bit_SET && Handle->Current == Bit_RESET)
	{
		Handle->ClickedCallback();
	}

	Handle->Previous = Handle->Current;
}
