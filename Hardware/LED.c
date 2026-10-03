#include "stm32f4xx.h"                  // Device header

/*
 * A型开发板板载LED共10颗，均低电平点亮（灌电流接VCC）：
 *   1. 用户自定义LED×8：PG1~PG8，绿色，单颗约4mA（LED_On/LED_Off 按 1~8 控制）
 *   2. 用户自定义LED×2：PF14=绿色，PE11=红色，约4mA（LED_Green/LED_Red 控制）
 * 手册来源：《RoboMaster开发版用户手册》1.5 / 1.6 节
 */
#define LED_PINS    (GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | \
                     GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8)

//LED初始化函数（PG1~PG8推挽输出，上电默认全灭）
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);   //PF14绿色LED
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);   //PE11红色LED

	//PG1~PG8推挽输出
	GPIO_InitStructure.GPIO_Pin = LED_PINS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	GPIO_SetBits(GPIOG, LED_PINS);      //高电平=熄灭

	//PF14（绿）、PE11（红）推挽输出，默认熄灭
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_SetBits(GPIOF, GPIO_Pin_14);   //高电平=熄灭
	GPIO_SetBits(GPIOE, GPIO_Pin_11);
}

//点亮第num颗LED（num = 1~8，低电平点亮）
void LED_On(uint8_t num)
{
	GPIO_ResetBits(GPIOG, (uint16_t)(1 << num));
}

//熄灭第num颗LED（num = 1~8）
void LED_Off(uint8_t num)
{
	GPIO_SetBits(GPIOG, (uint16_t)(1 << num));
}

//点亮绿色指示灯（PF14）
void LED_Green_On(void)
{
	GPIO_ResetBits(GPIOF, GPIO_Pin_14);
}

//熄灭绿色指示灯（PF14）
void LED_Green_Off(void)
{
	GPIO_SetBits(GPIOF, GPIO_Pin_14);
}

//点亮红色指示灯（PE11）
void LED_Red_On(void)
{
	GPIO_ResetBits(GPIOE, GPIO_Pin_11);
}

//熄灭红色指示灯（PE11）
void LED_Red_Off(void)
{
	GPIO_SetBits(GPIOE, GPIO_Pin_11);
}

//翻转红色指示灯（PE11），用于按键控制亮灭
void LED_Red_Toggle(void)
{
	GPIO_ToggleBits(GPIOE, GPIO_Pin_11);
}
