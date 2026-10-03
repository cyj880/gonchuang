#include "Buzzer.h"
#include "Delay.h"

/**
  * 板载蜂鸣器驱动（大疆 RoboMaster A 板）
  *
  * 硬件：PH6 = TIM12_CH1（AF9 复用），经 NPN 三极管（Q4）驱动贴片蜂鸣器
  *       高电平 → 三极管导通 → 蜂鸣器响
  * 时基：TIM12 挂 APB1，定时器时钟 84MHz（APB1 42MHz × 2）
  *       PSC = 0，ARR = 31111-1 → PWM 频率 = 84MHz / 31111 ≈ 2700Hz
  *       （用户手册 1.10 节：蜂鸣器额定驱动频率 2700Hz）
  * 占空比：约 50%（CCR = 15555）
  */

#define BUZZER_ARR      (31111 - 1)     /* 决定 PWM 频率 ≈ 2700Hz */
#define BUZZER_DUTY     (31111 / 33)    /* 约 3% 占空比，轻音量；想更响改回 (31111 / 2) */

/**
  * 函    数：蜂鸣器初始化（PWM 输出，上电默认静音）
  * 参    数：无
  * 返 回 值：无
  */
void Buzzer_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    /*开启时钟*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);        //TIM12挂APB1
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);        //GPIOH挂AHB1

    /*GPIO初始化：PH6 复用推挽，映射到 TIM12_CH1（AF9）*/
    GPIO_PinAFConfig(GPIOH, GPIO_PinSource6, GPIO_AF_TIM12);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;                    //PH6
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                 //F4复用模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;               //下拉，确保复位期间蜂鸣器不响
    GPIO_Init(GPIOH, &GPIO_InitStructure);

    /*时基单元：84MHz / 31111 ≈ 2700Hz*/
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = BUZZER_ARR;           //ARR，决定PWM频率
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;                 //PSC，不分频
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM12, &TIM_TimeBaseInitStructure);

    /*输出比较通道1：PWM1模式，高电平有效，初始占空比0（静音）*/
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;                           //初始静音
    TIM_OC1Init(TIM12, &TIM_OCInitStructure);                    //TIM12通道1

    TIM_Cmd(TIM12, ENABLE);
	
	Buzzer_On();
    Delay_ms(150);
    Buzzer_Off();
}

/**
  * 函    数：蜂鸣器响（开启 PWM 输出）
  * 参    数：无
  * 返 回 值：无
  */
void Buzzer_On(void)
{
    TIM_SetCompare1(TIM12, BUZZER_DUTY);
}

/**
  * 函    数：蜂鸣器停（关闭 PWM 输出）
  * 参    数：无
  * 返 回 值：无
  */
void Buzzer_Off(void)
{
    TIM_SetCompare1(TIM12, 0);
}

