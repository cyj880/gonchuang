#include "Light.h"

#define LIGHT_PWM_PERIOD 1000u
#define LIGHT_STARTUP_PERCENT 0u  /* 上电默认补光占空比，单位%(2026-11-17: 60→0上电熄灭) */

void Light_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef timebase;
    TIM_OCInitTypeDef oc;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
    /* 先低电平输出，再配置PWM，避免初始化阶段短暂亮灯。 */
    GPIO_ResetBits(GPIOI, GPIO_Pin_6);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOI, &gpio);

    /* APB2定时器时钟168MHz / 168 / 1000 = 1kHz，无定时器中断。 */
    TIM_TimeBaseStructInit(&timebase);
    timebase.TIM_Prescaler = 168u - 1u;
    timebase.TIM_Period = LIGHT_PWM_PERIOD - 1u;
    TIM_TimeBaseInit(TIM8, &timebase);
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_Pulse = LIGHT_STARTUP_PERCENT * (LIGHT_PWM_PERIOD / 100u);
    TIM_OC2Init(TIM8, &oc);
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
    GPIO_PinAFConfig(GPIOI, GPIO_PinSource6, GPIO_AF_TIM8);
    gpio.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOI, &gpio);
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
    TIM_Cmd(TIM8, ENABLE);
}

void Light_SetPercent(uint8_t percent)
{
    if (percent > 100u) percent = 100u;
    /* CCR=0全低，CCR=ARR+1全高；不影响舵机TIM5、蜂鸣器TIM12。 */
    TIM_SetCompare2(TIM8, (uint16_t)(percent * (LIGHT_PWM_PERIOD / 100u)));
}
