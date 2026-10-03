#include "stm32f4xx.h"

/* 1ms 定时中断初始化 */
void TIM2_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef         NVIC_InitStructure;

    // 1. 开启 TIM2 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // 2. 配置时基参数
    TIM_TimeBaseStructure.TIM_Period        = 999;   // 自动重装载值 (1ms)
    TIM_TimeBaseStructure.TIM_Prescaler     = 83;    // 预分频值 (84MHz/84=1MHz)
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  // 死区时钟不分频
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // 3. 使能更新中断
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    // 4. 配置 NVIC
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;   // 响应优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 5. 使能 TIM2
    TIM_Cmd(TIM2, ENABLE);
}

/* TIM2 中断服务函数 */
/*
void TIM2_IRQHandler(void)
{
    // 检查是否发生更新中断
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  // 清除中断标志位

        // ===== 在此添加用户代码，例如翻转 LED =====
        // GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
    }
}
*/
