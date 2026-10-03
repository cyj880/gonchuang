#include "stm32f4xx.h"
#include "Servo.h"

/**
  * 180°舵机驱动（F427 版，源自江协 6-4 例程移植）
  *
  * PWM 输出：PA1 = TIM5_CH2（AF2 复用）
  * 时基：TIM5 时钟 84MHz（APB1 42MHz × 2），PSC = 84-1 → 1MHz 计数
  *       ARR = 20000-1 → 20ms 周期（50Hz，标准舵机频率）
  * 脉宽：CCR 500~2500 → 0.5ms~2.5ms → 0°~180°（与例程一致）
  *
  * 选型说明：刻意用 TIM5 而不用 TIM2——PA0~PA3/PA15/PB10/PB11 上的
  * TIM2 复用让给 System/Timer.c 的 1ms 定时中断（TIM2_Init），
  * 两者互不冲突；TIM5 是通用定时器，无需高级定时器的 MOE 主输出。
  */

/**
  * 函    数：舵机初始化
  * 参    数：无
  * 返 回 值：无
  */
void Servo_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    /*开启时钟*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);         //TIM5挂APB1
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);        //GPIOA挂AHB1

    /*GPIO初始化：PA1 复用推挽，映射到 TIM5_CH2*/
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;                    //PA1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                 //F4复用模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*时基单元：84MHz/84 = 1MHz，计数20000个 = 20ms（50Hz）*/
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;            //ARR，决定PWM频率
    TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1;            //PSC，84MHz/84 = 1MHz
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);

    /*输出比较通道2：PWM1模式，高电平有效*/
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1500;                        //上电默认90°（1.5ms）
    TIM_OC2Init(TIM5, &TIM_OCInitStructure);                     //TIM5通道2

    /*TIM使能（TIM5通用定时器，无需TIM1那种MOE主输出）*/
    TIM_Cmd(TIM5, ENABLE);
}

/**
  * 函    数：舵机设置角度
  * 参    数：Angle 要设置的舵机角度，范围：0~180
  * 返 回 值：无
  * 说    明：角度线性映射到脉宽 0.5ms~2.5ms（CCR 500~2500）
  */
void Servo_SetAngle(float Angle)
{
    if (Angle < 0.0f)   Angle = 0.0f;                            //限幅保护
    if (Angle > 180.0f) Angle = 180.0f;

    TIM_SetCompare2(TIM5, (uint16_t)(Angle / 180.0f * 2000.0f + 500.0f));
}

/* ==================== 270° 舵机（TIM5_CH1, PA0） ====================
 * 与180°舵机共用 TIM5 时基（84MHz/84=1MHz, ARR=20000-1, 50Hz），
 * CH1 输出 0.5~2.5ms 脉宽线性映射 0~270°。
 * 信号脚: PA0 = 蓝色8路PWM排针（丝印TIM2_CH1(PA0)，同一引脚AF2=TIM5_CH1）
 * 供电  : 大扭矩舵机建议从蓝色排针"+/G"取电（5V@5A），注意拨码须全OFF
 *         （该电源是拨码可调的4.99~12.24V，拨错挡位会烧5V舵机！）；
 *         或确认电流不超限时从用户自定义接口5V(VCC_5V_U)取电。
 * 须在 Servo_Init() 之后调用 Servo270_Init()（共用其时基配置）。
 * 若你的270°舵机脉宽规格不是500~2500us，改下面两个宏即可。 */

#define SERVO270_PULSE_MIN   500.0f    /* 0°   对应脉宽 us */
#define SERVO270_PULSE_MAX   2500.0f   /* 270° 对应脉宽 us */
#define SERVO270_RANGE       270.0f

/**
  * 函    数：270°舵机初始化
  * 参    数：无
  * 返 回 值：无
  * 说    明：PA0 = TIM5_CH1（AF2），复用 Servo_Init() 配好的 TIM5 时基
  */
void Servo270_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /*GPIO: PA0 复用推挽，映射到 TIM5_CH1*/
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*输出比较通道1：PWM1模式，上电默认15°（三工位转盘的盘1工位，脉宽≈611us）*/
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = (uint16_t)(SERVO270_PULSE_MIN +
                                          (SERVO270_PULSE_MAX - SERVO270_PULSE_MIN) *
                                          (15.0f / SERVO270_RANGE));
    TIM_OC1Init(TIM5, &TIM_OCInitStructure);                     //TIM5通道1
}

/**
  * 函    数：270°舵机设置角度
  * 参    数：Angle 要设置的舵机角度，范围：0~270
  * 返 回 值：无
  * 说    明：角度线性映射到脉宽 SERVO270_PULSE_MIN~MAX（CCR 500~2500）
  */
void Servo270_SetAngle(float Angle)
{
    if (Angle < 0.0f)         Angle = 0.0f;                      //限幅保护
    if (Angle > SERVO270_RANGE) Angle = SERVO270_RANGE;

    TIM_SetCompare1(TIM5, (uint16_t)(SERVO270_PULSE_MIN +
                    (SERVO270_PULSE_MAX - SERVO270_PULSE_MIN) * Angle / SERVO270_RANGE));
}
