#include "arm_m2006.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

/* Ported from the supplied M2006 demo: 36:1 reduction, cascaded position /
 * speed PI loops at 100 Hz. CAN1 is independent of the CAN2 chassis bus. */
#define ARM_COUNTS_PER_DEG     819.2f
#define ARM_FEEDBACK_TIMEOUT_MS 100u
/* 参数对照官方 DEMO(位置环Kp120/限幅15000、速度环Kp1.5/Ki0.1、100Hz、无斜坡):
   这里取 DEMO 的约四成力度——比原来快约3倍, 又留出机械冲击和过冲余量。
   位置环Kp换算: 速度指令(rpm) = Kp * 角度误差(deg), Kp=50 即 1°误差对应50rpm转子
   ≈8.3°/s输出轴, 时间常数36/(50*6)=0.12s; 原来Kp=20时时间常数0.3s, 且转速死区
   10rpm正好对应0.5°误差, 末端会被死区吃掉电流, 这是"慢+尾部爬不动"的主因。 */
#define ARM_POSITION_KP        50.0f
#define ARM_POSITION_KI        0.0f    /*外环纯P: 长行程无积分可憋, 消除末端超调; 摩擦由内环速度PI补偿*/
#define ARM_SPEED_KP           1.5f
#define ARM_SPEED_KI           0.1f
#define ARM_SPEED_MAX_RPM      4000.0f   /*转子限速, 36:1后约111RPM输出轴(DEMO为15000)*/
#define ARM_SPEED_RAMP_RPM_S   12000.0f  /*目标转速变化率: 0→4000rpm约0.33s, 起步不反冲*/
#define ARM_CURRENT_MAX        10000.0f
#define ARM_ARRIVE_DEG         0.5f      /*AtTarget到位角度窗*/
#define ARM_ARRIVE_RPM         36.0f     /*到位转速窗: 36rpm转子=1rpm输出轴=6°/s*/
#define ARM_POSITION_DEADBAND  0.2f      /*位置死区: 窗内转速指令清零, 靠摩擦停住*/
#define ARM_SPEED_DEADBAND_RPM 10.0f     /*速度环死区: 与0.2°误差对应, 抑制末端抖动*/
#define ARM_SETTLE_CYCLES      10u       /*窗内连续10个控制周期(100ms)才算停稳*/

#if ARM_M2006_MOTOR_ID < 1 || ARM_M2006_MOTOR_ID > 4
#error ARM_M2006_MOTOR_ID must be between 1 and 4
#endif

static volatile uint8_t feedback_valid;
static volatile uint8_t feedback_fault;
static volatile uint16_t last_angle;
static volatile int32_t total_counts;
static volatile int16_t speed_rpm;
static volatile TickType_t feedback_tick;
static volatile float target_deg;
static volatile uint8_t control_enabled = 1;
static volatile uint8_t settled_cycles;
static float position_integral;
static float speed_integral;
static float speed_command_rpm;  /*限速+斜坡后的目标转速, 非反馈转速*/
static uint8_t tx_mailbox = CAN_TxStatus_NoMailBox;

static float arm_limit(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

void Arm_M2006_Init(void)
{
    GPIO_InitTypeDef gpio;
    CAN_InitTypeDef config;
    CAN_FilterInitTypeDef filter;
    NVIC_InitTypeDef irq;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &gpio);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);

    /* Do not CAN_DeInit(CAN1): CAN2 shares the master clock/filter block. */
    CAN_StructInit(&config);
    config.CAN_ABOM = ENABLE;
    config.CAN_Mode = CAN_Mode_Normal;
    config.CAN_SJW = CAN_SJW_1tq;
    config.CAN_BS1 = CAN_BS1_9tq;
    config.CAN_BS2 = CAN_BS2_4tq;
    config.CAN_Prescaler = 3; /* APB1 42 MHz / (3 * 14) = 1 Mbps. */
    CAN_Init(CAN1, &config);

    /* Bank 0 belongs to CAN1; the chassis keeps its existing bank 14. */
    filter.CAN_FilterNumber = 0;
    filter.CAN_FilterMode = CAN_FilterMode_IdMask;
    filter.CAN_FilterScale = CAN_FilterScale_32bit;
    filter.CAN_FilterIdHigh = (uint16_t)((0x200u + ARM_M2006_MOTOR_ID) << 5);
    filter.CAN_FilterIdLow = 0;
    filter.CAN_FilterMaskIdHigh = 0xFFE0;
    filter.CAN_FilterMaskIdLow = 0x0006; /* Standard, data frame only. */
    filter.CAN_FilterFIFOAssignment = CAN_FIFO0;
    filter.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&filter);

    irq.NVIC_IRQChannel = CAN1_RX0_IRQn;
    irq.NVIC_IRQChannelPreemptionPriority = 6;
    irq.NVIC_IRQChannelSubPriority = 0;
    irq.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&irq);
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg frame;
    while (CAN_MessagePending(CAN1, CAN_FIFO0) != 0)
    {
        uint16_t angle;
        int32_t delta;
        TickType_t now;
        CAN_Receive(CAN1, CAN_FIFO0, &frame);
        if (frame.IDE != CAN_Id_Standard || frame.RTR != CAN_RTR_Data ||
            frame.StdId != 0x200u + ARM_M2006_MOTOR_ID || frame.DLC != 8)
            continue;
        angle = (uint16_t)(((uint16_t)frame.Data[0] << 8) | frame.Data[1]);
        if (angle >= 8192u) continue;
        now = xTaskGetTickCountFromISR();
        if (!feedback_valid)
        {
            /* First actual feedback is the zero reference, even above 4096. */
            total_counts = 0;
            feedback_valid = 1;
        }
        else
        {
            /* Lost feedback makes multi-turn position uncertain. Require a
             * restart rather than silently assigning a new zero mid-move. */
            if ((now - feedback_tick) > pdMS_TO_TICKS(ARM_FEEDBACK_TIMEOUT_MS))
                feedback_fault = 1;
            delta = (int32_t)angle - last_angle;
            if (delta > 4096) delta -= 8192;
            if (delta < -4096) delta += 8192;
            total_counts += delta;
        }
        last_angle = angle;
        speed_rpm = (int16_t)(((uint16_t)frame.Data[2] << 8) | frame.Data[3]);
        feedback_tick = now;
    }
}

uint8_t Arm_M2006_Ready(void)
{
    uint8_t ready;
    taskENTER_CRITICAL();
    ready = (uint8_t)(feedback_valid && !feedback_fault &&
        (xTaskGetTickCount() - feedback_tick) <=
        pdMS_TO_TICKS(ARM_FEEDBACK_TIMEOUT_MS));
    taskEXIT_CRITICAL();
    return ready;
}

uint8_t Arm_M2006_Extend(void)
{
    return Arm_M2006_SetTargetDeg((int16_t)ARM_M2006_EXTEND_DEG);
}

uint8_t Arm_M2006_SetTargetDeg(int16_t degrees)
{
    if (degrees < ARM_M2006_MIN_TARGET_DEG || degrees > ARM_M2006_MAX_TARGET_DEG ||
        !Arm_M2006_Ready()) return 0;
    taskENTER_CRITICAL();
    target_deg = (float)degrees * ARM_M2006_DIRECTION;
    control_enabled = 1;
    settled_cycles = 0;
    position_integral = 0.0f;
    speed_integral = 0.0f;
    speed_command_rpm = (float)speed_rpm; /*从实际转速起步, 中途换目标也不突然制动*/
    taskEXIT_CRITICAL();
    return 1;
}

uint8_t Arm_M2006_AtTarget(void)
{
    return (uint8_t)(Arm_M2006_Ready() && control_enabled &&
                     settled_cycles >= ARM_SETTLE_CYCLES);
}

/* 夹取序列用: 位置进入指定容差且转子速度足够低即可, 不要求额外10周期稳定,
   避免减速箱间隙/PID微调让状态机在目标附近多等数秒。 */
uint8_t Arm_M2006_AtTargetWithin(float tolerance_deg)
{
    uint8_t ready;
    int32_t counts;
    int16_t rpm;
    float target, err;

    if (tolerance_deg < 0.0f) return 0;
    taskENTER_CRITICAL();
    ready = (uint8_t)(feedback_valid && !feedback_fault && control_enabled &&
        (xTaskGetTickCount() - feedback_tick) <=
        pdMS_TO_TICKS(ARM_FEEDBACK_TIMEOUT_MS));
    counts = total_counts;
    rpm = speed_rpm;
    target = target_deg;
    taskEXIT_CRITICAL();
    if (!ready) return 0;
    err = fabsf(target - (float)counts / ARM_COUNTS_PER_DEG);
    /* 已进入位置死区(0.2°): 位置环早已把转速指令清零、电流归零, 只可能残余抖动,
       此时不再要求转速窗, 否则间隙/摩擦会让状态机在门口多等好几拍。 */
    if (tolerance_deg >= ARM_POSITION_DEADBAND && err <= ARM_POSITION_DEADBAND) return 1;
    if (rpm < -ARM_ARRIVE_RPM || rpm > ARM_ARRIVE_RPM) return 0;
    return (uint8_t)(err <= tolerance_deg);
}

void Arm_M2006_Stop(void)
{
    taskENTER_CRITICAL();
    control_enabled = 0;
    settled_cycles = 0;
    speed_command_rpm = 0.0f;
    taskEXIT_CRITICAL();
}

static int16_t arm_control_step(void)
{
    int32_t counts;
    int16_t rpm;
    uint8_t ready;
    float target, error, speed_target, speed_error, current;

    taskENTER_CRITICAL();
    if (feedback_valid && (xTaskGetTickCount() - feedback_tick) >
        pdMS_TO_TICKS(ARM_FEEDBACK_TIMEOUT_MS)) feedback_fault = 1;
    ready = (uint8_t)(feedback_valid && !feedback_fault && control_enabled);
    counts = total_counts;
    rpm = speed_rpm;
    target = target_deg;
    taskEXIT_CRITICAL();
    if (!ready)
    {
        position_integral = speed_integral = 0.0f;
        speed_command_rpm = 0.0f;
        settled_cycles = 0;
        return 0;
    }

    error = target - (float)counts / ARM_COUNTS_PER_DEG;
    /* Unlike the demo's deadband branch, never reuse the previous output. */
    if (fabsf(error) <= ARM_POSITION_DEADBAND)
    {
        position_integral = 0.0f;
        speed_target = 0.0f;
    }
    else if (fabsf(error) <= 5.0f)
    {
        /* 终点积分带: 小限幅积分顶开静摩擦, 精确爬到目标; 量小不会超调。 */
        position_integral = arm_limit(position_integral + ARM_POSITION_KI * error, 50.0f);
        speed_target = arm_limit(ARM_POSITION_KP * error + position_integral, ARM_SPEED_MAX_RPM);
    }
    else
    {
        /* 远端: 抗积分饱和——转速指令顶到限幅且误差同向时, 停止同向积分,
           长行程慢速移动不会把积分憋满, 终点由比例项主导, 不冲过头。 */
        float unsat = ARM_POSITION_KP * error + position_integral;

        if ((unsat >= ARM_SPEED_MAX_RPM && error > 0.0f) ||
            (unsat <= -ARM_SPEED_MAX_RPM && error < 0.0f))
        {
            speed_target = arm_limit(unsat, ARM_SPEED_MAX_RPM);
        }
        else
        {
            position_integral = arm_limit(position_integral + ARM_POSITION_KI * error, 3000.0f);
            speed_target = arm_limit(ARM_POSITION_KP * error + position_integral, ARM_SPEED_MAX_RPM);
        }
    }
    {
        /* 100Hz斜坡: 只限制加速(柔顺起步减小反冲); 减速/刹车不限——
           位置环算出的转速随误差收缩, 末端直接跟随, 避免斜坡刹不住车导致超调。 */
        float max_delta = ARM_SPEED_RAMP_RPM_S * 0.01f;
        float delta = speed_target - speed_command_rpm;
        if (fabsf(speed_target) >= fabsf(speed_command_rpm))
            speed_command_rpm += arm_limit(delta, max_delta);   /* 加速: 限斜坡 */
        else
            speed_command_rpm = speed_target;                   /* 减速: 直接跟随 */
        speed_target = speed_command_rpm;
    }
    speed_error = speed_target - (float)rpm;
    if (fabsf(speed_error) <= ARM_SPEED_DEADBAND_RPM)
    {
        speed_integral = 0.0f;
        current = 0.0f;
    }
    else
    {
        speed_integral = arm_limit(speed_integral + ARM_SPEED_KI * speed_error, 5000.0f);
        current = arm_limit(ARM_SPEED_KP * speed_error + speed_integral, ARM_CURRENT_MAX);
    }
    if (fabsf(error) <= ARM_ARRIVE_DEG && rpm >= -ARM_ARRIVE_RPM && rpm <= ARM_ARRIVE_RPM)
    {
        if (settled_cycles < ARM_SETTLE_CYCLES) settled_cycles++;
    }
    else settled_cycles = 0;
    return (int16_t)current;
}

static void arm_send_current(int16_t current)
{
    CanTxMsg frame = {0};
    uint8_t slot = (ARM_M2006_MOTOR_ID - 1u) * 2u;
    /* One pending mailbox at most, including when the ESC is unplugged. */
    if (tx_mailbox != CAN_TxStatus_NoMailBox &&
        CAN_TransmitStatus(CAN1, tx_mailbox) == CAN_TxStatus_Pending)
        CAN_CancelTransmit(CAN1, tx_mailbox);
    frame.StdId = 0x200;
    frame.IDE = CAN_Id_Standard;
    frame.RTR = CAN_RTR_Data;
    frame.DLC = 8;
    frame.Data[slot] = (uint8_t)((uint16_t)current >> 8);
    frame.Data[slot + 1u] = (uint8_t)current;
    tx_mailbox = CAN_Transmit(CAN1, &frame);
}

void Arm_M2006_Task(void *argument)
{
    TickType_t wake = xTaskGetTickCount();
    (void)argument;
    for (;;)
    {
        arm_send_current(arm_control_step());
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
    }
}
