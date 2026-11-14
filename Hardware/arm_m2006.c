#include "arm_m2006.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

/* M2006 P36: 输出轴位置P -> 转子速度PI -> C610电流，控制周期10ms。
 * CAN1独立于底盘CAN2；8192*36/360=819.2编码值/输出轴度。 */
#define ARM_COUNTS_PER_DEG     819.2f
#define ARM_FEEDBACK_TIMEOUT_MS 100u
#define ARM_CONTROL_PERIOD_MS  10u
#define ARM_CONTROL_DT         0.01f
#define ARM_POSITION_KP        20.0f   /*角度误差deg -> 转子目标rpm；减轻终点来回纠偏，外环无积分*/
#define ARM_SPEED_KP           1.5f
#define ARM_SPEED_KI           0.1f
#define ARM_SPEED_ILIMIT       5000.0f /*速度环积分提供静摩擦/负载保持力，饱和时禁止继续同向积累*/
#define ARM_SPEED_MAX_RPM      8000.0f /*转子速度上限，36:1后输出轴约1333°/s*/
#define ARM_SPEED_RAMP_RPM_S   6000.0f /*加速限斜率；换目标沿用当前速度指令*/
#define ARM_BRAKE_RPM_S       12000.0f /*减速限斜率，同时用于剩余距离制动限速*/
#define ARM_CURRENT_MAX        10000.0f
#define ARM_ARRIVE_DEG         0.5f      /*AtTarget到位角度窗*/
#define ARM_ARRIVE_RPM         36.0f     /*转子36rpm=输出轴6°/s；角度进窗但高速掠过不算到位*/
#define ARM_POSITION_DEADBAND  0.2f      /*进入保持窗后目标速度归零，速度PI仍提供保持电流*/
#define ARM_HOLD_EXIT_DEG       0.5f     /*保持退出窗大于进入窗，避免间隙/噪声导致反复启停*/
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
static float speed_integral;
static float speed_command_rpm;  /*加减速限制后的转子目标速度*/
static uint8_t hold_active;
static volatile int16_t current_command;
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
    if (control_enabled && target_deg == (float)degrees * ARM_M2006_DIRECTION)
    {
        taskEXIT_CRITICAL();
        return 1; /*同一目标不重置停稳计数或保持力。*/
    }
    if (!control_enabled)
    {
        speed_integral = 0.0f;
        speed_command_rpm = (float)speed_rpm;
    }
    target_deg = (float)degrees * ARM_M2006_DIRECTION;
    control_enabled = 1;
    settled_cycles = 0;
    hold_active = 0; /*中途换目标保留速度与内环积分，反向先减速再加速。*/
    taskEXIT_CRITICAL();
    return 1;
}

float Arm_M2006_GetAngle(void)
{
    int32_t counts;
    taskENTER_CRITICAL();
    counts = total_counts;
    taskEXIT_CRITICAL();
    return (float)counts / ARM_COUNTS_PER_DEG;
}

float Arm_M2006_GetTargetDeg(void)
{
    float target;
    taskENTER_CRITICAL();
    target = target_deg;
    taskEXIT_CRITICAL();
    return target;
}

uint8_t Arm_M2006_GetReady(void)
{
    return Arm_M2006_Ready();
}

void Arm_M2006_GetDiagnostics(Arm_M2006_Diagnostics *out)
{
    if (!out) return;
    taskENTER_CRITICAL();
    out->angle_deg = (float)total_counts / ARM_COUNTS_PER_DEG;
    out->target_deg = target_deg;
    out->rpm = speed_rpm;
    out->current_command = current_command;
    out->ready = (uint8_t)(feedback_valid && !feedback_fault &&
        xTaskGetTickCount() - feedback_tick <= pdMS_TO_TICKS(ARM_FEEDBACK_TIMEOUT_MS));
    taskEXIT_CRITICAL();
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
    return (uint8_t)(err <= tolerance_deg && fabsf((float)rpm) <= ARM_ARRIVE_RPM);
}

void Arm_M2006_Stop(void)
{
    taskENTER_CRITICAL();
    control_enabled = 0;
    settled_cycles = 0;
    hold_active = 0;
    speed_integral = 0.0f;
    speed_command_rpm = 0.0f;
    taskEXIT_CRITICAL();
}

static int16_t arm_control_step(void)
{
    int32_t counts;
    int16_t rpm;
    uint8_t ready;
    float target, error, speed_target, speed_error, current, integral_next;

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
        speed_integral = speed_command_rpm = 0.0f;
        hold_active = 0;
        settled_cycles = 0;
        return 0;
    }

    /* 外环只做P，静摩擦由速度PI补偿；不在长行程积累位置积分。
       到位时明确把目标速度归零，不能沿用进入死区前的运动指令。 */
    error = target - (float)counts / ARM_COUNTS_PER_DEG;
    if (hold_active && fabsf(error) > ARM_HOLD_EXIT_DEG)
        hold_active = 0;
    if (!hold_active && fabsf(error) <= ARM_POSITION_DEADBAND &&
        fabsf((float)rpm) <= ARM_ARRIVE_RPM)
        hold_active = 1;
    speed_target = hold_active ? 0.0f :
        arm_limit(ARM_POSITION_KP * error, ARM_SPEED_MAX_RPM);
    /* 输出轴速度deg/s=转子rpm/6；由v²=2as得到rpm²=12*制动斜率*剩余角度。
       先按剩余距离减速，再限加减速斜率，避免到终点才突然反向刹车。 */
    {
        float brake_limit = sqrtf(12.0f * ARM_BRAKE_RPM_S * fabsf(error));
        float rate = ARM_SPEED_RAMP_RPM_S;
        speed_target = arm_limit(speed_target, brake_limit);
        if (speed_command_rpm * speed_target < 0.0f)
            speed_target = 0.0f; /*反向先减到0，下一拍再沿新方向加速。*/
        if (fabsf(speed_target) < fabsf(speed_command_rpm))
            rate = ARM_BRAKE_RPM_S;
        speed_command_rpm += arm_limit(speed_target - speed_command_rpm, rate * ARM_CONTROL_DT);
    }
    /* 连续速度PI：每拍重算电流，零速仍有积分保持力；不冻结旧运动电流。
       条件积分防止电流饱和后继续同向累积，保留反向消退积分的能力。 */
    speed_error = speed_command_rpm - (float)rpm;
    integral_next = arm_limit(speed_integral + ARM_SPEED_KI * speed_error, ARM_SPEED_ILIMIT);
    current = ARM_SPEED_KP * speed_error + integral_next;
    if (!((current > ARM_CURRENT_MAX && speed_error > 0.0f) ||
          (current < -ARM_CURRENT_MAX && speed_error < 0.0f)))
        speed_integral = integral_next;
    current = arm_limit(ARM_SPEED_KP * speed_error + speed_integral, ARM_CURRENT_MAX);

    if (fabsf(error) <= ARM_ARRIVE_DEG && fabsf((float)rpm) <= ARM_ARRIVE_RPM)
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
    current_command = current;
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
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(ARM_CONTROL_PERIOD_MS));
    }
}
