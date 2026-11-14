#include "hwt101_imu.h"
#include <math.h>
#include <string.h>

/* 协议依据商家HWT101协议和SDK WitSerialDataIn(WIT_PROTOCOL_NORMAL)：
   55 TYPE DATA[8] SUM，共11字节，小端有符号int16，SUM为前10字节低8位。
   0x53帧[6:7]为Yaw，乘180/32768；[8:9]为版本号，不能当角度。
   0x52是角速度，不是新航向；不使用其原始角速度字段替代Yaw。
   USART6优先级4，不调用FreeRTOS API，不等待、不写SysTick。
   仅启动配置时查询输出速率，不查询电机/编码器、不发送硬件记零。 */
#define HWT101_FRAME_SIZE 11u
#define HWT101_CMD_SIZE 5u
#define HWT101_TX_TIMEOUT_MS 250u
#define HWT101_REPLY_TIMEOUT_MS 500u
#define HWT101_CONFIG_GAP_MS 200u
#define HWT101_BOOT_TIMEOUT_MS 3000u
#define HWT101_CONFIG_ATTEMPTS 3u

typedef enum
{
    CFG_WAIT_DATA, CFG_READ, CFG_UNLOCK, CFG_WRITE, CFG_CHECK, CFG_SAVE, CFG_RATE
} ConfigStep;

static ConfigStep config_step;
static volatile HWT101_ConfigStatus_t config_status;
static uint32_t config_tick, rate_sequence;
static uint8_t config_started, config_attempts, tx_pending;
static volatile uint8_t tx_frame[HWT101_CMD_SIZE];
static volatile uint8_t tx_index;
static volatile uint8_t tx_busy, read_pending, reg_reply;
static volatile uint16_t reply_rate, reply_baud;

static uint8_t rx_frame[HWT101_FRAME_SIZE];
static uint8_t rx_count;
static volatile float yaw_raw_deg;
static volatile float yaw_zero_deg;
static volatile float yaw_deg;
static volatile uint8_t have_yaw;
static volatile uint32_t update_sequence;

static float norm360(float angle)
{
    angle = fmodf(angle, 360.0f);
    if (angle < 0.0f) angle += 360.0f;
    return angle;
}

static float norm180(float angle)
{
    angle = norm360(angle);
    if (angle > 180.0f) angle -= 360.0f;
    return angle;
}

void HWT101_SetYaw(float angle)
{
    uint32_t primask;
    angle = norm180(angle);
    primask = __get_PRIMASK();
    __disable_irq();
    yaw_deg = angle;
    if (have_yaw) yaw_zero_deg = yaw_raw_deg - angle;
    /* 尚无角度帧时保留请求角度，首帧到达后再建立偏移。 */
    __set_PRIMASK(primask);
}

void HWT101_SetZero(void)
{
    HWT101_SetYaw(0.0f);
}

float HWT101_GetYaw(void)
{
    return yaw_deg;
}

float HWT101_GetYawAbs360(void)
{
    return norm360(yaw_deg);
}

uint32_t HWT101_UpdateSequence(void)
{
    return update_sequence;
}

void HWT101_GetSample(IMU_Sample_t *out)
{
    uint32_t primask;
    if (out == NULL) return;
    /* USART6优先级4高于RTOS临界区屏蔽范围，短暂关中断取得一致快照。 */
    primask = __get_PRIMASK();
    __disable_irq();
    out->yaw_deg = yaw_deg;
    out->sequence = update_sequence;
    __set_PRIMASK(primask);
}

HWT101_ConfigStatus_t HWT101_ConfigStatus(void)
{
    return config_status;
}

/* TXE/TC中断发完整5字节；任务只入队，不轮询TXE或等待发送结束。 */
static void config_send(uint8_t reg, uint16_t value, ConfigStep next, uint32_t now)
{
    tx_frame[0] = 0xFFu;
    tx_frame[1] = 0xAAu;
    tx_frame[2] = reg;
    tx_frame[3] = (uint8_t)value;
    tx_frame[4] = (uint8_t)(value >> 8);
    tx_index = 0;
    config_step = next;
    config_tick = now;
    tx_pending = tx_busy = 1;
    reg_reply = 0;
    read_pending = (uint8_t)(reg == 0x27u);
    USART_ClearITPendingBit(USART6, USART_IT_TC);
    USART_ITConfig(USART6, USART_IT_TXE, ENABLE);
}

static void config_retry(uint32_t now)
{
    USART_ITConfig(USART6, USART_IT_TXE, DISABLE);
    USART_ITConfig(USART6, USART_IT_TC, DISABLE);
    tx_busy = tx_pending = read_pending = reg_reply = 0;
    config_tick = now;
    config_step = CFG_WAIT_DATA;
    if (config_attempts >= HWT101_CONFIG_ATTEMPTS)
        config_status = HWT101_CONFIG_FAILED;
}

static void config_measure_rate(uint32_t now)
{
    read_pending = 0;
    rate_sequence = update_sequence;
    config_tick = now;
    config_step = CFG_RATE;
}

void HWT101_Poll(uint32_t now)
{
    uint32_t elapsed;
    if (config_status != HWT101_CONFIG_PENDING) return;
    if (!config_started)
    {
        config_tick = now;
        config_started = 1;
    }
    elapsed = now - config_tick; /* 无符号差支持毫秒回绕。 */
    if (tx_busy)
    {
        if (elapsed >= HWT101_TX_TIMEOUT_MS) config_retry(now);
        return;
    }
    if (tx_pending)
    {
        tx_pending = 0;
        config_tick = now; /* TC确认发完后再计间隔，保证至少200ms。 */
        elapsed = 0;
    }
    switch (config_step)
    {
    case CFG_WAIT_DATA:
        if (have_yaw)
        {
            ++config_attempts;
            config_send(0x27u, 0x0003u, CFG_READ, now); /* 回读RRATE及BAUD。 */
        }
        else if (elapsed >= HWT101_BOOT_TIMEOUT_MS)
            config_status = HWT101_CONFIG_FAILED;
        break;
    case CFG_READ:
    case CFG_CHECK:
        if (reg_reply)
        {
            read_pending = 0;
            if (reply_baud != 0x0006u) config_retry(now);
            else if (reply_rate == 0x0009u)
            {
                if (config_step == CFG_READ) config_measure_rate(now);
                else config_send(0x00u, 0x0000u, CFG_SAVE, now); /* 改过才保存。 */
            }
            else if (config_step == CFG_READ)
                config_send(0x69u, 0xB588u, CFG_UNLOCK, now);
            else config_retry(now);
        }
        else if (elapsed >= HWT101_REPLY_TIMEOUT_MS) config_retry(now);
        break;
    case CFG_UNLOCK:
        if (elapsed >= HWT101_CONFIG_GAP_MS)
            config_send(0x03u, 0x0009u, CFG_WRITE, now); /* RRATE=100Hz。 */
        break;
    case CFG_WRITE:
        if (elapsed >= HWT101_CONFIG_GAP_MS)
            config_send(0x27u, 0x0003u, CFG_CHECK, now);
        break;
    case CFG_SAVE:
        if (elapsed >= HWT101_CONFIG_GAP_MS) config_measure_rate(now);
        break;
    case CFG_RATE:
        /* 除寄存器回读外，再确认实际角度帧频率，避免把10Hz判为成功。
           300ms窗口允许调度抖动，至少80Hz；不足时最多观察1秒。 */
        if (elapsed >= 300u && (uint64_t)(update_sequence - rate_sequence) * 1000u >=
                               (uint64_t)elapsed * 80u)
            config_status = HWT101_CONFIG_READY;
        else if (elapsed >= 1000u) config_retry(now);
        break;
    }
}

void HWT101_RxByte(uint8_t byte)
{
    uint8_t i, sum = 0;
    int16_t raw;

    rx_frame[rx_count++] = byte;
    /* 滑窗保留坏帧内部可能的新55帧头；丢字节/插入噪声后能重新对齐。 */
    while (rx_count && rx_frame[0] != 0x55u)
    {
        --rx_count;
        memmove(rx_frame, rx_frame + 1, rx_count);
    }
    if (rx_count < HWT101_FRAME_SIZE) return;

    for (i = 0; i < HWT101_FRAME_SIZE - 1u; ++i) sum += rx_frame[i];
    if (sum != rx_frame[HWT101_FRAME_SIZE - 1u])
    {
        --rx_count;
        memmove(rx_frame, rx_frame + 1, rx_count);
        return;
    }
    if (rx_frame[1] == 0x53u)
    {
        raw = (int16_t)((uint16_t)rx_frame[6] | ((uint16_t)rx_frame[7] << 8));
        yaw_raw_deg = (float)raw * (180.0f / 32768.0f) * HWT101_YAW_SIGN;
        if (!have_yaw)
        {
            yaw_zero_deg = yaw_raw_deg - yaw_deg;
            have_yaw = 1;
        }
        yaw_deg = norm180(yaw_raw_deg - yaw_zero_deg);
        ++update_sequence;
    }
    else if (rx_frame[1] == 0x5Fu && read_pending)
    {
        /* 本驱动只读起始寄存器03：四个返回字中的前两个为RRATE、BAUD。 */
        reply_rate = (uint16_t)rx_frame[2] | ((uint16_t)rx_frame[3] << 8);
        reply_baud = (uint16_t)rx_frame[4] | ((uint16_t)rx_frame[5] << 8);
        reg_reply = 1;
    }
    rx_count = 0;
}

void HWT101_Init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;
    NVIC_InitTypeDef nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
    NVIC_DisableIRQ(USART6_IRQn);
    USART_Cmd(USART6, DISABLE);
    USART_ITConfig(USART6, USART_IT_TXE, DISABLE);
    USART_ITConfig(USART6, USART_IT_TC, DISABLE);

    rx_count = 0;
    have_yaw = 0;
    yaw_raw_deg = yaw_zero_deg = yaw_deg = 0.0f;
    update_sequence = 0;
    config_status = HWT101_CONFIG_PENDING;
    config_step = CFG_WAIT_DATA;
    config_started = config_attempts = tx_pending = tx_busy = tx_index = 0;
    read_pending = reg_reply = 0;

    GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_USART6);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOG, &gpio);

    USART_StructInit(&uart);
    uart.USART_BaudRate = HWT101_BAUD_RATE;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART6, &uart); /* 8数据位/1停止位/无校验/无流控。 */
    (void)USART6->SR;
    (void)USART6->DR;
    NVIC_ClearPendingIRQ(USART6_IRQn);
    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

    nvic.NVIC_IRQChannel = USART6_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 4;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    USART_Cmd(USART6, ENABLE);
}

void HWT101_IRQHandler(void)
{
    uint32_t status = USART6->SR;
    if (status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE))
    {
        /* 一次SR->DR清RXNE及错误，避免二次读DR吞掉刚到的新字节。 */
        uint8_t byte = (uint8_t)USART6->DR;
        if (status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE))
            rx_count = 0; /* 丢弃受影响帧，等待下一个校验合格帧。 */
        else if (status & USART_SR_RXNE)
            HWT101_RxByte(byte);
    }
    /* 先判断TC，避免最后一字节刚写DR就误用旧SR中的TC位。 */
    if ((status & USART_SR_TC) && (USART6->CR1 & USART_CR1_TCIE))
    {
        USART_ITConfig(USART6, USART_IT_TC, DISABLE);
        USART_ClearITPendingBit(USART6, USART_IT_TC);
        tx_busy = 0;
    }
    if ((status & USART_SR_TXE) && (USART6->CR1 & USART_CR1_TXEIE))
    {
        USART6->DR = tx_frame[tx_index++];
        if (tx_index == HWT101_CMD_SIZE)
        {
            USART_ITConfig(USART6, USART_IT_TXE, DISABLE);
            USART_ITConfig(USART6, USART_IT_TC, ENABLE);
        }
    }
}
