#include "hwt101_imu.h"
#include <math.h>
#include <string.h>

/* 协议依据商家HWT101协议和SDK WitSerialDataIn(WIT_PROTOCOL_NORMAL)：
   55 TYPE DATA[8] SUM，共11字节，小端有符号int16，SUM为前10字节低8位。
   0x53帧[6:7]为Yaw，乘180/32768；[8:9]为版本号，不能当角度。
   0x52是角速度，不是新航向；不使用其原始角速度字段替代Yaw。
   USART6优先级4，不调用FreeRTOS API，不等待、不写SysTick。
   模块已由上位机设为100Hz；本驱动只接收，不发配置/回读/保存/硬件记零。 */
#define HWT101_FRAME_SIZE 11u
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
    return have_yaw ? HWT101_CONFIG_READY : HWT101_CONFIG_PENDING;
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
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOG, &gpio);

    USART_StructInit(&uart);
    uart.USART_BaudRate = HWT101_BAUD_RATE;
    uart.USART_Mode = USART_Mode_Rx;
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
}
