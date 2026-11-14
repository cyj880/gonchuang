#include "stm32f4xx.h"
#include "lunqu_imu.h"
#include <string.h>
#include <math.h>

/**
  ******************************************************************************
  * @file    lunqu_imu.c
  * @brief   轮趣 WHEELTEC FDI 惯导接收 —— 帧缓冲/对齐方式按轮趣例程,
  *          解析在中断内直接完成(不设独立解析任务)
  ******************************************************************************
  * 沿用移植前轮趣例程的帧长/字段布局：
  *   Fd_data[64] 连续缓冲; 空闲时遇FC开始，Count 计到帧长；
  *   TYPE/LEN 匹配置 flag; 收满 IMU_RS(64)/AHRS_RS(56) 且帧尾 0xFD 校验通过
  *   才 memcpy 到 Fd_rsimu/Fd_rsahrs; AHRS直接在接收中断解析。
  *   不校验 CRC，沿用移植前轮趣例程接收规则。
  * 偏移(例程实测): Fd_rsahrs[19..22]=Roll [23..26]=Pitch [27..30]=Heading(rad)
  * 收满帧后再检查64字节边界，串口错误或非法float不会发布新航向。
  ******************************************************************************
  */

LunquData_t lunqu_data;
static volatile uint8_t lq_have_yaw;

/* ---- 零点机制: 上电/按PB2时刻的朝向 = 0 deg, yaw以相对零点输出 ----
   闭环(IMU_GetYaw)用+-180相对角(PID连续); 显示(IMU_GetYawAbs360)用0~360 */
static float    lq_yaw_zero_deg = 0.0f;
static uint8_t  lq_zero_done = 0;
static float    lq_yaw_raw_deg = 0.0f;
static volatile uint32_t imu_update_sequence = 0;

/* ---- 例程同款缓冲与标志 ---- */
static uint8_t Fd_data[64];
static uint8_t Fd_rsimu[64];
static uint8_t Fd_rsahrs[56];
static uint8_t lq_count = 0;             /*例程 Count: 帧内字节计数*/
static uint8_t lq_rsimu_flag = 0;
static uint8_t lq_rsacc_flag = 0;

/* ---- 例程协议常量(sys.h) ---- */
#define FRAME_HEAD 0xFC
#define FRAME_END  0xFD
#define TYPE_IMU   0x40
#define TYPE_AHRS  0x41
#define IMU_LEN    0x38        /*56字节载荷*/
#define AHRS_LEN   0x30        /*48字节载荷*/
#define IMU_RS     64          /*IMU整帧长*/
#define AHRS_RS    56          /*AHRS整帧长*/

/* ---- float32 字段按例程字节序还原: 第一个字节为最低有效字节 ---- */
static float LqF32(const uint8_t *p)
{
    float f;
    memcpy(&f, p, 4);        /*CM4 小端, 与官方 DATA_Trans 例程一致*/
    return f;
}

/* ---- 角度归一 ---- */
static float LqNorm180(float a)
{
    a = fmodf(a, 360.0f);
    while (a > 180.0f)   a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}
static float LqNorm360(float a)
{
    a = fmodf(a, 360.0f);
    if (a < 0.0f) a += 360.0f;
    return a;
}

/* 当前物理朝向设为指定软件yaw：短暂关中断，避免高优先级USART6打断更新。 */
void Lunq_SetYaw(float yaw_deg)
{
    uint32_t primask;
    yaw_deg = LqNorm180(yaw_deg);
    primask = __get_PRIMASK();
    __disable_irq();
    if (lq_have_yaw) lq_yaw_zero_deg = lq_yaw_raw_deg - yaw_deg;
    lq_zero_done = lq_have_yaw; /* 首帧前设角时，首帧再建立偏移。 */
    lunqu_data.yaw = LqNorm180(yaw_deg);
    lunqu_data.yaw_abs360 = LqNorm360(yaw_deg);
    __set_PRIMASK(primask);
}

/* PB2/连续两批启动：当前朝向记为0°。 */
void Lunq_SetZero(void)
{
    Lunq_SetYaw(0.0f);
}

void Lunqu_Init(uint32_t baud)
{
    GPIO_InitTypeDef        GPIO_InitStructure;
    USART_InitTypeDef       USART_InitStructure;
    NVIC_InitTypeDef        NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

    NVIC_DisableIRQ(USART6_IRQn);
    USART_Cmd(USART6, DISABLE);
    USART_ITConfig(USART6, USART_IT_TXE, DISABLE);
    USART_ITConfig(USART6, USART_IT_TC, DISABLE);
    memset(&lunqu_data, 0, sizeof(lunqu_data));
    lq_yaw_zero_deg = lq_yaw_raw_deg = 0.0f;
    lq_zero_done = lq_have_yaw = 0;
    imu_update_sequence = 0;
    lq_count = lq_rsimu_flag = lq_rsacc_flag = 0;
    memset(Fd_data, 0, sizeof(Fd_data));
    memset(Fd_rsimu, 0, sizeof(Fd_rsimu));
    memset(Fd_rsahrs, 0, sizeof(Fd_rsahrs));

    GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOG, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = baud;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx;
    USART_Init(USART6, &USART_InitStructure);

    (void)USART6->SR;
    (void)USART6->DR;
    NVIC_ClearPendingIRQ(USART6_IRQn);
    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART6, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = USART6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  逐字节接收轮趣IMU/AHRS帧，使用原项目字段布局。
  * @note   由imu.c统一USART6中断按上电选择分流；未知帧最多缓存64字节。
  */
void Lunq_RxByte(uint8_t Usart_Receive)
{
    Fd_data[lq_count] = Usart_Receive;
    if (Usart_Receive == FRAME_HEAD || lq_count > 0) /* 允许直接从上电首个FC帧头接收。 */
    {
        lq_count++;
        if (lq_count >= 3 && (Fd_data[1] == TYPE_IMU)  && (Fd_data[2] == IMU_LEN))  lq_rsimu_flag = 1;
        if (lq_count >= 3 && (Fd_data[1] == TYPE_AHRS) && (Fd_data[2] == AHRS_LEN)) lq_rsacc_flag = 1;
    }
    else
        lq_count = 0;

    if (lq_rsimu_flag == 1 && lq_count == IMU_RS)      /*收满IMU整帧*/
    {
        lq_count = 0;
        lq_rsimu_flag = 0;
        if (Fd_data[IMU_RS - 1] == FRAME_END)          /*帧尾校验通过才拷贝*/
            memcpy(Fd_rsimu, Fd_data, IMU_RS);         /*IMU帧当前不使用, 仅保持例程行为*/
    }
    if (lq_rsacc_flag == 1 && lq_count == AHRS_RS)     /*收满AHRS整帧*/
    {
        lq_count = 0;
        lq_rsacc_flag = 0;
        if (Fd_data[AHRS_RS - 1] == FRAME_END)
        {
            memcpy(Fd_rsahrs, Fd_data, AHRS_RS);
            /*解析直接在中断内完成(卡死根因是crc16_idx越界, 已修复):*/
            if (Fd_rsahrs[1] == TYPE_AHRS && Fd_rsahrs[2] == AHRS_LEN)
            {
                /* 拒绝NaN/Inf或明显非法航向，避免坏float进入角差归一循环。 */
                float yaw_rad = LqF32(&Fd_rsahrs[27]);
                if (!(yaw_rad >= -100.0f && yaw_rad <= 100.0f))
                {
                    memset(Fd_data, 0, sizeof(Fd_data));
                    return;
                }
                lunqu_data.roll      = LqF32(&Fd_rsahrs[19]) * 57.29578f;
                lunqu_data.pitch     = LqF32(&Fd_rsahrs[23]) * 57.29578f;
                lq_yaw_raw_deg       = yaw_rad * 57.29578f;
                lunqu_data.yaw_speed = LqF32(&Fd_rsahrs[15]) * 57.29578f;

                if (!lq_zero_done)            /*上电首帧: 当前朝向=0 deg*/
                {
                    lq_yaw_zero_deg = lq_yaw_raw_deg - lunqu_data.yaw;
                    lq_zero_done = 1;
                }
                lunqu_data.yaw       = LqNorm180(lq_yaw_raw_deg - lq_yaw_zero_deg);
                lunqu_data.yaw_abs360 = LqNorm360(lq_yaw_raw_deg - lq_yaw_zero_deg);
                lq_have_yaw = 1;
                imu_update_sequence++;
            }
        }
        memset(Fd_data, 0, 64);
    }
    if (lq_count >= sizeof(Fd_data)) /* 先处理完整64字节IMU帧，再保护未知帧边界。 */
    {
        lq_count = lq_rsimu_flag = lq_rsacc_flag = 0;
        memset(Fd_data, 0, sizeof(Fd_data));
    }
}




/* 轮趣后端只提供本款航向；统一选择和USART6入口在imu.c。 */
uint8_t Lunq_Ready(void) { return lq_have_yaw; }
float Lunq_GetYaw(void) { return lunqu_data.yaw; }
float Lunq_GetYawAbs360(void) { return lunqu_data.yaw_abs360; }
uint32_t Lunq_UpdateSequence(void) { return imu_update_sequence; }

void Lunq_GetSample(IMU_Sample_t *out)
{
    uint32_t primask;
    if (out == NULL) return;
    primask = __get_PRIMASK();
    __disable_irq();
    out->yaw_deg = lunqu_data.yaw;
    out->sequence = imu_update_sequence;
    __set_PRIMASK(primask);
}

void Lunqu_IRQHandler(void)
{
    uint32_t status = USART6->SR;
    if (status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE))
    {
        uint8_t byte = (uint8_t)USART6->DR;
        if (status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE))
        {
            lq_count = lq_rsimu_flag = lq_rsacc_flag = 0;
            memset(Fd_data, 0, sizeof(Fd_data));
        }
        else if (status & USART_SR_RXNE)
            Lunq_RxByte(byte);
    }
}
