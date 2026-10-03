#include "stm32f4xx.h"
#include "lunqu_imu.h"
#include "bno08x_uart_rvc.h"
#include <string.h>

/**
  ******************************************************************************
  * @file    lunqu_imu.c
  * @brief   轮趣 WHEELTEC FDI 惯导接收 —— 帧缓冲/对齐方式按轮趣例程,
  *          解析在中断内直接完成(不设独立解析任务)
  ******************************************************************************
  * 例程结构(与例程逐行一致, 仅外设名/变量名适配):
  *   Fd_data[64] 连续缓冲; "0xFD后跟0xFC 或 Count>0" 判在帧中; Count 计到帧长;
  *   TYPE/LEN 匹配置 flag; 收满 IMU_RS(64)/AHRS_RS(56) 且帧尾 0xFD 校验通过
  *   才 memcpy 到 Fd_rsimu/Fd_rsahrs; 解析在任务上下文(TTL_Hex2Dec 等价)。
  *   不校验 CRC(与例程一致; CRC16 仅作诊断统计)。
  * 偏移(例程实测): Fd_rsahrs[19..22]=Roll [23..26]=Pitch [27..30]=Heading(rad)
  * 唯一增补: Count>=64 强制清零(防例程中 TYPE/LEN 不匹配帧的缓冲越界写)。
  ******************************************************************************
  */

LunquData_t lunqu_data;

/* ---- 例程同款缓冲与标志 ---- */
static uint8_t Fd_data[64];
static uint8_t Fd_rsimu[64];
static uint8_t Fd_rsahrs[56];
static uint8_t lq_count = 0;             /*例程 Count: 帧内字节计数*/
static uint8_t lq_last_rsnum = 0;        /*例程 last_rsnum: 上一字节*/
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

void Lunqu_Init(uint32_t baud)
{
    GPIO_InitTypeDef        GPIO_InitStructure;
    USART_InitTypeDef       USART_InitStructure;
    NVIC_InitTypeDef        NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

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

    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART6, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = USART6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  逐字节接收 —— 例程 UART5_IRQHandler 主体逐行移植(仅变量名适配)
  * @note   由 bno08x_uart_rvc.c 的 USART6 中断按开关分流调用; 唯一增补是
  *         Count>=64 强制清零(防例程中 TYPE/LEN 不匹配帧的缓冲越界写)
  */
void Lunq_RxByte(uint8_t Usart_Receive)
{
    Fd_data[lq_count] = Usart_Receive;
    if (((lq_last_rsnum == FRAME_END) && (Usart_Receive == FRAME_HEAD)) || lq_count > 0)
    {
        lq_count++;
        if ((Fd_data[1] == TYPE_IMU)  && (Fd_data[2] == IMU_LEN))  lq_rsimu_flag = 1;
        if ((Fd_data[1] == TYPE_AHRS) && (Fd_data[2] == AHRS_LEN)) lq_rsacc_flag = 1;
        if (lq_count >= 64)                /*增补: 越界保护(例程缺陷, 必要)*/
        {
            lq_count = 0;
            lq_rsimu_flag = 0;
            lq_rsacc_flag = 0;
        }
    }
    else
        lq_count = 0;
    lq_last_rsnum = Usart_Receive;

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
                lunqu_data.roll      = LqF32(&Fd_rsahrs[19]) * 57.29578f;
                lunqu_data.pitch     = LqF32(&Fd_rsahrs[23]) * 57.29578f;
                lunqu_data.yaw       = LqF32(&Fd_rsahrs[27]) * 57.29578f;
                lunqu_data.yaw_speed = LqF32(&Fd_rsahrs[15]) * 57.29578f;
            }
        }
        memset(Fd_data, 0, 64);
    }
}




/**
  * @brief  按开关返回当前陀螺仪 yaw(deg) —— 上层(里程计/闭环)统一入口
  */
float IMU_GetYaw(void)
{
    if (lunqu_RunFlag)  return lunqu_data.yaw;
    if (Bno085_RunFlag) return bno08x_data.yaw;
    return 0.0f;
}
