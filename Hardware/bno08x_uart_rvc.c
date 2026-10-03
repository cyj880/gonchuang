#include "stm32f4xx.h"
#include "bno08x_uart_rvc.h"
#include "lunqu_imu.h"

/**
  ******************************************************************************
  * @file    bno08x_uart_rvc.c
  * @brief   BNO085 UART RVC 接收驱动（USART6 + RXNE 中断逐字节接收）
  ******************************************************************************
  * 接收机制：纯串口中断逐字节收（不用 DMA），帧头滑窗状态机对齐——
  *   空闲态：等 0xAA；收到一个 0xAA 进入预备态；再收 0xAA 进入收帧态；
  *   收满 19 字节做校验和（字节 2..14，与 TI 直角转弯例程逐字段一致）。
  *   任一字节丢失/错位，状态机最多丢一帧自动重新对齐，无累积错位。
  * 数据率 100Hz×19B = 1900 中断/秒 @168MHz，CPU 占用 <0.5%。
  * 中断优先级 4（<5），避免被 FreeRTOS 临界区（BASEPRI=5）屏蔽。
  ******************************************************************************
  */

static uint8_t rvc_buf[19];
static uint8_t rvc_idx = 0;
static uint8_t rvc_state = 0;        /*0=找头 1=已见一个AA 2=收帧中*/

BNO08X_Data_t bno08x_data;

void BNO08X_Init(uint32_t baud)
{
    GPIO_InitTypeDef        GPIO_InitStructure;
    USART_InitTypeDef       USART_InitStructure;
    NVIC_InitTypeDef        NVIC_InitStructure;

    /*开启时钟：USART6 挂 APB2，GPIOG 挂 AHB1*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

    /*GPIO：PG9 复用推挽，映射到 USART6_RX（AF8），只收*/
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6);
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;                /*RX上拉，防悬空乱码*/
    GPIO_Init(GPIOG, &GPIO_InitStructure);

    /*USART：115200-8-N-1，仅接收（波特率由 main 传入，TI 例程同为 115200）*/
    USART_InitStructure.USART_BaudRate   = baud;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits   = USART_StopBits_1;
    USART_InitStructure.USART_Parity     = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode       = USART_Mode_Rx;
    USART_Init(USART6, &USART_InitStructure);

    /*使能 RXNE 逐字节中断（ORE 溢出中断随 RXNEIE 自动使能）*/
    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART6, ENABLE);

    /*NVIC：抢占优先级 4 —— 数值必须 < 5！FreeRTOS 临界区用 BASEPRI=5
      屏蔽所有 ≥5 的中断，本中断内不调用任何 FreeRTOS API，提级合法。*/
    NVIC_InitStructure.NVIC_IRQChannel                   = USART6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* USART6 中断：RXNE 逐字节收 + 帧头滑窗状态机 + 校验解析
   （校验和字节 2..14 与 TI 直角转弯例程逐字段一致） */
void USART6_IRQHandler(void)
{
    uint8_t checkSum = 0;
    int     i;

    /*双陀螺仪开关分流: 轮趣FDI模式下喂给FDILink解析(同一USART6/PG9, 物理换插头)*/
    if (lunqu_RunFlag)
    {
        if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
        {
            Lunq_RxByte((uint8_t)USART_ReceiveData(USART6));     /*例程逐行接收(中断里完成帧缓冲)*/
        }
        if (USART_GetFlagStatus(USART6, USART_FLAG_ORE) != RESET)
        {
            (void)USART6->SR;
            (void)USART6->DR;
        }
        return;
    }

    if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
    {
        uint8_t b = (uint8_t)USART_ReceiveData(USART6);  /*读DR自动清RXNE*/

        switch (rvc_state)
        {
        case 0:                                  /*找帧头第一个AA*/
            if (b == 0xAA) rvc_state = 1;
            break;

        case 1:                                  /*已见一个AA: 再AA进收帧态*/
            if (b == 0xAA)
            {
                rvc_idx = 2;
                rvc_state = 2;
            }
            else
            {
                rvc_state = 0;
            }
            break;

        case 2:                                  /*收帧: 第2..18字节*/
            rvc_buf[rvc_idx++] = b;
            if (rvc_idx >= 19)
            {
                for (i = 2; i <= 14; i++)        /*校验和：与 TI 例程一致 2..14*/
                    checkSum += rvc_buf[i];

                if (checkSum == rvc_buf[18])
                {
                    bno08x_data.index = rvc_buf[2];
                    bno08x_data.yaw   = (int16_t)((rvc_buf[4] << 8) | rvc_buf[3]) / 100.0;
                    bno08x_data.pitch = (int16_t)((rvc_buf[6] << 8) | rvc_buf[5]) / 100.0;
                    bno08x_data.roll  = (int16_t)((rvc_buf[8] << 8) | rvc_buf[7]) / 100.0;
                    bno08x_data.ax    = (rvc_buf[10] << 8) | rvc_buf[9];
                    bno08x_data.ay    = (rvc_buf[12] << 8) | rvc_buf[11];
                    bno08x_data.az    = (rvc_buf[14] << 8) | rvc_buf[13];
                }
                rvc_state = 0;                   /*回找头态, 下一帧自动对齐*/
            }
            break;

        default:
            rvc_state = 0;
            break;
        }
    }

    /*溢出保护：理论上不应发生(每字节中断即收)，读SR+DR清除即可*/
    if (USART_GetFlagStatus(USART6, USART_FLAG_ORE) != RESET)
    {
        (void)USART6->SR;
        (void)USART6->DR;
    }
}
