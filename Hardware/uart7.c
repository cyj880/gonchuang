#include "stm32f4xx.h"
#include "uart7.h"
#include "route_rx.h"

/**
  * UART7 串口（上位机坐标通信口）
  *   PE7 = UART7_RX（AF8）  接上位机 TX
  *   PE8 = UART7_TX（AF8）  接上位机 RX
  * 接收：RXNE 中断逐字节喂 route_rx_byte() 状态机
  * 发送：阻塞式（供任务回传坐标文本，115200 下 1KB 约 90ms）
  */

void UART7_ZigBee_Init(uint32_t baud)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /*开启时钟*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7, ENABLE);        //UART7挂APB1
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    /*GPIO：PE7/PE8 复用推挽，映射到 UART7*/
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_UART7);     //PE7 = UART7_RX
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_UART7);     //PE8 = UART7_TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                 //RX上拉，防止悬空乱码
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    /*USART：115200-8-N-1*/
    USART_InitStructure.USART_BaudRate = baud;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART7, &USART_InitStructure);

    /*NVIC：抢占优先级6（数值≥5，落在 configMAX_SYSCALL_INTERRUPT_PRIORITY
      管辖范围内，将来如需在 ISR 里调 FreeRTOS FromISR API 是安全的）*/
    NVIC_InitStructure.NVIC_IRQChannel = UART7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(UART7, USART_IT_RXNE, ENABLE);
    USART_Cmd(UART7, ENABLE);
}

/* 阻塞发送一个字节（带超时保护，避免异常时死等） */
void UART7_SendByte(uint8_t byte)
{
    uint32_t timeout = 100000;

    USART_SendData(UART7, byte);
    while (USART_GetFlagStatus(UART7, USART_FLAG_TXE) == RESET)
    {
        if (--timeout == 0) break;
    }
}

/* 阻塞发送字符串 */
void UART7_SendString(const char *str)
{
    while (*str)
    {
        UART7_SendByte((uint8_t)*str++);
    }
}

/* UART7 接收中断：逐字节喂入坐标帧状态机 */
void UART7_IRQHandler(void)
{
    if (USART_GetITStatus(UART7, USART_IT_RXNE) != RESET)
    {
        route_rx_byte((uint8_t)USART_ReceiveData(UART7));        //读DR同时清RXNE
    }

    if (USART_GetFlagStatus(UART7, USART_FLAG_ORE) != RESET)     //溢出错误清除：读SR再读DR
    {
        USART_ReceiveData(UART7);
    }
}
