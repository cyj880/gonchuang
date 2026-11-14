#include "stm32f4xx.h"
#include "Encoder_RS485.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/**
  ******************************************************************************
  * @file    Encoder_RS485.c
  * @brief   RS-485 绝对值编码器(欧艾迪 OID-R3806D-17S1S)驱动 —— Modbus-RTU 主站
  ******************************************************************************
  * 硬件: USART2(PD5=TX / PD6=RX, AF7, 115200-8N1) + RS485收发模块(MAX485)
  *       模块 DI<-PD5, RO->PD6, DE+RE短接->PA4(方向控制: 高=发送, 低=接收)
  *       模块 A/B -> 编码器 白(485A)/绿(485B), 仅接里程计1(站号1)
  *
  * 协议(欧艾迪手册V3.09, 标准Modbus-RTU):
  *   读虚拟多圈位置: [站号 03 00 00 00 02 CRC_L CRC_H]
  *   应答          : [站号 03 04 D3 D2 D1 D0 CRC_L CRC_H]  数据大端
  *   虚拟多圈值    : 32位连续位置, 掉电归零, 车轮每转一圈 +131072(17bit)
  *   软置零        : [站号 06 00 08 00 01 CRC_L CRC_H]
  *   CRC16         : 初值0xFFFF, 多项式0xA001(低位在前), 低字节先发
  *   站号: 仅使用 1(出厂默认)，不轮询其他站号。
  ******************************************************************************
  */

/* 方向控制脚: DE/RE短接 -> PA4(高=发送, 低=接收) */
#define RS485_DIR_PORT    GPIOA
#define RS485_DIR_PIN     GPIO_Pin_4

/* USART2中断负责发送、TC切换收发方向、缓存接收；任务只组帧和解析。
   避免任务切换拉长请求字节间隔或错过应答；所有等待有上限，不改写SysTick。 */
#define RS485_RX_BUFFER_SIZE 64u
#define RS485_REPLY_TIMEOUT_MS 40u
#define RS485_TX_TIMEOUT_MS 10u
#define RS485_RECOVER_FAILURES 3u
#define RS485_RECOVER_INTERVAL_MS 1000u
static volatile uint8_t rs485_rx_buf[RS485_RX_BUFFER_SIZE];
static volatile uint8_t rs485_rx_head, rs485_rx_tail, rs485_receiving, rs485_rx_fault;
static uint8_t rs485_tx_buf[8];
static volatile uint8_t rs485_tx_len, rs485_tx_index, rs485_tx_active;
static volatile EncRS485_Diagnostics rs485_diag;
static uint8_t rs485_read_failures, rs485_have_recovered;
static TickType_t rs485_recover_tick;
static void rs485_dir(uint8_t tx_on);

/* 在失败当刻保存寄存器和引脚配置，区分无字节与接收/IRQ被关闭。
   bit0 UE、1 RE、2 RXNEIE、3 NVIC使能、4 PD6=AF、5 PD6=AF7、6 PA4=输出。 */
static void rs485_capture_hw(void)
{
    uint32_t cr1 = USART2->CR1;
    rs485_diag.hw_flags = (uint8_t)(
        ((cr1 & USART_CR1_UE) ? 1u : 0u) |
        ((cr1 & USART_CR1_RE) ? 2u : 0u) |
        ((cr1 & USART_CR1_RXNEIE) ? 4u : 0u) |
        ((NVIC->ISER[(uint32_t)USART2_IRQn >> 5] & (1u << ((uint32_t)USART2_IRQn & 31u))) ? 8u : 0u) |
        ((((GPIOD->MODER >> 12) & 3u) == 2u) ? 16u : 0u) |
        ((((GPIOD->AFR[0] >> 24) & 15u) == 7u) ? 32u : 0u) |
        ((((GPIOA->MODER >> 8) & 3u) == 1u) ? 64u : 0u));
    rs485_diag.direction_tx = (GPIOA->ODR & GPIO_Pin_4) ? 1u : 0u;
}

void USART2_IRQHandler(void)
{
    uint32_t status = USART2->SR;
    rs485_diag.irq_calls++;
    if (status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE))
    {
        uint8_t byte = (uint8_t)USART2->DR; /*先读SR再读DR，清除RXNE/ORE/FE/NE/PE。*/
        if (status & USART_SR_RXNE) rs485_diag.rx_bytes++;
        if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE))
        {
            rs485_diag.uart_errors++;
            if (status & USART_SR_ORE) rs485_diag.overruns++;
            if (rs485_receiving) rs485_rx_fault = ENC_RS485_UART_ERROR;
        }
        else if ((status & USART_SR_RXNE) && rs485_receiving)
        {
            uint8_t next = (uint8_t)((rs485_rx_head + 1u) & (RS485_RX_BUFFER_SIZE - 1u));
            if (next == rs485_rx_tail)
            {
                rs485_diag.buffer_overflows++;
                rs485_rx_fault = ENC_RS485_RX_OVERFLOW;
            }
            else
            {
                rs485_rx_buf[rs485_rx_head] = byte;
                rs485_rx_head = next;
            }
        }
    }
    if (rs485_tx_active && (status & USART_SR_TC) && (USART2->CR1 & USART_CR1_TCIE))
    {
        USART_ITConfig(USART2, USART_IT_TC, DISABLE);
        rs485_receiving = 1;
        rs485_dir(0); /*最后停止位已移出，立刻释放485总线；不等任务重新获得CPU。*/
        rs485_diag.tx_completed++;
        rs485_tx_active = 0;
    }
    else if (rs485_tx_active && (status & USART_SR_TXE) && (USART2->CR1 & USART_CR1_TXEIE))
    {
        USART_SendData(USART2, rs485_tx_buf[rs485_tx_index++]);
        if (rs485_tx_index >= rs485_tx_len)
        {
            USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
            USART_ITConfig(USART2, USART_IT_TC, ENABLE);
        }
    }
}

void EncRS485_GetDiagnostics(EncRS485_Diagnostics *out)
{
    if (!out) return;
    taskENTER_CRITICAL();
    *out = rs485_diag;
    taskEXIT_CRITICAL();
}

/* RS485方向切换: 1=发送(DE=1/RE=1), 0=接收(DE=0/RE=0) */
static void rs485_dir(uint8_t tx_on)
{
    if (tx_on) GPIO_SetBits(RS485_DIR_PORT, RS485_DIR_PIN);
    else       GPIO_ResetBits(RS485_DIR_PORT, RS485_DIR_PIN);
}

/* Modbus CRC16(手册V3.09参考算法): 初值0xFFFF, 多项式0xA001, 低字节在前发送 */
static uint16_t modbus_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i, j;

    for (i = 0; i < len; i++)
    {
        crc ^= buf[i];
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
            else              crc >>= 1;
        }
    }
    return crc;
}

/* 事务开始时清理旧缓存；IRQ与任务只交换字节，不使用FromISR内核API。 */
static void rs485_rx_begin(void)
{
    taskENTER_CRITICAL();
    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    USART_ITConfig(USART2, USART_IT_TC, DISABLE);
    rs485_tx_active = 0;
    rs485_receiving = 0;
    rs485_rx_head = rs485_rx_tail = 0;
    rs485_rx_fault = 0;
    rs485_diag.requests++;
    rs485_diag.last_rx_len = 0;
    (void)USART2->SR;
    (void)USART2->DR;
    taskEXIT_CRITICAL();
}

static uint8_t rs485_send(const uint8_t *data, uint8_t len)
{
    TickType_t start;
    if (!len || len > sizeof(rs485_tx_buf)) return 0;
    rs485_rx_begin();
    taskENTER_CRITICAL();
    memcpy(rs485_tx_buf, data, len);
    rs485_tx_len = len;
    rs485_tx_index = 0;
    rs485_dir(1);
    rs485_tx_active = 1;
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
    taskEXIT_CRITICAL();
    start = xTaskGetTickCount();
    while (rs485_tx_active)
    {
        if (xTaskGetTickCount() - start >= pdMS_TO_TICKS(RS485_TX_TIMEOUT_MS))
        {
            taskENTER_CRITICAL();
            /*TC中断可能恰在进入临界区前完成，不能误判刚完成的发送。*/
            if (rs485_tx_active)
            {
                rs485_capture_hw();
                USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
                USART_ITConfig(USART2, USART_IT_TC, DISABLE);
                rs485_tx_active = rs485_receiving = 0;
                rs485_dir(0);
                rs485_diag.last_error = ENC_RS485_TX_TIMEOUT;
                taskEXIT_CRITICAL();
                return 0;
            }
            taskEXIT_CRITICAL();
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1u));
    }
    return 1;
}

static uint8_t rs485_pop(uint8_t *byte)
{
    if (rs485_rx_tail == rs485_rx_head) return 0;
    *byte = rs485_rx_buf[rs485_rx_tail];
    rs485_rx_tail = (uint8_t)((rs485_rx_tail + 1u) & (RS485_RX_BUFFER_SIZE - 1u));
    if (rs485_diag.last_rx_len < 255u) rs485_diag.last_rx_len++;
    return 1;
}

/* 搜索本次请求的完整帧，噪声/坏CRC后逐字节重同步；静默时让出CPU。 */
static uint8_t rs485_receive(uint8_t *frame, uint8_t len, const uint8_t *prefix, uint8_t prefix_len)
{
    TickType_t start = xTaskGetTickCount();
    uint8_t used = 0, byte;
    while (xTaskGetTickCount() - start < pdMS_TO_TICKS(RS485_REPLY_TIMEOUT_MS))
    {
        if (rs485_rx_fault) break;
        if (!rs485_pop(&byte))
        {
            vTaskDelay(pdMS_TO_TICKS(1u));
            continue;
        }
        frame[used++] = byte;
        while (used)
        {
            uint8_t compare = used < prefix_len ? used : prefix_len;
            if (memcmp(frame, prefix, compare) != 0)
            {
                --used;
                memmove(frame, frame + 1, used);
                continue;
            }
            if (used < len) break;
            {
                uint16_t crc = modbus_crc16(frame, (uint16_t)(len - 2u));
                if (!rs485_rx_fault && frame[len - 2u] == (uint8_t)crc &&
                    frame[len - 1u] == (uint8_t)(crc >> 8))
                {
                    rs485_receiving = 0;
                    rs485_diag.successes++;
                    rs485_diag.last_error = ENC_RS485_OK;
                    return 1;
                }
            }
            rs485_diag.crc_errors++;
            --used;
            memmove(frame, frame + 1, used);
        }
    }
    rs485_receiving = 0;
    rs485_capture_hw();
    rs485_diag.last_error = rs485_rx_fault ? rs485_rx_fault :
        (rs485_diag.last_rx_len ? ENC_RS485_BAD_FRAME : ENC_RS485_NO_REPLY);
    return 0;
}

/* 连续失败后的本地恢复，只由OdomTask在完整事务结束后调用。
   禁用USART2 IRQ后重建UART/引脚，清错误和缓存；不发写寄存器、置零或复位编码器命令。
   其他串口、CAN、SysTick、里程累计及通信统计均不重置，掉线保护仍生效。 */
static void rs485_read_failed(void)
{
    TickType_t now = xTaskGetTickCount();
    if (rs485_read_failures < 255u) rs485_read_failures++;
    if (rs485_read_failures < RS485_RECOVER_FAILURES ||
        (rs485_have_recovered && now - rs485_recover_tick <
         pdMS_TO_TICKS(RS485_RECOVER_INTERVAL_MS))) return;

    NVIC_DisableIRQ(USART2_IRQn);
    USART_Cmd(USART2, DISABLE);
    USART_DeInit(USART2);
    rs485_tx_active = rs485_tx_len = rs485_tx_index = 0;
    rs485_receiving = rs485_rx_fault = rs485_rx_head = rs485_rx_tail = 0;
    NVIC_ClearPendingIRQ(USART2_IRQn);
    Encoder_RS485_Init();
    rs485_diag.recoveries++;
    rs485_read_failures = 0;
    rs485_have_recovered = 1;
    rs485_recover_tick = now;
}

/**
  * 函    数：阻塞读一次虚拟多圈位置(Modbus 0x03, 寄存器0x0000~0x0001)
  * 参    数：station 编码器站号   pos 出参(32位位置值, 掉电归零)
  * 返 回 值：1=成功 0=失败(超时/站号错/功能码错/CRC错)
  * 说    明：线缆收发约1.5ms(8字节发+9字节收@115200)，另加编码器应答等待, 只能在任务上下文调用
  */
uint8_t EncRS485_ReadPos(uint8_t station, uint32_t *pos)
{
    uint8_t tx[8], rx[9];
    uint16_t crc;
    uint8_t prefix[3] = {station, 0x03u, 0x04u};

    tx[0] = station;
    tx[1] = 0x03;                        /*功能码: 读保持寄存器*/
    tx[2] = 0x00; tx[3] = 0x00;          /*起始寄存器 0x0000*/
    tx[4] = 0x00; tx[5] = 0x02;          /*寄存器数 2(32位位置值)*/
    crc = modbus_crc16(tx, 6);
    tx[6] = (uint8_t)(crc & 0xFF);       /*CRC低字节在前*/
    tx[7] = (uint8_t)(crc >> 8);

    if (!rs485_send(tx, 8u) || !rs485_receive(rx, 9u, prefix, 3u))
    {
        rs485_read_failed();
        return 0;
    }
    rs485_read_failures = 0;

    *pos = ((uint32_t)rx[3] << 24) | ((uint32_t)rx[4] << 16) |
           ((uint32_t)rx[5] << 8)  |  (uint32_t)rx[6];
    return 1;
}

/**
  * 函    数：写单个保持寄存器(Modbus 0x06): 置零/方向/波特率/站号等
  * 参    数：station 站号   reg 寄存器地址   val 写入值
  * 返 回 值：1=成功(应答与请求帧一致) 0=失败
  */
uint8_t EncRS485_WriteReg(uint8_t station, uint16_t reg, uint16_t val)
{
    uint8_t tx[8], rx[8];
    uint16_t crc;

    tx[0] = station;
    tx[1] = 0x06;                        /*功能码: 写单寄存器*/
    tx[2] = (uint8_t)(reg >> 8);  tx[3] = (uint8_t)(reg & 0xFF);
    tx[4] = (uint8_t)(val >> 8);  tx[5] = (uint8_t)(val & 0xFF);
    crc = modbus_crc16(tx, 6);
    tx[6] = (uint8_t)(crc & 0xFF);
    tx[7] = (uint8_t)(crc >> 8);

    /*0x06应答必须与请求前6字节一致，随后验证CRC。*/
    return (uint8_t)(rs485_send(tx, 8u) && rs485_receive(rx, 8u, tx, 6u));
}

/**
  * 函    数：软置零(把编码器当前位置设为0)
  * 参    数：station 站号
  * 返 回 值：1=成功 0=失败
  */
uint8_t EncRS485_SetZero(uint8_t station)
{
    return EncRS485_WriteReg(station, 0x0008, 0x0001);
}

/**
  * 函    数：把编码器报告的回绕值换算成有符号位置
  * 参    数：raw 编码器报告值(0~2^31-1)
  * 返 回 值：有符号位置(反转过零后为负, 可无歧义区分约±8192圈)
  * 说    明：虚拟多圈值按2^31回绕——反转越过零点后从上限(2^31-1)继续,
  *           报告值落在高段(≥2^30)即为负数, 减2^31还原
  */
int32_t EncRS485_ToSigned(uint32_t raw)
{
    if (raw >= 0x40000000u)                /*≥2^30: 负区(反转过零回绕)*/
        return (int32_t)(raw - 0x80000000u);
    return (int32_t)raw;
}

/**
  * 函    数：RS485编码器初始化(USART2 + 方向控制脚)
  * 参    数：无
  * 返 回 值：无
  */
void Encoder_RS485_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef irq;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOA, ENABLE);

    /*PD5=TX(复用推挽), PD6=RX(上拉输入), AF7*/
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /*PA4: RS485方向控制(DE/RE), 复位后默认低=接收*/
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_4);

    /*USART2: 115200-8-N-1, 收发(编码器设备端也须配置为115200)*/
    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);
    irq.NVIC_IRQChannel = USART2_IRQn;
    irq.NVIC_IRQChannelPreemptionPriority = 5; /*不调用RTOS API，仍遵循任务临界区屏蔽。*/
    irq.NVIC_IRQChannelSubPriority = 0;
    irq.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&irq);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART2, USART_IT_ERR, ENABLE);
    USART_ITConfig(USART2, USART_IT_PE, ENABLE);
    USART_Cmd(USART2, ENABLE);
}
