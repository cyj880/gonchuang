#include "stm32f4xx.h"
#include "Encoder_RS485.h"

/**
  ******************************************************************************
  * @file    Encoder_RS485.c
  * @brief   RS-485 绝对值编码器(欧艾迪 OID-R3806D-17S1S)驱动 —— Modbus-RTU 主站
  ******************************************************************************
  * 硬件: USART2(PD5=TX / PD6=RX, AF7, 9600-8N1) + RS485收发模块(MAX485)
  *       模块 DI<-PD5, RO->PD6, DE+RE短接->PA4(方向控制: 高=发送, 低=接收)
  *       模块 A/B -> 编码器 白(485A)/绿(485B), 两只编码器并联同一总线(站号1,2)
  *
  * 协议(欧艾迪手册V3.09, 标准Modbus-RTU):
  *   读虚拟多圈位置: [站号 03 00 00 00 02 CRC_L CRC_H]
  *   应答          : [站号 03 04 D3 D2 D1 D0 CRC_L CRC_H]  数据大端
  *   虚拟多圈值    : 32位连续位置, 掉电归零, 车轮每转一圈 +131072(17bit)
  *   软置零        : [站号 06 00 08 00 01 CRC_L CRC_H]
  *   CRC16         : 初值0xFFFF, 多项式0xA001(低位在前), 低字节先发
  *   站号          : 当前只有 1(出厂默认); 第二只用上位机改站号为 2 后,
  *                   把 odom_task.c 里 ODOM_ENC_NUM 改成 2、站号表加 2 即可
  ******************************************************************************
  */

/* 方向控制脚: DE/RE短接 -> PA4(高=发送, 低=接收) */
#define RS485_DIR_PORT    GPIOA
#define RS485_DIR_PIN     GPIO_Pin_4

/* 收字节超时(递减循环数, 168MHz下约10ms, 粗略值) */
#define RS485_RX_TIMEOUT  300000u

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

static void rs485_put_byte(uint8_t b)
{
    USART_SendData(USART2, b);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

/* 清残留: 上次超时中断可能留下未读字节/溢出标志 */
static void rs485_rx_flush(void)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET)
    {
        USART_ReceiveData(USART2);
    }
    if (USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET)
    {
        (void)USART_ReceiveData(USART2);       /*读SR后读DR清除ORE*/
    }
}

/**
  * 函    数：阻塞读一次虚拟多圈位置(Modbus 0x03, 寄存器0x0000~0x0001)
  * 参    数：station 编码器站号   pos 出参(32位位置值, 掉电归零)
  * 返 回 值：1=成功 0=失败(超时/站号错/功能码错/CRC错)
  * 说    明：阻塞约25ms(8字节发+9字节收@9600), 只能在任务上下文调用
  */
uint8_t EncRS485_ReadPos(uint8_t station, uint32_t *pos)
{
    uint8_t tx[8], rx[9];
    uint16_t crc;
    uint8_t i;
    uint32_t t;

    tx[0] = station;
    tx[1] = 0x03;                        /*功能码: 读保持寄存器*/
    tx[2] = 0x00; tx[3] = 0x00;          /*起始寄存器 0x0000*/
    tx[4] = 0x00; tx[5] = 0x02;          /*寄存器数 2(32位位置值)*/
    crc = modbus_crc16(tx, 6);
    tx[6] = (uint8_t)(crc & 0xFF);       /*CRC低字节在前*/
    tx[7] = (uint8_t)(crc >> 8);

    rs485_rx_flush();                    /*清上次残留*/
    rs485_dir(1);                        /*切发送*/
    for (i = 0; i < 8; i++) rs485_put_byte(tx[i]);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);  /*等最后一位移出*/
    rs485_dir(0);                        /*切接收, 释放总线给编码器应答*/

    /*收9字节应答: 站号 03 04 D3 D2 D1 D0 CRC_L CRC_H(数据大端)*/
    for (i = 0; i < 9; i++)
    {
        t = RS485_RX_TIMEOUT;
        while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET)
        {
            if (--t == 0) return 0;      /*超时: 编码器未应答*/
        }
        rx[i] = (uint8_t)USART_ReceiveData(USART2);
    }

    if (rx[0] != station || rx[1] != 0x03 || rx[2] != 0x04) return 0;
    crc = modbus_crc16(rx, 7);
    if (rx[7] != (uint8_t)(crc & 0xFF) || rx[8] != (uint8_t)(crc >> 8)) return 0;

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
    uint8_t i;
    uint32_t t;

    tx[0] = station;
    tx[1] = 0x06;                        /*功能码: 写单寄存器*/
    tx[2] = (uint8_t)(reg >> 8);  tx[3] = (uint8_t)(reg & 0xFF);
    tx[4] = (uint8_t)(val >> 8);  tx[5] = (uint8_t)(val & 0xFF);
    crc = modbus_crc16(tx, 6);
    tx[6] = (uint8_t)(crc & 0xFF);
    tx[7] = (uint8_t)(crc >> 8);

    rs485_rx_flush();
    rs485_dir(1);
    for (i = 0; i < 8; i++) rs485_put_byte(tx[i]);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
    rs485_dir(0);

    /*0x06应答=请求帧原样回显, 逐字节一致才算成功*/
    for (i = 0; i < 8; i++)
    {
        t = RS485_RX_TIMEOUT;
        while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET)
        {
            if (--t == 0) return 0;
        }
        rx[i] = (uint8_t)USART_ReceiveData(USART2);
    }
    for (i = 0; i < 8; i++)
    {
        if (rx[i] != tx[i]) return 0;
    }
    return 1;
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

    /*USART2: 9600-8-N-1, 收发(编码器出厂默认; 提速用0x0005寄存器)*/
    USART_InitStructure.USART_BaudRate            = 9600;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);
    USART_Cmd(USART2, ENABLE);
}
