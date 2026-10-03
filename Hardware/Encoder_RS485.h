#ifndef __ENCODER_RS485_H
#define __ENCODER_RS485_H

#include "stm32f4xx.h"

/**
  ******************************************************************************
  * @file    Encoder_RS485.h
  * @brief   RS-485 绝对值编码器(欧艾迪 OID-R3806D-17S1S)驱动 —— Modbus-RTU
  ******************************************************************************
  * 硬件: USART2(PD5=TX / PD6=RX, AF7, 9600-8N1) + RS485收发模块(MAX485)
  *       模块 DI<-PD5, RO->PD6, DE+RE短接->PA4(方向控制: 高=发送, 低=接收)
  *       模块 A/B -> 编码器 白(485A)/绿(485B), 两只编码器并联同一总线(站号1,2)
  *
  * 协议(欧艾迪手册V3.09, 标准Modbus-RTU):
  *   读虚拟多圈位置: [站号 03 00 00 00 02 CRC_L CRC_H]
  *   应答          : [站号 03 04 D3 D2 D1 D0 CRC_L CRC_H]  数据大端
  *   虚拟多圈值    : 0~2^31-1 回绕型(反转过零后从上限继续), 掉电归零,
  *                   车轮每转一圈 +131072(17bit), 用 EncRS485_ToSigned 换算有符号
  *   软置零        : [站号 06 00 08 00 01 CRC_L CRC_H]
  *   CRC16         : 初值0xFFFF, 多项式0xA001(低位在前), 低字节先发
  *
  * 站号: 当前只有 1(出厂默认); 第二只编码器用上位机把站号改成 2 后,
  *       把 odom_task.c 里 ODOM_ENC_NUM 改成 2、站号表加 2 即可。
  ******************************************************************************
  */

#define ENC_RS485_DEFAULT_STATION   1     /* 当前总线上的编码器站号 */

/* 阻塞读一次虚拟多圈位置(含收发与超时, 约25ms; 失败返回0)
   station: 编码器站号  pos: 出参(32位位置值) */
uint8_t EncRS485_ReadPos(uint8_t station, uint32_t *pos);

/* 写单寄存器(0x06): 置零(0x0008=1)/方向(0x0009)/波特率(0x0005)等 */
uint8_t EncRS485_WriteReg(uint8_t station, uint16_t reg, uint16_t val);

/* 回绕值 -> 有符号位置: 反转过零后报告值落在高段(≥2^30), 减2^31还原为负
   可无歧义区分的范围约 ±8192 圈, 每局上电/置零后远够用 */
int32_t EncRS485_ToSigned(uint32_t raw);

void Encoder_RS485_Init(void);        /* USART2 + PA4 方向脚初始化 */

#endif /* __ENCODER_RS485_H */
