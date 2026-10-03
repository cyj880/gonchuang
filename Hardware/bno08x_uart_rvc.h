/**
  ******************************************************************************
  * @file    bno08x_uart_rvc.h
  * @brief   BNO085 陀螺仪 UART RVC 模式驱动（移植自 mspm0-modules 例程）
  ******************************************************************************
  * 移植说明（mspm0 例程 -> STM32F427 标准外设库 + FreeRTOS）：
  *   1. 接口：USART6，PG9 = USART6_RX（AF8），只接收；接 BNO085 的 TX
  *   2. 波特率 115200-8-N-1，BNO085 需刷成 CE Sensor Streamer 固件的 RVC 模式
 *   3. 接收：USART6 RXNE 中断逐字节收 + 帧头滑窗状态机
 *      （帧头 0xAA 0xAA 对齐，错位/丢字节最多丢一帧自动恢复，
 *        中断优先级 4 —— 必须 <5，否则被 FreeRTOS 临界区 BASEPRI=5 屏蔽）
  *   4. 帧格式（Datasheet Figure 1-25，100Hz）：AA AA | Index(1B) | yaw(2B)
  *              | pitch(2B) | roll(2B) | accel(6B) | 保留(3B) | checksum(1B)
  *              yaw/pitch/roll 为 0.01° 单位 int16，小端（LSB 在前）
  *   5. 解析与 mspm0 例程逐字段一致（校验和累加字节 2..14）
  ******************************************************************************
  */

#ifndef __BNO08X_UART_RVC_H
#define __BNO08X_UART_RVC_H

#include "stm32f4xx.h"

typedef struct {
    uint8_t index;
    float pitch;
    float roll;
    float yaw;
    int16_t ax;
    int16_t ay;
    int16_t az;
} BNO08X_Data_t;

extern BNO08X_Data_t bno08x_data;

void BNO08X_Init(uint32_t baud);

#endif /* __BNO08X_UART_RVC_H */
