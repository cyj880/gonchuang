#ifndef __LUNQU_IMU_H
#define __LUNQU_IMU_H

#include "stm32f4xx.h"

/**
  ******************************************************************************
  * @file    lunqu_imu.h
  * @brief   轮趣 WHEELTEC FDI 惯导接收(逐行照抄轮趣 F103 例程, 与BNO085复用USART6/PG9)
  ******************************************************************************
  * 两个陀螺仪物理上共用同一串口(同一时刻只插一个), 由开关互斥选择:
  *   Bno085_RunFlag = 1 -> BNO085 RVC 模式  @115200 (代码在 bno08x_uart_rvc.c)
  *   lunqu_RunFlag  = 1 -> 轮趣 FDI FDILink @921600 (本文件, 例程逐行移植)
  *   两标志定义在 main.c, 改后需重新上电(波特率在初始化时确定), 只能一个为1!
  * 例程模式: 中断里完成帧缓冲(例程 UART5_IRQHandler 逐行移植, 不校验CRC),
  *           LunqTask 任务里解析(例程 TTL_Hex2Dec 等价)。
  * 偏移(例程实测): Fd_rsahrs[19..22]=Roll [23..26]=Pitch [27..30]=Heading(rad)
  ******************************************************************************
  */

extern uint8_t Bno085_RunFlag;   /*1=BNO085运行(定义在main.c)*/
extern uint8_t lunqu_RunFlag;    /*1=轮趣FDI运行(定义在main.c)*/

typedef struct
{
    float roll;        /*横滚(deg)*/
    float pitch;       /*俯仰(deg)*/
    float yaw;         /*偏航(deg; 协议为rad, 已转度)*/
    float yaw_speed;   /*偏航角速度(deg/s; 协议rad/s已转)*/
} LunquData_t;

extern LunquData_t lunqu_data;


void Lunqu_Init(uint32_t baud);   /* USART6 初始化(轮趣模式, 只收) */
void Lunq_RxByte(uint8_t b);      /* USART6中断里调用: 逐字节接收+帧缓冲+解析(全在中断内) */
float IMU_GetYaw(void);           /* 按开关返回当前陀螺仪 yaw(deg) —— 上层统一入口 */

#endif /* __LUNQU_IMU_H */
