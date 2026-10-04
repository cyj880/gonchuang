#ifndef __LUNQU_IMU_H
#define __LUNQU_IMU_H

#include "stm32f4xx.h"

/**
  ******************************************************************************
  * @file    lunqu_imu.h
  * @brief   轮趣 WHEELTEC FDI 惯导接收(例程帧缓冲方式, 与BNO085复用USART6/PG9)
  ******************************************************************************
  * 两个陀螺仪物理上共用同一串口(同一时刻只插一个), 由开关互斥选择:
  *   Bno085_RunFlag = 1 -> BNO085 RVC 模式  @115200 (代码在 bno08x_uart_rvc.c)
  *   lunqu_RunFlag  = 1 -> 轮趣 FDI FDILink @921600 (本文件)
  *   两标志定义在 main.c, 改后需重新上电(波特率在初始化时确定), 只能一个为1!
  * 接收与解析都在 USART6 中断内完成(帧缓冲/对齐按轮趣例程, 不校验CRC)。
  * 偏移(例程实测): Fd_rsahrs[19..22]=Roll [23..26]=Pitch [27..30]=Heading(rad)
  * 零点机制: 上电首帧自动记零, PB2(key_task)可重新记零 —— yaw 输出相对零点角。
  ******************************************************************************
  */

extern uint8_t Bno085_RunFlag;   /*1=BNO085运行(定义在main.c)*/
extern uint8_t lunqu_RunFlag;    /*1=轮趣FDI运行(定义在main.c)*/

typedef struct
{
    float roll;        /*横滚(deg)*/
    float pitch;       /*俯仰(deg)*/
    float yaw;         /*偏航(deg): 相对零点±180(闭环用); 零点=上电/按PB2时刻朝向*/
    float yaw_abs360;  /*偏航(deg): 相对零点0~360(LCD显示用)*/
    float yaw_speed;   /*偏航角速度(deg/s)*/
} LunquData_t;

extern LunquData_t lunqu_data;

void Lunqu_Init(uint32_t baud);   /* USART6 初始化(轮趣模式, 只收) */
void Lunq_RxByte(uint8_t b);      /* USART6中断里调用: 逐字节接收+帧缓冲+解析(全在中断内) */
void Lunq_SetZero(void);          /* 重新记零点: 当前朝向=0°(key_task PB2调用) */
float IMU_GetYaw(void);           /* 闭环统一入口: 相对零点±180(连续) */
float IMU_GetYawAbs360(void);     /* LCD显示: 相对零点0~360 */
void IMU_SetZero(void);           /* PB2统一入口: 轮趣记零点, BNO085空操作 */

#endif /* __LUNQU_IMU_H */
