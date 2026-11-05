#ifndef BELT_ZDT_H
#define BELT_ZDT_H

#include "can.h"

/* CAN2地址5，Emm 0xFD绝对位置。上电位置=0，不自动回零；
   中心点7050，抓取点1400，范围0~10000。仅中心标定+50，其余绝对坐标不偏移。
   坐标增大为CW，减小为CCW。21120脉冲=一圈。 */
#define BELT_ZDT_ADDR              5u
#define BELT_POS_MAX_PUL           10000u
#define BELT_POS_CENTER_PUL        7050   /* 当前中心标定位 */
#define BELT_POS_GRAB_PUL          1400   /* 原抓取坐标-5600 */
#define BELT_POS_POSITIVE_DIR      0u     /* 正绝对目标的dir字节: 0=CW；非本次运动方向 */
#define BELT_POS_SPEED_RPM         300u   /* 电机端转速 */
#define BELT_POS_ACC               200u
void Belt_ZDT_OnCanRx(const volatile CanRxMsg *frame); /* CAN2 RX中断里调用 */

/* 位置模式到位回传(驱动器Response=Other时): 发命令前Clear, 等待中Get。 */
void Belt_ZDT_ReachedClear(void);
uint8_t Belt_ZDT_ReachedGet(void);

uint8_t Belt_ZDT_QueryPos(int32_t *pos); /* 读回同一逻辑坐标，阻塞至多40ms，1=成功 */

#endif
