#ifndef ARM_M2006_H
#define ARM_M2006_H

#include <stdint.h>

/* C610 ID 1, CAN1 (PD0=RX, PD1=TX), 1 Mbps, M2006 P36 output shaft. */
#define ARM_M2006_MOTOR_ID       1u
#define ARM_M2006_EXTEND_DEG     120.0f
#define ARM_M2006_DIRECTION      1.0f
#define ARM_M2006_MOVE_TIMEOUT_MS 10000u
#define ARM_M2006_MIN_TARGET_DEG 0        /* 负方向机械堵转, 禁止负目标 */
#define ARM_M2006_MAX_TARGET_DEG 500

void Arm_M2006_Init(void);
void Arm_M2006_Task(void *argument);
uint8_t Arm_M2006_Extend(void); /* Absolute +120 degrees from power-on zero. */
uint8_t Arm_M2006_SetTargetDeg(int16_t degrees); /* Manual absolute target from power-on zero. */
uint8_t Arm_M2006_Ready(void);
uint8_t Arm_M2006_AtTarget(void);
uint8_t Arm_M2006_AtTargetWithin(float tolerance_deg); /* 到位且基本停稳, 可指定角度容差 */
void Arm_M2006_Stop(void);      /* Zero current; no automatic retraction. */

#endif
