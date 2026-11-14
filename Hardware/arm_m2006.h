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

typedef struct
{
    float angle_deg, target_deg;
    int16_t rpm, current_command; /*转子rpm；发给C610的电流指令，不是实测电流。*/
    uint8_t ready;
} Arm_M2006_Diagnostics;
void Arm_M2006_GetDiagnostics(Arm_M2006_Diagnostics *out); /*只读快照，不发送CAN查询。*/

void Arm_M2006_Init(void);
void Arm_M2006_Task(void *argument);
uint8_t Arm_M2006_Extend(void); /* Absolute +120 degrees from power-on zero. */
uint8_t Arm_M2006_SetTargetDeg(int16_t degrees); /*上电零点绝对角度；换目标沿用速度/保持力，同目标不重置。*/
uint8_t Arm_M2006_Ready(void);
uint8_t Arm_M2006_AtTarget(void);
float Arm_M2006_GetAngle(void);      /* 当前输出轴角度(deg, 上电零点, LCD诊断用) */
float Arm_M2006_GetTargetDeg(void);  /* 当前位置目标(deg) */
uint8_t Arm_M2006_GetReady(void);    /* 反馈有效性(1=CAN反馈正常) */
uint8_t Arm_M2006_AtTargetWithin(float tolerance_deg); /*角度进指定容差且转子速度<=36rpm；掠过不算到位。*/
void Arm_M2006_Stop(void);      /* Zero current; no automatic retraction. */

#endif
