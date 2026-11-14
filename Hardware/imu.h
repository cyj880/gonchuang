#ifndef IMU_H
#define IMU_H

#include "stm32f4xx.h"

enum { IMU_SOURCE_HWT101 = 0, IMU_SOURCE_LUNQU = 1 };
/* 在main.c修改，重新编译上电后生效；运行中修改不会切换串口或航向源。 */
extern uint8_t IMU_Select;

typedef enum
{
    IMU_CONFIG_PENDING = 0,
    IMU_CONFIG_READY,
    IMU_CONFIG_FAILED
} IMU_ConfigStatus_t;

typedef struct
{
    float yaw_deg;
    uint32_t sequence; /* 与角度对应的新帧序号，软件记零不增加序号。 */
} IMU_Sample_t;

void IMU_Init(uint8_t source); /* main上电调用一次，锁存选择；两款共用USART6/PG9。 */
uint8_t IMU_Source(void);      /* 实际初始化的源，不直接读取运行中可能被修改的选择变量。 */
void IMU_Poll(uint32_t now_ms);
IMU_ConfigStatus_t IMU_ConfigStatus(void);
float IMU_GetYaw(void);        /* (-180,180]，保留原车方向及软件零点语义。 */
float IMU_GetYawAbs360(void);  /* [0,360) */
void IMU_SetYaw(float yaw_deg);
void IMU_SetZero(void);
uint32_t IMU_UpdateSequence(void);
void IMU_GetSample(IMU_Sample_t *out); /* 原子读取角度/序号；不检查运行期断流。 */

#endif
