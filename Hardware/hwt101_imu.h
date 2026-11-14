#ifndef HWT101_IMU_H
#define HWT101_IMU_H

#include "imu.h"

/* HWT101 TTL -> USART6：PG9(RX)接模块TX，115200-8N1；模块RX无需连接。
   100Hz已由上位机配置；只接收，首个有效角度帧后就绪，不自动配置。
   原路线约定yaw正方向为俯视车体顺时针。水平安装、Z轴朝上时，
   将维特右手系Z角度取反；若反面安装，应先实测方向后改为+1。
   方向只在驱动入口转换，不能同时修改电机任务的YAW/TURN_OUT_SIGN。 */
#define HWT101_BAUD_RATE    115200u
#define HWT101_YAW_SIGN     (-1.0f)

typedef enum
{
    HWT101_CONFIG_PENDING = 0,
    HWT101_CONFIG_READY,
    HWT101_CONFIG_FAILED
} HWT101_ConfigStatus_t;

void HWT101_Init(void);
void HWT101_RxByte(uint8_t byte); /* 仅由USART6 ISR调用，便于协议回放验证。 */
HWT101_ConfigStatus_t HWT101_ConfigStatus(void);

/* 航向后端接口，上层通过imu.h的IMU_*统一入口调用。 */
float HWT101_GetYaw(void);
float HWT101_GetYawAbs360(void);
void HWT101_SetYaw(float yaw_deg);
void HWT101_SetZero(void);
uint32_t HWT101_UpdateSequence(void);
void HWT101_GetSample(IMU_Sample_t *out);
void HWT101_IRQHandler(void); /* 仅由imu.c统一USART6入口调用。 */

#endif
