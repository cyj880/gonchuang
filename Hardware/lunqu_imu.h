#ifndef LUNQU_IMU_H
#define LUNQU_IMU_H

#include "imu.h"

/* 保留原轮趣FDILink硬件及波特率：模块TX接PG9/USART6_RX，仅接收。 */
#define LUNQU_BAUD_RATE 921600u

typedef struct
{
    float roll, pitch, yaw, yaw_abs360, yaw_speed;
} LunquData_t;

extern LunquData_t lunqu_data;
void Lunqu_Init(uint32_t baud);
void Lunqu_IRQHandler(void); /* 仅由统一USART6入口调用。 */
void Lunq_RxByte(uint8_t byte);
uint8_t Lunq_Ready(void); /* 收到过有效AHRS航向，不受软件记零影响。 */
void Lunq_SetYaw(float yaw_deg);
void Lunq_SetZero(void);
float Lunq_GetYaw(void);
float Lunq_GetYawAbs360(void);
uint32_t Lunq_UpdateSequence(void);
void Lunq_GetSample(IMU_Sample_t *out);

#endif
