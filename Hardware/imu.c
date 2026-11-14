#include "imu.h"
#include "hwt101_imu.h"
#include "lunqu_imu.h"
#include <stddef.h>

static uint8_t active_source = 0xFFu;

void IMU_Init(uint8_t source)
{
    active_source = source;
    if (source == IMU_SOURCE_HWT101) HWT101_Init();
    else if (source == IMU_SOURCE_LUNQU) Lunqu_Init(LUNQU_BAUD_RATE);
    /* 非法选择不初始化串口，配置状态FAILED，不能误用另一款启动。 */
}

uint8_t IMU_Source(void)
{
    return active_source;
}

void IMU_Poll(uint32_t now_ms)
{
    if (active_source == IMU_SOURCE_HWT101) HWT101_Poll(now_ms);
}

IMU_ConfigStatus_t IMU_ConfigStatus(void)
{
    if (active_source == IMU_SOURCE_HWT101)
    {
        HWT101_ConfigStatus_t status = HWT101_ConfigStatus();
        if (status == HWT101_CONFIG_READY) return IMU_CONFIG_READY;
        if (status == HWT101_CONFIG_PENDING) return IMU_CONFIG_PENDING;
    }
    else if (active_source == IMU_SOURCE_LUNQU)
        return Lunq_Ready() ? IMU_CONFIG_READY : IMU_CONFIG_PENDING;
    return IMU_CONFIG_FAILED;
}

float IMU_GetYaw(void)
{
    if (active_source == IMU_SOURCE_HWT101) return HWT101_GetYaw();
    if (active_source == IMU_SOURCE_LUNQU) return Lunq_GetYaw();
    return 0.0f;
}

float IMU_GetYawAbs360(void)
{
    if (active_source == IMU_SOURCE_HWT101) return HWT101_GetYawAbs360();
    if (active_source == IMU_SOURCE_LUNQU) return Lunq_GetYawAbs360();
    return 0.0f;
}

void IMU_SetYaw(float yaw_deg)
{
    if (active_source == IMU_SOURCE_HWT101) HWT101_SetYaw(yaw_deg);
    else if (active_source == IMU_SOURCE_LUNQU) Lunq_SetYaw(yaw_deg);
}

void IMU_SetZero(void)
{
    IMU_SetYaw(0.0f);
}

uint32_t IMU_UpdateSequence(void)
{
    if (active_source == IMU_SOURCE_HWT101) return HWT101_UpdateSequence();
    if (active_source == IMU_SOURCE_LUNQU) return Lunq_UpdateSequence();
    return 0;
}

void IMU_GetSample(IMU_Sample_t *out)
{
    if (out == NULL) return;
    if (active_source == IMU_SOURCE_HWT101) HWT101_GetSample(out);
    else if (active_source == IMU_SOURCE_LUNQU) Lunq_GetSample(out);
    else { out->yaw_deg = 0.0f; out->sequence = 0; }
}

/* USART6只有这一个中断入口，按上电锁存值分流，避免两驱动互抢同一串口。 */
void USART6_IRQHandler(void)
{
    if (active_source == IMU_SOURCE_HWT101) HWT101_IRQHandler();
    else if (active_source == IMU_SOURCE_LUNQU) Lunqu_IRQHandler();
}
