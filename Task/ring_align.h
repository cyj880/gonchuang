#ifndef RING_ALIGN_H
#define RING_ALIGN_H

#include "rdk_center.h"

typedef struct
{
    int vx, vy, w;
} RingAlignMotion;

void RingAlign_Start(uint32_t now_ms);
void RingAlign_StartMode(uint8_t mode, int8_t sign, uint32_t now_ms);
void RingAlign_Stop(void); /* preserves RAM-only yaw direction */
void RingAlign_ResetYawSign(void); /* 恢复默认yaw校正方向-1(清0x15方向覆盖) */
void RingAlign_Update(const RDK_RingResult *result, uint32_t now_ms, RingAlignMotion *out);
uint8_t RingAlign_Command(uint8_t op, int8_t value, const RDK_RingResult *result, uint32_t now_ms);
uint8_t RingAlign_Done(void);
const char *RingAlign_State(void);
uint8_t RingAlign_TestYaw(int16_t *before, int16_t *after); /* after valid only when returning 1 */

#endif
