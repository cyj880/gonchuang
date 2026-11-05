#include "belt_zdt.h"
#include "Emm_V5.h"
#include "Delay.h"

/* 底部同步带(地址5): 位置读回 + 到位回传捕获，上电位置为0，不自动回零。
   应答帧格式与丝杆(lift_zdt.c)同款: ExtId=(addr<<8)|包序, Data[0]=功能码, 尾字节0x6B。
   到位回传帧(Response=Other时): Data=[FD][9F][6B], DLC=3。 */

#define BELT_REPLY_WAIT_MS       40u    /* 位置读回等待(任务上下文) */

static volatile uint8_t reply_code;   /* 0=未武装 */
static volatile uint8_t reply_ok;
static volatile uint8_t reply_len;
static volatile uint8_t reply_buf[8];
static volatile uint8_t reached;      /* 位置模式到位回传: Data=[FD][9F][6B] */

/* CAN2接收中断里调用: 捕获地址5的应答/位置读回/到位回传帧, 不使用任何FreeRTOS API。 */
void Belt_ZDT_OnCanRx(const volatile CanRxMsg *frame)
{
    uint8_t i;

    if (frame->IDE == CAN_Id_Extended &&
        frame->ExtId == ((uint32_t)BELT_ZDT_ADDR << 8))
    if (frame->IDE != CAN_Id_Extended || frame->RTR != CAN_RTR_Data ||
        frame->ExtId != ((uint32_t)BELT_ZDT_ADDR << 8) ||
        frame->DLC < 2u || frame->DLC > 8u ||
        frame->Data[frame->DLC - 1u] != 0x6Bu)
        return;

    /* 位置模式到位回传(驱动器Response=Other时主动推送): Data=[FD][9F][6B]。 */
    if (frame->Data[0] == 0xFDu && frame->Data[1] == 0x9Fu && frame->DLC == 3u)
    {
        reached = 1;
        return;
    }

    if (reply_code != 0u && frame->Data[0] == reply_code && reply_ok == 0u)
    {
        reply_len = frame->DLC;
        for (i = 0; i < frame->DLC; ++i) reply_buf[i] = frame->Data[i];
        reply_ok = 1; /* 整帧复制完再置位 */
    }
}

void Belt_ZDT_ReachedClear(void)
{
    reached = 0;
}

uint8_t Belt_ZDT_ReachedGet(void)
{
    return reached;
}

static void belt_prepare_reply(uint8_t code)
{
    reply_code = 0;
    reply_ok = 0;
    reply_code = code;
}

/* 位置回读等待应答。1=收到且长度匹配。 */
static uint8_t belt_wait_reply(uint8_t size, uint32_t timeout_ms)
{
    uint32_t waited = 0u;

    while (!reply_ok)
    {
        Delay_ms(1);
        if (++waited >= timeout_ms) { reply_code = 0; return 0; }
    }
    reply_code = 0;
    return (reply_len == size) ? 1u : 0u;
}

/* 电机原始坐标以CW为正；按正向映射转换为与目标相同的逻辑坐标。 */
uint8_t Belt_ZDT_QueryPos(int32_t *pos)
{
    belt_prepare_reply(0x36u);
    Emm_V5_Read_Sys_Params(BELT_ZDT_ADDR, S_CPOS);
    if (!belt_wait_reply(7u, BELT_REPLY_WAIT_MS)) return 0;
    *pos = (int32_t)(((uint32_t)reply_buf[2] << 24) |
                     ((uint32_t)reply_buf[3] << 16) |
                     ((uint32_t)reply_buf[4] << 8) |
                     (uint32_t)reply_buf[5]);
    if (reply_buf[1] == 1u) *pos = -*pos; /* 符号字节: 1=负坐标 */
    if (BELT_POS_POSITIVE_DIR == 1u) *pos = -*pos;
    return 1;
}
