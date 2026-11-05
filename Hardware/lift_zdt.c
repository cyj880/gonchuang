#include "lift_zdt.h"
#include "Emm_V5.h"
#include "Delay.h"
#include "FreeRTOS.h"
#include "task.h"

#define LIFT_REPLY_TIMEOUT_MS  200u
#define LIFT_POLL_MS           10u
#define ORIGIN_READY           0x03u
#define ORIGIN_RUNNING         0x04u
#define ORIGIN_FAULTS          0x38u /* Home failed, overheat or overcurrent. */
#define MOTOR_ENABLED          0x01u
#define MOTOR_STALL_PROTECT    0x08u
#define LIFT_STOP_TIMEOUT_MS   1000u

static uint8_t lift_addr;
static volatile uint8_t reply_code;
static volatile uint8_t reply_ready;
static volatile uint8_t reply_size;
static volatile uint8_t reply_data[8];
static volatile uint8_t reached;   /*位置模式到位回传: Data=[FD][9F][6B]*/
static uint32_t clock_last, clock_ms;
volatile LiftHomeDiag_t g_lift_home_diag;

/* DWT gives an independent startup deadline, including CAN transmit/wait
 * time. Sample frequently so CYCCNT wrapping (about 25 s) is harmless. */
static uint32_t lift_now_ms(void)
{
    uint32_t cycles_per_ms = SystemCoreClock / 1000u;
    uint32_t elapsed = (DWT->CYCCNT - clock_last) / cycles_per_ms;
    clock_last += elapsed * cycles_per_ms;
    clock_ms += elapsed;
    return clock_ms;
}

void Lift_ZDT_OnCanRx(const volatile CanRxMsg *frame)
{
    uint8_t i;
    if (frame->IDE != CAN_Id_Extended || frame->RTR != CAN_RTR_Data ||
        frame->ExtId != ((uint32_t)lift_addr << 8) ||
        frame->DLC < 2 || frame->DLC > 8 ||
        frame->Data[frame->DLC - 1] != 0x6B)
        return;

    /* 位置模式到位回传(驱动器Response=Other时主动推送): Data=[FD][9F][6B]。 */
    if (frame->Data[0] == 0xFDu && frame->Data[1] == 0x9Fu && frame->DLC == 3u)
    {
        reached = 1;
        return;
    }

    if (reply_code == 0 || reply_ready ||
        frame->Data[0] != reply_code)
        return;

    reply_size = frame->DLC;
    for (i = 0; i < frame->DLC; ++i) reply_data[i] = frame->Data[i];
    reply_ready = 1; /* Publish only after the whole reply is copied. */
}

void Lift_ZDT_ReachedClear(void)
{
    reached = 0;
}

uint8_t Lift_ZDT_ReachedGet(void)
{
    return reached;
}

static void lift_prepare_reply(uint8_t code)
{
    /* Disarm first; an unrelated/late frame must not overwrite our reply. */
    reply_code = 0;
    reply_ready = 0;
    g_lift_home_diag.command = code;
    g_lift_home_diag.reply = 0;
    g_lift_home_diag.length = 0;
    reply_code = code;
}

static LiftHomeResult_t lift_wait_reply(uint8_t size, uint32_t timeout_ms)
{
    uint32_t start = lift_now_ms();
    while (!reply_ready)
    {
        if (lift_now_ms() - start >= timeout_ms)
        {
            reply_code = 0;
            return LIFT_HOME_COMM_ERROR;
        }
        Delay_ms(1);
    }
    reply_code = 0;
    g_lift_home_diag.length = reply_size;
    g_lift_home_diag.reply = reply_data[1];
    if (reply_size != size) return LIFT_HOME_REJECTED;
    return LIFT_HOME_OK;
}

/* Reuse the supplied Emm driver, and check each motor command's ACK.
 * A CAN bus ACK alone does not mean the addressed motor accepted a command. */
static LiftHomeResult_t lift_command(uint8_t code)
{
    LiftHomeResult_t result;
    lift_prepare_reply(code);
    switch (code)
    {
        case 0x9C: Emm_V5_Origin_Interrupt(lift_addr); break;
        case 0xFE: Emm_V5_Stop_Now(lift_addr, false); break;
        case 0x0E: Emm_V5_Reset_Clog_Pro(lift_addr); break;
        case 0xF3: Emm_V5_En_Control(lift_addr, true, false); break;
        case 0x4C:
            /* RAM only, CW, sensorless; MCU explicitly starts each home. */
            Emm_V5_Origin_Modify_Params(lift_addr, false, 2, 0,
                LIFT_HOME_SPEED_RPM, LIFT_HOME_TIMEOUT_MS,
                LIFT_STALL_SPEED_RPM, LIFT_STALL_CURRENT_MA,
                LIFT_STALL_TIME_MS, false);
            break;
        case 0x9A: Emm_V5_Origin_Trigger_Return(lift_addr, 2, false); break;
        case 0x0A: Emm_V5_Reset_CurPos_To_Zero(lift_addr); break;
        default: reply_code = 0; return LIFT_HOME_REJECTED;
    }
    /* Reached response mode can return only 9A 9F after motion completes. */
    result = lift_wait_reply(3, code == 0x9A ?
        LIFT_HOME_TIMEOUT_MS + 1000u : LIFT_REPLY_TIMEOUT_MS);
    if (result != LIFT_HOME_OK) return result;
    /* These are documented homing responses, not generic command ACKs.
     * Always verify fresh origin flags before stopping/clearing coordinates.
     * Response=Other时0x9A可能立即回02(命令确认): 视为触发已接受,
     * 回零完成仍由后续0x3B OFLAG轮询判定。 */
    if (code == 0x9A &&
        (reply_data[1] == 0x02 || reply_data[1] == 0x12 || reply_data[1] == 0x9F))
        return LIFT_HOME_OK;
    return reply_data[1] == 0x02 ? LIFT_HOME_OK : LIFT_HOME_REJECTED;
}

static LiftHomeResult_t lift_read(uint8_t code, SysParams_t param, uint8_t size)
{
    lift_prepare_reply(code);
    Emm_V5_Read_Sys_Params(lift_addr, param);
    return lift_wait_reply(size, LIFT_REPLY_TIMEOUT_MS);
}

/* 任务上下文读取丝杆当前位置(带符号脉冲, 上电清零起算)。
   返回1=成功; 0=应答超时。仅夹取序列等运行期流程使用, 不动回零诊断状态。 */
uint8_t Lift_ZDT_QueryPos(int32_t *pos)
{
    uint32_t waited = 0u;

    lift_prepare_reply(0x36u);
    Emm_V5_Read_Sys_Params(lift_addr, S_CPOS);
    while (!reply_ready)
    {
        /* 运行期查询必须让出CPU, 不能使用会改写并关闭SysTick的Delay_ms。 */
        vTaskDelay(pdMS_TO_TICKS(2u));
        waited += 2u;
        if (waited >= 40u) { reply_code = 0; return 0; }
    }
    reply_code = 0;
    if (reply_size != 7u) return 0;
    *pos = (int32_t)(((uint32_t)reply_data[2] << 24) |
                     ((uint32_t)reply_data[3] << 16) |
                     ((uint32_t)reply_data[4] << 8) |
                     (uint32_t)reply_data[5]);
    if (reply_data[1] == 1u) *pos = -*pos; /* 符号字节: 1=负坐标 */
    return 1;
}

static LiftHomeResult_t lift_wait_stopped(void)
{
    LiftHomeResult_t result;
    uint32_t start = lift_now_ms();
    uint16_t speed;
    uint8_t stable = 0;

    /* Native sensorless homing already stops the motor. Verify that fact
     * instead of requiring another FE command, which can return E2 here. */
    while (lift_now_ms() - start < LIFT_STOP_TIMEOUT_MS)
    {
        result = lift_read(0x35, S_VEL, 5);
        if (result != LIFT_HOME_OK) return result;
        if (reply_data[1] > 1) return LIFT_HOME_STOP_ERROR;
        speed = ((uint16_t)reply_data[2] << 8) | reply_data[3];
        stable = speed <= 2u ? stable + 1u : 0u;
        if (stable >= 3u) return LIFT_HOME_OK;
        Delay_ms(20);
    }
    return LIFT_HOME_STOP_ERROR;
}

static LiftHomeResult_t lift_finish_home(void)
{
    LiftHomeResult_t result;
    uint8_t flags;

    result = lift_wait_stopped();
    if (result != LIFT_HOME_OK) return result;
    result = lift_read(0x3A, S_FLAG, 3);
    if (result != LIFT_HOME_OK) return result;
    flags = reply_data[1];
    g_lift_home_diag.motor_flags = flags;

    /* A successful origin status does not include the separate stall
     * protection latch. Recover it once, only after confirmed homing. */
    if (flags & MOTOR_STALL_PROTECT)
    {
        result = lift_command(0x0E);
        if (result != LIFT_HOME_OK) return result;
    }
    if (!(flags & MOTOR_ENABLED) || (flags & MOTOR_STALL_PROTECT))
    {
        result = lift_command(0xF3);
        if (result != LIFT_HOME_OK) return result;
        Delay_ms(20);
        result = lift_wait_stopped();
        if (result != LIFT_HOME_OK) return result;
    }

    result = lift_read(0x3B, S_OFLAG, 3);
    if (result != LIFT_HOME_OK) return result;
    flags = reply_data[1];
    g_lift_home_diag.origin_flags = flags;
    if ((flags & ORIGIN_READY) != ORIGIN_READY ||
        (flags & (ORIGIN_RUNNING | ORIGIN_FAULTS)))
        return LIFT_HOME_FAILED;

    result = lift_read(0x3A, S_FLAG, 3);
    if (result != LIFT_HOME_OK) return result;
    flags = reply_data[1];
    g_lift_home_diag.motor_flags = flags;
    if (!(flags & MOTOR_ENABLED) || (flags & MOTOR_STALL_PROTECT))
        return LIFT_HOME_NOT_READY;
    return LIFT_HOME_OK;
}

static void lift_abort(void)
{
    uint8_t i;
    reply_code = 0;
    /* Keep holding torque on a vertical screw. Do not disable the driver. */
    for (i = 0; i < 2; ++i)
    {
        Emm_V5_Origin_Interrupt(lift_addr);
        Delay_ms(20);
        Emm_V5_Stop_Now(lift_addr, false);
        Delay_ms(20);
    }
}

LiftHomeResult_t Lift_ZDT_HomeBeforeScheduler(uint8_t addr)
{
    static const uint8_t setup_commands[] = {0x9C, 0x0E, 0xF3, 0xFE, 0x4C};
    LiftHomeResult_t result;
    uint32_t start, position;
    uint32_t waited = 0u;
    uint8_t i, flags;

    /* Never use the broadcast address or an existing chassis address. */
    if (addr <= 4) return LIFT_HOME_BAD_ADDRESS;
    lift_addr = addr;
    reply_code = 0;
    reply_ready = 0;
    g_lift_home_diag.command = 0;
    g_lift_home_diag.reply = 0;
    g_lift_home_diag.length = 0;
    g_lift_home_diag.origin_flags = 0;
    g_lift_home_diag.motor_flags = 0;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    clock_last = DWT->CYCCNT;
    clock_ms = 0;

    Delay_ms(500); /* Wait for the motor driver to finish powering up. */
    for (i = 0; i < sizeof(setup_commands); ++i)
    {
        result = lift_command(setup_commands[i]);
        if (result != LIFT_HOME_OK) goto failed;
        Delay_ms(10);
    }

    result = lift_read(0x3B, S_OFLAG, 3);
    if (result != LIFT_HOME_OK) goto failed;
    g_lift_home_diag.origin_flags = reply_data[1];
    if ((reply_data[1] & ORIGIN_READY) != ORIGIN_READY ||
        (reply_data[1] & 0x30u) != 0)
    {
        result = LIFT_HOME_NOT_READY;
        goto failed;
    }

    /* 预移动: 若上一轮丝杆停在上位限位处, 直接CW回零会"未移动即堵转",
       被驱动器判为回零失败(Org_CF, 屏显failed:5/ORG:0B)。先CCW向下退
       2000脉冲离开顶部, 再触发回零。预移动未确认到位也不阻塞, 继续回零。 */
    Lift_ZDT_ReachedClear();
    Emm_V5_Pos_Control(addr, LIFT_POS_DOWN_DIR, 500u, 200u,
                       LIFT_HOME_RETREAT_PUL, false, false);
    while (!Lift_ZDT_ReachedGet() && waited < LIFT_HOME_RETREAT_WAIT_MS)
    { Delay_ms(10); waited += 10; }
    if (!Lift_ZDT_ReachedGet())
    {
        /* 预移动未确认(如已在下限位堵转): 解堵转+重新使能后照常回零。 */
        lift_prepare_reply(0x0E);
        Emm_V5_Reset_Clog_Pro(lift_addr);
        (void)lift_wait_reply(3, LIFT_REPLY_TIMEOUT_MS);
        lift_prepare_reply(0xF3);
        Emm_V5_En_Control(lift_addr, true, false);
        (void)lift_wait_reply(3, LIFT_REPLY_TIMEOUT_MS);
        Delay_ms(20);
    }
    Lift_ZDT_ReachedClear();

    result = lift_command(0x9A);
    if (result != LIFT_HOME_OK) goto failed;
    start = lift_now_ms();
    /* A fresh status is meaningful only after the trigger was acknowledged.
     * Allow a driver already at the top to finish before our first poll. */
    Delay_ms(LIFT_STALL_TIME_MS + 20u);
    for (;;)
    {
        if (lift_now_ms() - start >= LIFT_HOME_TIMEOUT_MS + 1000u)
        {
            result = LIFT_HOME_TIMEOUT;
            goto failed;
        }
        result = lift_read(0x3B, S_OFLAG, 3);
        if (result != LIFT_HOME_OK) goto failed;
        flags = reply_data[1];
        g_lift_home_diag.origin_flags = flags;
        if ((flags & ORIGIN_READY) != ORIGIN_READY || (flags & ORIGIN_FAULTS))
        {
            result = LIFT_HOME_FAILED;
            goto failed;
        }
        if (!(flags & ORIGIN_RUNNING)) break;
        Delay_ms(LIFT_POLL_MS);
    }

    result = lift_finish_home();
    if (result != LIFT_HOME_OK) goto failed;
    result = lift_command(0x0A); /* Clear motor coordinates, not single-turn O. */
    if (result != LIFT_HOME_OK) goto failed;
    Delay_ms(20);
    result = lift_read(0x36, S_CPOS, 7);
    if (result != LIFT_HOME_OK) goto failed;
    position = ((uint32_t)reply_data[2] << 24) |
               ((uint32_t)reply_data[3] << 16) |
               ((uint32_t)reply_data[4] << 8) | reply_data[5];
    if (reply_data[1] > 1 || position > LIFT_ZERO_TOLERANCE)
    {
        result = LIFT_HOME_ZERO_ERROR;
        goto failed;
    }
    return LIFT_HOME_OK;

failed:
    lift_abort();
    return result;
}
