#ifndef LIFT_ZDT_H
#define LIFT_ZDT_H

#include "can.h"

/* CAN2, 1 Mbps. Chassis motors already occupy addresses 1..4.
 * Set this to the lift driver's actual address before flashing. */
#define LIFT_ZDT_ADDR              6u
#define LIFT_HOME_SPEED_RPM        30u    /*回零转速50→30: 碰撞时挤进螺纹的行程减半, 减轻顶紧拧死(2026-11-08)*/
#define LIFT_HOME_TIMEOUT_MS       60000u /*30→60s: 回零降速后全程更久(保留原回零超时), 超时必须跟着放大(2026-11-08)*/
#define LIFT_STALL_SPEED_RPM       10u
#define LIFT_STALL_CURRENT_MA      250u   /*400→250: 更早判停顶得更轻, 防碰撞后拧紧; 若未到顶就误判堵转(回零failed)回调到300-350(2026-11-08)*/
#define LIFT_STALL_TIME_MS         100u
/* 位置读回单位=驱动器细分脉冲(手册默认16细分=3200脉冲/转)；允许编码器抖动。 */
#define LIFT_ZERO_TOLERANCE        4u

/* UART7 0x17 调试帧：张大头位置模式(0xFD)绝对目标。clk为无符号幅值，方向由dir
   字节给出：绝对目标=±clk。帧值=离开回零位(最高点)的下行行程(0=最高、63775=最低)；
   实车方向反了改 LIFT_POS_DOWN_DIR。 */
/* 手册5.3.12(Emm固件0xFD)：速度字段0000-0BB8=0-3000RPM。
 * 丝杆正常定位速度统一3000RPM，自动流程及独立0x17调试共用。
 * 加速度0-255档，每(256-acc)*50us加减1RPM → 当前acc=248为400us/RPM(2500RPM/s)，
 * acc=255为曲线最快档；acc=0为不加减速直接启动。竖直丝杆负荷重，高速更易
 * 触发堵转保护(屏显Protect!)，先降档再降速。 */
#define LIFT_POS_SPEED_RPM         3000u
#define LIFT_POS_ACC               248u   /* 正常定位加速度档位：255→248；自动流程及0x17共用 */
#define LIFT_POS_MAX_PUL           63775u /* 新丝杆全行程上限：原255100/4；坐标0=回零最高点 */
#define LIFT_POS_DOWN_DIR          1u      /* 下行dir字节: 0=CW(向上) 1=CCW(向下) */

typedef enum {
    LIFT_HOME_OK = 0,
    LIFT_HOME_BAD_ADDRESS,
    LIFT_HOME_COMM_ERROR,
    LIFT_HOME_REJECTED,
    LIFT_HOME_NOT_READY,
    LIFT_HOME_FAILED,
    LIFT_HOME_TIMEOUT,
    LIFT_HOME_ZERO_ERROR,
    LIFT_HOME_STOP_ERROR
} LiftHomeResult_t;

typedef struct {
    uint8_t command;
    uint8_t reply;
    uint8_t length;
    uint8_t origin_flags;
    uint8_t motor_flags;
} LiftHomeDiag_t;

/* 丝杆上电自动回零开关(定义在main.c，默认0): 1=开机阻塞回零,失败卡死不开机; 0=跳过回零直接开机。
   跳过后丝杆位置未知,执行升降/定位动作前需先手动回零或重新置零。 */
extern uint8_t Lift_AutoHome_RunFlag;

/* Frozen on failure; cleanup commands do not overwrite the failing reply. */
extern volatile LiftHomeDiag_t g_lift_home_diag;

/* Blocking startup operation: CAN2 + Delay_Init must be ready, interrupts
 * enabled, scheduler NOT started. CW/up, sensorless home, then position zero.
 * On failure, attempts homing interrupt + stop; never declares a new zero. */
LiftHomeResult_t Lift_ZDT_HomeBeforeScheduler(uint8_t addr);

/* Called by CAN2 RX ISR. Only captures the currently awaited lift reply. */
void Lift_ZDT_OnCanRx(const volatile CanRxMsg *frame);

/* 位置模式到位回传(驱动器Response=Other时): 发命令前Clear, 等待中Get。 */
void Lift_ZDT_ReachedClear(void);
uint8_t Lift_ZDT_ReachedGet(void);

/* Task-context position readback (blocking, up to 40ms). 1=ok. */
uint8_t Lift_ZDT_QueryPos(int32_t *pos);

#endif
