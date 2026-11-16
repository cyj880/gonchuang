#include "motor_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "Emm_V5.h"
#include "route_rx.h"
#include "pid.h"
#include "odom_task.h"
#include "imu.h"
#include "uart8.h"
#include "Buzzer.h"
#include "rdk_center.h"
#include "ring_align.h"
#include "arm_m2006.h"
#include "lift_zdt.h"
#include "belt_zdt.h"
#include "Servo.h"

/**
  * 四路 Emm_V5 步进闭环电机控制任务（CAN2，1Mbps）
  * ------------------------------------------------------------------
  * 参考例程：gcs-gold-medal-main/yyb_stm32/can_ZDT/yyb_move.c
  *   car_move(v_x, v_y, w)  —— 麦轮运动学 + 四轮速度模式
  * 本任务全部使用**速度模式**（Emm_V5_Vel_Control，功能码 0xF6）：
  *   直行 / 平移 / 45° 斜走 / 原地自转都只是 vx、vy、w 的不同组合，
  *   位置模式（Emm_V5_Pos_Control）仅作为备用接口保留，任务里不调用。
  *
  * 与例程的差异（移植要点）：
  *   1. 例程每帧之间 delay_ms(1) 空过 CAN，这里改为 Emm_V5 的"同步标志 + 广播帧"，
  *      四轮指令全部缓存后一次性广播，真正同时起步（也可由 MOTOR_SYNC_START 改回逐条下发）；
  *   2. 例程在裸机 while(1) 里跑，这里延时统一 vTaskDelay(pdMS_TO_TICKS(x))，
  *      状态机使用RTOS tick/vTaskDelay，不用忙等；Delay_ms现已兼容RTOS且不改SysTick；
  *   3. CAN 初始化改由 board.c 的 board_init() 提供，main.c 中调用。
  * ------------------------------------------------------------------
  */

/* ---------------- 电机地址 & 安装方向映射 ----------------
   下标 0 ~ 3 依次对应例程的 speed[1] ~ speed[4]（左前/右前/左后/右后按实际接线）
     Motor_Addr   : 电机自己的 CAN 地址（ZDT 出厂默认 1~4，可用上位机改）
     Motor_FwdDir : 该电机"正速度"对应的方向位 0 = CW 正转，1 = CCW 反转
                    —— 实车跑起来发现某个轮子方向反了，就把这一位的 0/1 对调
   例程原始写法（yyb_move.c 第 28~55 行）：
     speed[1] -> 地址 2，正速度 -> dir 0        speed[3] -> 地址 3，正速度 -> dir 0
     speed[2] -> 地址 1，正速度 -> dir 1        speed[4] -> 地址 4，正速度 -> dir 1

   ---- 车体布局对应关系（关键，装错车走不了直线）----
     下标0 speed[1] = 左前轮 = addr 2   ┐ 左侧
     下标2 speed[3] = 左后轮 = addr 3   ┘
     下标1 speed[2] = 右前轮 = addr 1   ┐ 右侧
     下标3 speed[4] = 右后轮 = addr 4   ┘
   即：**addr 2 与 addr 3 必须是同一侧的两个轮子（一前一后）；
        addr 1 与 addr 4 是另一侧的两个轮子**。
   另一组对应关系是"斜对角"：addr 2 与 addr 4 在一条对角线上（左前↔右后），
                             addr 1 与 addr 3 在另一条对角线上（右前↔左后）。

   ---- 怎么一眼验证装对了（看轮子转向，不用上车跑）----
     · 跑"原地自转"（w≠0）时：同一侧的两个轮子应该朝同一个方向转。
       若看到的是同一条对角线上的两个轮子同向 → 地址配对错了。
     · 跑"左右平移"（vx≠0）时：同一条对角线上的两个轮子应该朝同一方向转。
     · 判断依据：自转项 w 只在同侧之间符号相同；左右项 vx 只在对角之间符号相同。
   */
static const uint8_t Motor_Addr[MOTOR_NUM]   = {2, 1, 3, 4};
static uint8_t raw_chassis_held; /* 原料区定位后固定四轮目标；真实停车/运动命令清除此标志 */
/* 实车标定(2026-09-25): 本车前/后两组电机安装方向镜像 —— 例程原始 {0,1,0,1} 下
   vy>0(前进)实车表现为左平移。对调 LF/RR 两位得 {1,1,0,0}, 并在 Motor_Move()
   里对 vx 列取反(两处必须配套修改), vx/vy/w 三个轴的方向才与注释约定一致。 */
static const uint8_t Motor_FwdDir[MOTOR_NUM] = {1, 1, 0, 0};

/**
  * @brief  速度取绝对值并限幅为 RPM 值
  */
static uint16_t motor_clamp(int v)
{
    if (v < 0)
    {
        v = -v;
    }
    if (v > MOTOR_MAX_VEL)
    {
        v = MOTOR_MAX_VEL;
    }
    return (uint16_t)v;
}

/**
  * @brief  有符号速度 -> 方向位（0 = CW，1 = CCW），自动套用安装方向映射
  */
static uint8_t motor_dir(uint8_t idx, int speed)
{
    uint8_t fwd = Motor_FwdDir[idx];
    return (speed >= 0) ? fwd : (uint8_t)(1 - fwd);
}

/**
  * @brief  一次控制四个电机（速度模式）
  * @param  s1 ~ s4 ：四轮带符号转速(RPM)，正负即方向
  * @note   四帧指令全部发出后，若 MOTOR_SYNC_START = 1 再广播同步帧，
  *         四轮同一时刻起步，避免各帧间隔造成的走偏
  */
void Motor_SetSpeed4(int s1, int s2, int s3, int s4)
{
    int sp[MOTOR_NUM];
    uint8_t i;

    raw_chassis_held = 0;
    sp[0] = s1;
    sp[1] = s2;
    sp[2] = s3;
    sp[3] = s4;

    for (i = 0; i < MOTOR_NUM; i++)
    {
        Emm_V5_Vel_Control(Motor_Addr[i],
                           motor_dir(i, sp[i]),
                           motor_clamp(sp[i]),
                           MOTOR_DEF_ACC,
                           (bool)MOTOR_SYNC_START);
    }

#if MOTOR_SYNC_START
    Emm_V5_Synchronous_motion(0x00);   /* 广播地址 0x00：全部电机同时执行缓存的指令 */
#endif
}

/**
  * @brief  麦轮底盘移动（例程矩阵 + 本车 vx 列取反 + w 列实车修正）
  * @param  vx ：**左右轴**，> 0 左移（RPM）
  * @param  vy ：**前后轴**，> 0 前进（RPM）
  * @param  w  ：自转，> 0 逆时针（RPM）
  * @note   注意轴的命名：v_x 管左右、v_y 管前后（不是常见的 x=前后）。
  *         判断依据是矩阵结构 —— 四行里系数全为 +1 的那一项才是前后轴：
  *           vy 列：+ + + +  → 四轮同速同向 = 前后（麦轮直行）
  *           vx 列：- + + -  → 按**对角线**分成两组 = 左右平移
  *           w  列：+ + - -  → **1&2号同向、3&4号反向**（实车标定：
  *              与原地旋转脉冲命令 CCW={s,s,-,-} 的正确模式一致；
  *              例程原始 w 列为 - + - +，本车实转方向分组不符，已修正）
  */
void Motor_Move(int vx, int vy, int w)
{
    /*  例程 yyb_move.c 原始矩阵:
          speed[1] = +v_x + v_y - w;
          speed[2] = -v_x + v_y + w;
          speed[3] = -v_x + v_y - w;
          speed[4] = +v_x + v_y + w;
        本车两处实车修正（缺一不可）：
        1) vx 列整体取反 —— 前后两组电机安装方向镜像（原矩阵下 vy>0 实际左移）；
        2) w 列改为 + + - - —— 实车正确的自转分组是 1&2 同向、3&4 反向
           （与 Motor_FwdDir={1,1,0,0} 标定的脉冲命令 CCW 模式一致）： */
    Motor_SetSpeed4( -vx + vy + w,
                      vx + vy + w,
                      vx + vy - w,
                     -vx + vy - w);
}

/**
  * @brief  45° 斜向平移
  * @param  dir_x ：+1 含向左分量 / -1 含向右分量
  * @param  dir_y ：+1 含向前分量 / -1 含向后分量（对应 Motor_Move 的 vy 约定）
  * @param  speed ：合成速度(RPM)，量级与直行相同
 * @note   把速度按 0.7071 分解到 vx / vy，代入运动学矩阵后正好是 2v 与 0：
 *           左前 45°：speed[1] = 0，speed[2] = 2v，speed[3] = 2v，speed[4] = 0
 *         即对角线上只有两个轮子出力、另两个完全停转 —— 这是麦轮的固有特性，
 *         所以 45° 斜走的轮速是合成速度的 1.414 倍，命令值别贴限幅上限。
 *         出力的轮子（对应上面的车体布局）：
 *           左前 / 右后 45° → addr 1（右前）+ addr 3（左后）
 *           右前 / 左后 45° → addr 2（左前）+ addr 4（右后）
  */
void Motor_MoveDiagonal(int8_t dir_x, int8_t dir_y, uint16_t speed)
{
    int vx = (int)speed * MOTOR_DIAG_COEF / 10000;   /* 0.7071 × speed */
    int vy = vx;

    if (dir_x < 0)
    {
        vx = -vx;
    }
    if (dir_y < 0)
    {
        vy = -vy;
    }

    Motor_Move(vx, vy, 0);
}

/**
  * @brief  四轮位置模式（相对运动，四轮同步起步）
  * @param  pulse[4] ：pulse[i] 符号 = 方向，绝对值 = 脉冲数(0 ~ 2^32-1)
  * @note   与例程 car_move_distance_x/y() 里 Emm_V5_Pos_Control(..., false, true)
  *         完全一致：raF = false 相对运动，snF = true 多机同步
  */
void Motor_MovePulses(const int32_t pulse[MOTOR_NUM])
{
    uint8_t i;

    for (i = 0; i < MOTOR_NUM; i++)
    {
        Emm_V5_Pos_Control(Motor_Addr[i],
                           motor_dir(i, (int)pulse[i]),
                           MOTOR_DEF_VEL,
                           MOTOR_DEF_ACC,
                           (uint32_t)ABS(pulse[i]),
                           false,
                           true);
    }

    Emm_V5_Synchronous_motion(0x00);
}

/**
  * @brief  立即停止四轮
  * @note   这里不用同步标志，逐轮立即停 —— 停机要可靠，不能等广播帧
  */
void Motor_Stop(void)
{
    uint8_t i;

    raw_chassis_held = 0;
    for (i = 0; i < MOTOR_NUM; i++)
    {
        Emm_V5_Stop_Now(Motor_Addr[i], false);
    }
}

/* 固定当前位置只下发一次；不能在取放循环内重设目标或用FE打断。 */
static uint8_t motor_hold_current(void)
{
    uint8_t i;
    for (i = 0; i < MOTOR_NUM; ++i)
    {
        if (!Emm_V5_Hold_Current_Position(Motor_Addr[i]))
        {
            Motor_Stop();
            return 0;
        }
    }
    return 1;
}

/**
  * @brief  使能 / 失能四轮
  * @param  state ：true 使能（FOC 上电抱死），false 失能（自然松开）
  */
void Motor_Enable(bool state)
{
    uint8_t i;

    for (i = 0; i < MOTOR_NUM; i++)
    {
        Emm_V5_En_Control(Motor_Addr[i], state, false);
    }
}

#if MOTOR_TASK_AUTO_DEMO
/**
  * @brief  自检动作的一步：以 (vx, vy, w) 跑 ms 毫秒后停下
  */
static void motor_demo_step(int vx, int vy, int w, uint32_t ms)
{
    Motor_Move(vx, vy, w);
    vTaskDelay(pdMS_TO_TICKS(ms));
    Motor_Stop();
    vTaskDelay(pdMS_TO_TICKS(300));   /* 停稳再切下一个动作 */
}

/**
  * @brief  自检动作的一步：向 45° 斜向 (dir_x, dir_y) 跑 ms 毫秒后停下
  */
static void motor_demo_diag(int8_t dir_x, int8_t dir_y, uint32_t ms)
{
    Motor_MoveDiagonal(dir_x, dir_y, MOTOR_DEF_VEL);
    vTaskDelay(pdMS_TO_TICKS(ms));
    Motor_Stop();
    vTaskDelay(pdMS_TO_TICKS(300));
}
#endif

#if !MOTOR_TASK_AUTO_DEMO
/**
  * @brief  方向 + 脉冲数 -> 四轮位置模式脉冲表
  * @param  dir  ：PULSE_DIR_FWD/BACK/LEFT/RIGHT/FL/FR/BL/BR（route_rx.h）
  * @param  n    ：出力轮脉冲数（1 ~ PULSE_MAX，route_rx 解析时已限幅）
  * @note   与 Motor_Move 的运动学一致（vx 列已取反的版本）：
  *           前进/后退 = 四轮同向；左移/右移与四个45°斜向 = 对角两轮出力，
  *           斜向出力轮各走 n 脉冲，另两轮停转（车体位移约为直走的0.707倍）
  */
static void pulse_build(int32_t out[MOTOR_NUM], uint8_t dir, uint32_t n)
{
    int32_t s = (int32_t)n;

    switch (dir)
    {
    case PULSE_DIR_BACK:   out[0] = -s; out[1] = -s; out[2] = -s; out[3] = -s; break;
    case PULSE_DIR_LEFT:   out[0] = -s; out[1] =  s; out[2] =  s; out[3] = -s; break;
    case PULSE_DIR_RIGHT:  out[0] =  s; out[1] = -s; out[2] = -s; out[3] =  s; break;
    case PULSE_DIR_FL:     out[0] =  0; out[1] =  s; out[2] =  s; out[3] =  0; break;
    case PULSE_DIR_FR:     out[0] =  s; out[1] =  0; out[2] =  0; out[3] =  s; break;
    case PULSE_DIR_BL:     out[0] = -s; out[1] =  0; out[2] =  0; out[3] = -s; break;
    case PULSE_DIR_BR:     out[0] =  0; out[1] = -s; out[2] = -s; out[3] =  0; break;
    case PULSE_DIR_CCW:    out[0] =  s; out[1] =  s; out[2] = -s; out[3] = -s; break;
    case PULSE_DIR_CW:     out[0] = -s; out[1] = -s; out[2] =  s; out[3] =  s; break;
    default:               out[0] =  s; out[1] =  s; out[2] =  s; out[3] =  s; break;
    }
}
#endif

/* ---- 位置闭环（串口 05 帧触发，目标=编码值；里程计 pos 反馈） ----
   位置环: pos误差 -> vy(RPM); 航向环: yaw误差 -> w(RPM), 反馈由所选陀螺仪统一入口提供。
   到位: |pos-目标| ≤ POS_TOL_COUNTS 立即停车(爬行段低速, 停车必落容差内)。
   整定: 先 Kp(0.0005起, 太慢加大/振荡减小), 再 Ki(消除稳态误差), Kd 默认关闭
   (里程计约35ms刷新一次位置, 微分对量化噪声敏感)。 */
#define POS_TOL_COUNTS      5000     /* 到位容差(编码值): 目标±5000, 进入即停车 */
#define POS_DECEL_COUNTS    280000   /* 减速区240000→280000，距目标约427mm提前减速；容差±5000 */
#define POS_QR_SLOW_COUNTS  12000    /* 最后约18mm进入低速平滑收尾 */
#define POS_APPROACH_RPM    44       /* 直行停车接近速度22→44RPM（2倍） */
#define POS_QR_SLOW_RPM     5        /* 扫码搜索速度保持5RPM */

/* 原料区视觉靠近：此阶段完全不使用里程计，只用实时圆心误差驱动ZDT速度模式。 */
#define RDK_CENTER_TARGET_X      320
#define RDK_CENTER_TARGET_Y      240
#define RDK_CENTER_TOL_PX        3
#define RDK_CENTER_KP_RPM_PX     0.35f
#define RDK_CENTER_MIN_RPM       4        /*比例段下限,乘速度系数后约2.7RPM*/
#define RDK_CENTER_MAX_RPM       40
#define RDK_CENTER_SPEED_SCALE   (2.0f / 3.0f) /*定位平移速度、加速斜率统一降为原来的2/3*/
#define RDK_CENTER_TIMEOUT_MS    60000u
#define RDK_CENTER_RESUME_MS      1000u
#define RDK_CENTER_STALE_MS      300u
/* 实车确认：AA 55 14 01 FF 14 0D 0A；正式路线到原料区后使用同一组方向。 */
#define RDK_CENTER_X_SIGN        1       /*画面Y误差 -> 车体vx(左右)*/
#define RDK_CENTER_Y_SIGN        -1      /*画面X误差 -> 车体vy(前后)*/
#define RDK_CENTER_SLEW_RPM_S    180.0f  /*视觉速度每秒最大变化，防止突然推车*/
#define RDK_CENTER_RAW_YAW       90.0f   /*原料区作业航向*/
#define RDK_RAW_ARM_PRE_DEG      455     /*原料区到站先伸夹爪角度,到位后再识别转盘*/
#define RDK_RAW_ARM_GRAB_DEG     435     /* 转盘定位并锁轮后的RM预定位目标 */
/* 新丝杆：以下丝杆目标均已按旧值除以4，命令与回读直接使用新脉冲值。 */
#define RDK_RAW_PRE_LIFT_POS     2000u    /* 原料区提前准备丝杆绝对目标，不是回零动作 */
#define RDK_RAW_LIFT_POS         21000u  /* 原纯机构抓取序列前两块收尾位置；视觉对位后不再下发 */
#define RDK_RAW_FINISH_ARM_DEG   18      /* 原料区第三块装车后RM收起目标：原101°改为20° */
#define RDK_RAW_FINISH_LIFT_POS  12500u  /*第三块装车后丝杆收起高度，确认到位再结束/回程*/
#define RDK_RAW_BELT_POS         7040 /*原料区专用到站/收尾位置，不改变圆环站点中心*/
#define RDK_RAW_CLAW_CLOSE_DEG   79.5f   /*原料区统一夹紧角度：真实路线及独立1A/1E共用(2026-11-15, 原79°)*/
#define RDK_RAW_REGION_CLAW_CLOSE_DEG 87.5f /* 三区视觉抓取夹紧角度；纯机构1A沿用原角度 */
#define RDK_RAW_SERVO180_DEG     25.0f   /*原料区统一张开角度：到站、每块入槽及第三块收尾共用(2026-11-15回调, 原28°)*/
#define RING_STN_ARM_DEG         190     /*圆环站点(粗加工/暂存)到站预定位: RM电机角度*/
#define RING_STN_BELT_POS        7100 /*两轮粗加工/两轮暂存共用到站位置，独立于中心标定(2026-11-15, 原7080)*/
#define RING_STN_SERVO180_DEG    25.0f   /* 两轮粗加工/暂存驶入及到站预定位张开角；放料松爪仍30° */
#define GRAB_SERVO_DWELL_MS      400u    /*统一夹取序列: 舵机步进后驻留时间(夹紧用)*/
#define PLACE_OPEN_DWELL_MS      100u    /*放料/取回松爪驻留: 100ms(2026-11-15, 原400ms)*/
#define PLACE_RETRIEVE_CLOSE_DWELL_MS 300u /*取回夹紧驻留: 300ms(2026-11-15, 原400ms共用)*/
#define RAW_LIFT_WAIT_LIMIT_MS   1000u   /*原料区丝杆：只等9F到位帧，1秒未确认也放行*/
#define RAW_BELT_WAIT_LIMIT_MS   2000u   /*原料区同步带：只等9F到位帧，2秒未确认也放行*/
#define RAW_ARM_WAIT_LIMIT_MS    1500u   /* 原料区RM等待3→1.5秒；反馈正常时到位提前放行 */
#define GRAB_TRIG_TIMEOUT_MS     30000u  /*发一次颜色命令后只等0x11状态帧, 30s总窗超时no next*/
#define RAW_ARM_POS_TOL_DEG      5.0f    /*原料区RM到位角度容差(2026-11-13放宽2°→5°)*/
#define BELT_POS_TOL             200     /*到站同步带回读兜底容差(脉冲)*/
#define GRAB_ARM_POS_TOL_DEG     1.0f    /*统一夹取序列: M2006到位角度容差*/

/* ---- 动态中断优先级调度(2026-11-14): 数值越小越高, ≥5可调FromISR ----
 * UART7: 初始0(路线上传窗口全局最高, ISR无FromISR合法); 一键启动/0x1E/0x12
 *        启动后降7(不再收路线); 06中止恢复0(重新可上传)。
 * USART3: 恒5(=MAX_SYSCALL)。HWT101全程保持4，不参与到站降级。
 * Encoder/轮趣IMU: 行车5/4；到站对位/机构动作期降15让路;
 *        恢复点=RESUME/ROUTE_RESUME进入时(回程与续路开始前), 行驶段恒行车档。 */
#define IRQ_PRIO_UART7_UPLOAD    0u
#define IRQ_PRIO_UART7_IDLE      7u
#define IRQ_PRIO_RDK_RUN         5u
#define IRQ_PRIO_ENC_RUN         5u
#define IRQ_PRIO_ENC_LOW         15u
#define IRQ_PRIO_IMU_RUN         4u
#define IRQ_PRIO_IMU_LOW         15u

#define POS_OUT_MAX_RPM     270      /* 直线巡航上限300→270RPM（降低10%）；起步/接近/末端速度保持 */
#define POS_START_RPM       15       /* 起步速度上限15RPM，按900ms爬升至巡航上限 */
#define POS_FINAL_RPM       16.0f    /* 末段停车速度8→16RPM（2倍）；到位/跨目标仍结束 */
#define POS_FINISH_MS       150u    /*仅低速正常到位用150ms收尾；故障/超目标仍立即停*/
#define POS_DECEL_SLEW_RPM_S 270.0f /*直行减速斜率随巡航提高1.5倍（180→270RPM/s）；
                                        缩短减速区后仍保留原到位/跨目标停车保护*/
#define POS_START_RAMP_MS   900u      /* 直行起步加速时间600→900ms，保留平滑爬升 */
#define POS_RUN_TIMEOUT_MS  30000u   /* 位置闭环最长运行时间, 超时自动停车 */
#define POS_PROGRESS_MS     1000u   /* 持续未向目标靠近时停车(反馈不变/方向错误) */
#define POS_PROGRESS_COUNTS 1000.0f /* 每次确认进度需至少靠近约1.5mm */
#define ODOM_RESUME_VALID_FRAMES 3u /* 掉线停车后，连续3次有效位置回包才自动续当前路段 */

#define POS_PID_KP          0.00075f /* 直行位置环速度输出提高1.5倍，非仅提高巡航上限 */
#define POS_PID_KI          0.00045f
#define POS_PID_KD          0.0f     /* 微分默认关闭 */
#define POS_ERR_INT_MAX     30000.0f /* 积分限幅(编码值) */

#define POS_YAW_TARGET      0.0f     /* (已废弃) 航向保持目标改为每步起始朝向 yaw_hold */
#define YAW_PID_KP          2.0f     /* 误差(deg) -> RPM */
#define YAW_PID_KI          0.5f
#define YAW_PID_KD          0.0f
#define YAW_OUT_MAX_RPM     50
#define LINE_YAW_KP        3.0f    /*增强及时回正(RPM/deg)，参考工程PID思路适配本车*/
#define LINE_YAW_KI        0.8f    /*按秒积分(RPM/(deg*s))，补偿持续轮速偏差*/
#define LINE_YAW_KD        0.15f   /*角速度阻尼，避免阻尼过强拖慢回正*/
#define LINE_YAW_INT_RPM   3.0f    /*积分贡献上限，避免旧环的大积分残留*/
#define LINE_YAW_MAX_RPM   25.0f
#define LINE_YAW_SPEED_RATIO 0.35f /*低速随平移速度收紧修正，避免末段反向摆头*/
#define LINE_YAW_MIN_LIMIT 2.0f
#define LINE_YAW_ENTER_DEG 0.4f
#define LINE_YAW_EXIT_DEG  0.2f
#define LINE_YAW_RATE_TAU  0.04f
#define LINE_YAW_SLEW_RPM_S 120.0f /*比上一版更快建立纠偏，仍按真实dt限斜率*/
#define LINE_DIRECTION_MAX_DEG 15.0f /*车头暂时偏移时补偿直行方向，限制横向分量*/
#define YAW_ERR_INT_MAX     30.0f    /* 积分限幅(deg) */
#define YAW_OUT_SIGN        (-1.0f)  /*航向纠正方向: 实车45°斜走实测越纠越偏, 已翻转。
                                        适用于直线/缓行/校准/45°的yaw保持(共用YawPID);
                                        两款驱动分别保持原车yaw方向，保持此实车符号*/
#define TURN_OUT_SIGN       (-1.0f)  /*原地转向输出方向: 实车"左转发成右转270°"已实测, 翻转。
                                        与YAW_OUT_SIGN同值(两者闭环的都是IMU增方向)*/

#define POS_LEG1_RPM        60       /* 45°斜走巡航: 出力轮转速(RPM), 右前=LF+RR 左前=RF+LR */
#define POS_LEG1_START_MS    800     /*斜走按时间缓启动，减少两轮突然出力打滑*/
#define POS_LEG1_START_RPM   8
#define POS_ROUTE_DIAG_SPEED_SCALE 2.25f /* 一键路线出发45°：在原1.5倍基础上再×1.5，合计2.25倍 */
#define POS_LEG1_APPROACH_RPM 12     /*斜行到点速度，上一版10RPM*/
#define POS_LEG1_APPROACH_COUNTS 25000 /*末段约38mm平滑减速，上一版30000计数*/

/* ---- 场地与点位(编码值) ----
   注意: 两启停区出发时X轮计数方向镜像 —— 同一"场地右上角",
   区一(右前去程)坐标系下 pos=POS_HOME_POS, 区二(左前去程)下 pos=X总长-POS_HOME_POS ---- */
#define POS_X_SPAN          1450000  /* 场地X向总长(编码值, 实测丈量) */
#define POS_HOME_POS        200000   /* 右上角点位(区一坐标系) */

/* ---- 任务阶段状态机(串口05/06帧驱动) ----
   阶段号即枚举值, 与"RunFlag"语义一一对应, 维护时只看这里:
     1 = PHASE_DIAG  45°斜走段(leg_dir=1右前/2左前), 段末自动衔接直线
     2 = PHASE_LINE  直线行驶段(纯PID直行)
     5 = PHASE_SLOW  缓行直行+扫码判停段(距目标<QR_SLOW时由LINE切入)
     3 = PHASE_DONE  已到位/已扫到码, 停车保持
     4 = PHASE_TURN  原地转向段(yaw闭环)
     7 = PHASE_HOME  回右上角点位段(直线同款PID, 方向由误差自动决定)
     0 = PHASE_IDLE  空闲, 等待命令(扫到码停车后=DONE, 同样可接新帧继续行走)
   流转: DIAG --段末--> LINE --距目标<QR_SLOW--> SLOW --扫到码--> DONE
         (停车后等待上位机下一动作帧继续行走)
   帧流程不变: 45°段帧 + 直线段帧(排队) 两帧触发一条完整任务链;
   07一键帧等价于两帧连发。 */
typedef enum
{
    PHASE_IDLE = 0,
    PHASE_DIAG = 1,
    PHASE_LINE = 2,
    PHASE_DONE = 3,
    PHASE_TURN = 4,
    PHASE_SLOW = 5,
    PHASE_HOME = 7,
    PHASE_RDK_CENTER = 8,
    PHASE_RDK_RESUME = 9,
    PHASE_RDK_GRAB_WAIT = 10,
    PHASE_RDK_RETURN = 11,
    PHASE_RDK_ARM_EXTEND = 12,
    PHASE_RDK_RING_WAIT = 13,
    PHASE_ARM_MANUAL = 14,
    PHASE_RDK_ARM_PRE = 15,
    PHASE_RDK_GRAB_TRIG = 16,
    PHASE_RDK_GRAB_SEQ = 17,
    PHASE_RDK_ARM_POST = 18,
    PHASE_STATION_PLACE = 19,
    PHASE_RDK_CHASSIS_HOLD = 20,
    PHASE_RDK_GRAB_REGION = 22 /* 三区共用抓取序列，首组目标由报告区域决定 */
} TaskPhase;

/* 新任务可启动判定: 空闲 或 已完成停车。
   单帧=单步指令: 车停着(DONE)就必须立即执行下一帧(实车bug: 停车后发帧只回ACK不走车);
   07整体链同理(链的前置判断在链内部: 各段自动衔接, 无需外部状态)。 */
#define PHASE_CAN_START(p)  ((p) == PHASE_IDLE || (p) == PHASE_DONE)

/* ---- 任务步骤数组(上位机04动作帧驱动, 每改变一个方向=一步) ----
   type: 1=45°斜走(dir=地图方向码05右前/04左前/07右后/06左后)
         2=直线到pos(里程计1反馈, target正负=计数方向)
         3=缓行+扫到码即停  5=原地转90°(dir=3顺/4逆)
   区一坐标系: X总长1450000, 右上角100000, 扫码区650000 —— 数值实测后直接改本表 */
#define STEPT_DIAG  1
#define STEPT_LINE  2
#define STEPT_SLOW  3
#define STEPT_TURN  5

typedef struct
{
    uint8_t type;      /*步骤类型*/
    uint8_t dir;       /*斜走方向码 / 转弯方向(3顺4逆)*/
    int32_t target;    /*相对步长(编码值): 正=沿当前车头前进, 负=后退; 45°斜走=斜向距离*/
} TaskStep;

/* 图二"第一批"动作序列(黄色物料) —— target=每步相对步长(正前进/负后退), 全部走里程计1 */
static const TaskStep task_z1[] = {
    {STEPT_DIAG, 5,   100000},   /*45度斜走: 纵向计到100000(启停区1-右上角45度, 方向码实测调)*/
    {STEPT_LINE, 0,   650000},   /*切换直线: 继续纵向到650000(扫码区)*/
    {STEPT_LINE, 0,  -550000},   /*后退550000: 扫码区-右上角(650000-100000)*/
    {STEPT_TURN, 3,         0},  /*顺时针转90度(车头变为朝场地左)*/
    {STEPT_LINE, 0,   634000},   /*前进(估算1050mm): 到原料区横向位置——实测改*/
    {STEPT_TURN, 4,         0},  /*逆时针转90度(车头回朝场地下方)*/
    {STEPT_LINE, 0,  1253000},   /*前进(估算2075mm): 到粗加工区——实测改*/
    {STEPT_TURN, 3,         0},  /*顺时针转90度(车头朝场地左)*/
    {STEPT_LINE, 0,   300000},   /*前进(估算500mm): 到左下角/暂存区横向位——实测改*/
    {STEPT_TURN, 3,         0},  /*顺时针转90度*/
    {STEPT_LINE, 0,   544000},   /*前进(估算900mm): 到暂存区纵向位——实测改*/
};
#define TASK_Z1_N  (sizeof(task_z1) / sizeof(task_z1[0]))

/* ---- 各步骤步长=两测量点间距(mm)x604.17, 实测后直接改上表数值 ---- */

/* ---- 原地转向闭环(05帧 sub=2左转/sub=3右转, param=角度deg) ----
   反馈: IMU_GetYaw()直读当前陀螺仪最新值(中断刷新, 滞后<=10ms)。
   若经 OdomTask(35ms RS485轮询节奏)透传, 高速段每盲区多转2~3°, 会来回振荡!
   以触发时刻航向为原点计算相对角, 角差全程归一, 无±180°环绕跳变。 */
#define TURN_TOL_DEG        1.5f     /* 到位容差±1.5°，连续3个新角度帧确认 */
#define TURN_OUTPUT_SPEED_RATIO 0.75f /* 所有原地转弯最终轮速为调整前的3/4，含爬行和站点慢转 */
#define TURN_CREEP_DEG      15.0f    /* 最后15°固定低速逼近 */
#define TURN_CREEP_RPM      30       /* 最后15°逼近速度15→30RPM（2倍） */
#define TURN_SPEED_SCALE    4.2f     /* 远段倍率3.6→4.2；30°~15°平滑降到爬行 */
#define TURN_OUT_MAX_RPM    140      /* 转向PID基础限幅70→140RPM（2倍） */
#define TURN_FAST_DEG       30.0f    /*此角度以内沿用已实测的减速参数*/
#define TURN_FAST_KP        0.50f    /* 远段比例增量提高2倍，与基础PID输出配套 */
#define TURN_FAST_MAX_RPM   170      /* 大角度基础限幅85→170RPM（再乘远段倍率） */
#define TURN_MIN_RPM        32       /* PID段最低速度16→32RPM（2倍），衔接30RPM爬行 */
#define TURN_SETTLE_TICKS   3        /* 到位后连续3个新角度帧确认；100Hz时约30ms，旧帧不计数。 */
#define TURN_TIMEOUT_MS     15000u   /* 转向最长运行时间, 超时自动停车(陀螺仪无效兜底) */
#define TURN_FULL_TIMEOUT_MS 60000u  /* 相对大角度转向(180~360°)的最长运行时间 */
#define TURN_PID_KP         1.50f    /* 角度误差→RPM，原0.75的2倍，接近目标仍减速
                                             有滞后, 增益须保守(参照直线环稳定比例) */
#define TURN_PID_KI         0.0f     /* 关闭积分，避免停下后的反复补偿 */
#define TURN_PID_KD         20.0f    /* 微分阻尼随转向输出提高2倍（10→20），保留减速形状 */
#define TURN_ERR_INT_MAX    45.0f    /* 积分限幅(deg) */
#define TURN_SLEW_RPM       20       /* 每拍(10ms)速度变化上限10→20RPM（2倍）。
                                        命令阶跃会激励ZDT内部斜率跟随导致惯性甩尾超调
                                        (开源工程同款电机靠此招稳住90°转弯), 必加 */
#define TURN_RING_STATION_RATIO 0.5f /* 两轮粗加工/暂存伸臂前到站转向：整段轮速减半 */

static PID_t PosPID;    /* 位置环: 里程计1(X纵向) -> vy(RPM) */
static PID_t YawPID;    /* 航向环: yaw误差 -> w(RPM) */
static PID_t TurnPID;   /* 转向环: 相对角误差 -> w(RPM) */

/**
  * @brief  角差归一: a-b 收敛到 (-180,180], 用于相对角计算(避开±180°跳变)
  */
static float turn_angdiff(float a, float b)
{
    float d = a - b;
    while (d > 180.0f)   d -= 360.0f;
    while (d <= -180.0f) d += 360.0f;
    return d;
}

/*直行PID按实际dt积分，配合速度方向补偿；转弯和45°控制保持独立。*/
static struct
{
    float yaw_last;
    float rate;
    float output;
    float integral;
    float error_last;
    TickType_t tick_last;
    uint8_t active;
    uint8_t ready;
} line_yaw;

static void line_yaw_reset(void)
{
    line_yaw.ready = 0;
}

static int line_rpm_round(float rpm)
{
    return (int)(rpm >= 0.0f ? rpm + 0.5f : rpm - 0.5f);
}

static int line_yaw_output(float target_yaw, int translation_rpm)
{
    TickType_t now = xTaskGetTickCount();
    float yaw = IMU_GetYaw();
    float error = turn_angdiff(target_yaw, yaw);
    float dt, desired, limit, slew, corrected_error;

    if (!line_yaw.ready)
    {
        line_yaw.yaw_last = yaw;
        line_yaw.rate = 0.0f;
        line_yaw.output = 0.0f;
        line_yaw.integral = 0.0f;
        line_yaw.error_last = error;
        line_yaw.active = 0;
        line_yaw.tick_last = now;
        line_yaw.ready = 1;
        dt = 0.01f;
    }
    else
    {
        TickType_t elapsed = now - line_yaw.tick_last;
        dt = (float)elapsed / (float)configTICK_RATE_HZ;
        if (elapsed != 0)
        {
            /*角差归一防止359°->0°产生假角速度；真实dt避免调度快慢改变阻尼。*/
            line_yaw.rate += dt / (LINE_YAW_RATE_TAU + dt) *
                            (turn_angdiff(yaw, line_yaw.yaw_last) / dt - line_yaw.rate);
            line_yaw.yaw_last = yaw;
            line_yaw.tick_last = now;
        }
    }

    if (fabsf(error) >= LINE_YAW_ENTER_DEG) line_yaw.active = 1;
    else if (fabsf(error) <= LINE_YAW_EXIT_DEG) line_yaw.active = 0;

    limit = fabsf((float)translation_rpm) * LINE_YAW_SPEED_RATIO;
    if (limit < LINE_YAW_MIN_LIMIT) limit = LINE_YAW_MIN_LIMIT;
    if (limit > LINE_YAW_MAX_RPM) limit = LINE_YAW_MAX_RPM;
    if (translation_rpm == 0) limit = 0.0f;
    limit = floorf(limit); /*整数电机指令也不能因四舍五入超过当前限幅*/
    /*控制间隔异常时不补积一大段误差。*/
    if (dt > 0.1f) dt = 0.1f;
    if (error * line_yaw.error_last < 0.0f || !line_yaw.active || translation_rpm == 0)
        line_yaw.integral = 0.0f;
    line_yaw.error_last = error;

    corrected_error = 0.0f;
    if (line_yaw.active)
    {
        if (error > LINE_YAW_EXIT_DEG) corrected_error = error - LINE_YAW_EXIT_DEG;
        if (error < -LINE_YAW_EXIT_DEG) corrected_error = error + LINE_YAW_EXIT_DEG;
    }
    desired = LINE_YAW_KP * corrected_error + line_yaw.integral - LINE_YAW_KD * line_yaw.rate;
    /*输出已饱和时，只允许有助于解除饱和的积分；目标附近不留积分。*/
    if (line_yaw.active && translation_rpm != 0 &&
        ((fabsf(desired) < limit) || desired * error < 0.0f))
    {
        float int_limit = limit * 0.25f;
        if (int_limit > LINE_YAW_INT_RPM) int_limit = LINE_YAW_INT_RPM;
        line_yaw.integral += LINE_YAW_KI * corrected_error * dt;
        if (line_yaw.integral > int_limit) line_yaw.integral = int_limit;
        if (line_yaw.integral < -int_limit) line_yaw.integral = -int_limit;
    }
    if (!line_yaw.active && fabsf(line_yaw.rate) <= 2.0f) desired = 0.0f;
    else desired = (LINE_YAW_KP * corrected_error + line_yaw.integral -
                    LINE_YAW_KD * line_yaw.rate) * YAW_OUT_SIGN;
    if (desired > limit) desired = limit;
    if (desired < -limit) desired = -limit;
    /*调度间隔过长时也限制单次变化，防止补偿指令跳变。*/
    if (dt > 0.1f) dt = 0.1f;
    slew = LINE_YAW_SLEW_RPM_S * dt;
    if (desired > line_yaw.output + slew) line_yaw.output += slew;
    else if (desired < line_yaw.output - slew) line_yaw.output -= slew;
    else line_yaw.output = desired;
    /*减速后立即遵守当前限幅，末段5RPM时修正最多2RPM。*/
    if (line_yaw.output > limit) line_yaw.output = limit;
    if (line_yaw.output < -limit) line_yaw.output = -limit;
    return line_rpm_round(line_yaw.output);
}

/*参考Car_Move的坐标变换：保持本段目标yaw方向的直行速度。
  本车vx正=左移，yaw正=顺时针，与参考车符号不同，横向取负sin。
  只补偿当前航向偏差，不改变路线pos/行程；倒车由translation_rpm符号自然处理。*/
static void line_move(int translation_rpm, float target_yaw)
{
    float error = turn_angdiff(target_yaw, IMU_GetYaw());
    float radians;
    int vx, vy, w;
    if (error > LINE_DIRECTION_MAX_DEG) error = LINE_DIRECTION_MAX_DEG;
    if (error < -LINE_DIRECTION_MAX_DEG) error = -LINE_DIRECTION_MAX_DEG;
    radians = error * 0.01745329252f;
    vx = line_rpm_round(-(float)translation_rpm * sinf(radians));
    vy = line_rpm_round((float)translation_rpm * cosf(radians));
    w = line_yaw_output(target_yaw, translation_rpm);
    Motor_Move(vx, vy, w);
}

static float yaw_abs360_to_signed(float yaw)
{
    yaw = fmodf(yaw, 360.0f);
    if (yaw > 180.0f) yaw -= 360.0f;
    return yaw;
}

/**
  * @brief  四轮电机任务
  * @note   MOTOR_TASK_AUTO_DEMO = 1 时循环跑自检动作；
  *         = 0 时轮询执行 ZigBee(UART7) 收到的命令：
  *           03 脉冲定距: 回ACK -> 使能 -> 延时1s -> 位置模式走N脉冲 -> 走完自停
  *           05/06 定位闭环: PID驱动到目标pos(容差±5000), 06中止; 参数见上方宏
  */
/* ---- 任务序列执行器状态 ---- */
static uint8_t step_idx = 0;      /*当前步骤下标*/
static uint8_t array_mode = 0;    /*0=单步(每帧一步) 1=一键连跑*/
static void task_step_start(const TaskStep *st);
static TaskPhase    phase = PHASE_IDLE;   /*任务阶段(执行器与主循环共用)*/
static int32_t      pos_target = 0;    /*目标累计编码值，与LCD pos/PID反馈同坐标*/
static int32_t      leg1_end = 0;      /*45°段纵向编码器行程*/
static int32_t      leg1_next = 0;     /*07旧链衔接目标*/
static uint8_t      leg1_next_valid = 0;
static uint8_t      leg_dir = 1;       /*斜走轮组: 1右前 2左前 3右后 4左后*/
static float        yaw_start = 0.0f;  /*转向段起始航向*/
static float        turn_target = 0.0f;/*转向段目标相对角*/
static uint8_t      turn_accumulate = 0; /*相对0~360°使用连续累计角*/
static float        turn_travel = 0.0f;
static float        turn_yaw_last = 0.0f;
static uint32_t     turn_timeout_ms = TURN_TIMEOUT_MS;
static int          turn_w_last = 0;   /*转向限斜率基准*/
static float        turn_output_ratio = 1.0f; /* 每段重置，站点慢转不能影响后续路线转弯 */
static uint8_t      turn_settle_ticks = 0; /*到位停车稳定确认计数*/
static uint32_t     turn_sample_sequence = 0; /* 转向只按新有效角度帧累计到位确认。 */
static TickType_t   posrun_start = 0;  /*本步骤起始时刻*/
static TickType_t   pos_ramp_start = 0; /*恢复后只重启起步斜坡，不重置本段累计运行时间*/
static volatile uint8_t odom_wait_active = 0; /* LCD读取等待状态 */
static TickType_t   odom_wait_tick = 0;
static uint32_t     odom_wait_sequence = 0;
static TaskPhase    odom_wait_phase;
static float        yaw_hold = 0.0f;   /*本步航向保持目标: 每步启动时刻的车头朝向(相对零点±180)*/
static float        pos_initial_error = 0.0f;
static float        pos_best_error = 0.0f;
static TickType_t   pos_progress_tick = 0;
static float        pos_speed_last = 0.0f;
static TickType_t   pos_finish_tick = 0;
static uint8_t      pos_finish_active = 0;
static volatile uint8_t run_fault = MOTOR_RUN_FAULT_NONE;
static MotorRunFaultSnapshot_t run_fault_snapshot;
static int32_t      diag_start = 0;    /*斜走步起点计数(相对步长基准)*/
static uint8_t      debug_route_active = 0; /*0x13调试路线运行中*/
static uint8_t      debug_route_batch = 1;
static uint8_t      debug_route_next_batch = 0; /*两批连跑时预先锁定的第二批*/
static uint8_t      debug_first_raw_reached = 0; /*首次到原料区后，扫码区校准改用相对行程*/
static uint8_t      debug_route_step = 0;   /*1斜行/2扫码/3校准或回右上/4内部转向/5中心/6路线转向/7到站*/
static uint16_t     debug_route_heading = 0; /*已完成动作的目标yaw，限定0/90/180/270*/
static uint16_t     debug_goal_heading = 0;
static volatile int32_t debug_pos_offset = 0; /*路线任务写，LCD读：路线pos=原始pos+偏移*/
static int32_t      debug_turn_pos_raw = 0;    /*转弯开始时里程计1原始pos*/
static volatile int32_t debug_turn_pos_hold = 0;   /*转弯期间保持的路线pos*/
static volatile uint8_t debug_turn_pos_active = 0; /*路线转弯期间冻结路线pos*/
static RouteDebugLeg debug_route_leg;
static uint8_t      debug_rule_index = 0;
static uint8_t      debug_motion_index = 0;
static volatile uint8_t no_next_alarm = 0;  /*MotorTask写，LCD_Task读*/
#define NO_NEXT_RETRY_MS 300u /* 仅RTOS tick定期检测，不阻塞或跳过未完成操作 */
enum { NEXT_RETRY_NONE, NEXT_RETRY_LOAD, NEXT_RETRY_MOTION,
       NEXT_RETRY_DEPART, NEXT_RETRY_FAULT };
static uint8_t no_next_retry = NEXT_RETRY_NONE;
static TickType_t no_next_retry_tick;
static volatile uint8_t key_start_request;
static TickType_t key_start_tick, start_message_tick;
enum { START_MSG_NONE, START_MSG_QUEUED, START_MSG_BUSY, START_MSG_IMU,
       START_MSG_UPLOAD, START_MSG_ROUTE, START_MSG_RUNNING };
static uint8_t start_message;
static uint8_t      debug_pause_active = 0;
static uint8_t      debug_pause_next = 0;   /*1下一点对/2转向后/3操作后/4扫码成功/0结束(5/6双声已删)*/
static TickType_t   debug_pause_until = 0;
static uint8_t      debug_qr_scan_active = 0;
static uint8_t      debug_qr_scan_done = 0; /*成功后跳过后续找码；新一轮第一批重置*/
static uint8_t      debug_rdk_center_active = 0;
static uint8_t      debug_rdk_standalone = 0; /*0x14独立定位：里程计持续读取，到位后不启动夹取/回程*/
static int8_t       debug_rdk_x_sign = RDK_CENTER_X_SIGN;
static int8_t       debug_rdk_y_sign = RDK_CENTER_Y_SIGN;
static uint8_t      belt_servo_done = 0;  /*同步带到位确认后180°舵机已转48°*/
static uint8_t      grab_step_index = 0;  /*统一夹取序列当前步*/
static uint8_t      grab_sub = 0;         /*0=发步进命令 1=等待该步完成*/
static uint8_t      grab_parallel_belt_done = 0;
static uint8_t      grab_parallel_lift_done = 0;
static MotorGrabFault_t grab_fault;
static uint8_t      grab_pick_index = 0;  /*本批已完成夹取次数(0=第一块)*/
static uint8_t      raw_debug_full = 0; /*1E独立原料全流程：三块收尾后停车，不回位/续路*/
static uint8_t      grab_manual = 0;      /*1=网页/串口手动触发: 完成后直接停车*/
static uint8_t      grab_pick_fixed = 0xFF; /*!=0xFF: 固定用该块序, 不递增序号*/
static uint8_t      grab_trig_auto = 0;   /*1=完成后不等夹取通知直接回程(等RDK触发帧流程)*/
static MotorGrabRegion_t grab_region; /* 本轮首个匹配报告；与槽位/环号独立 */
static uint8_t      grab_expect_color = 0; /*本周期发给RDK的追踪颜色1~6*/
static TickType_t   grab_step_tick = 0;
static TickType_t   grab_trig_tick = 0;
static TickType_t   raw_chassis_hold_tick = 0;

/* 前两块收尾回7040/290°/21000；第三块同步带保持1540、RM18°/丝杆12500，舵机转首个待放物料原槽。 */
enum { GRAB_ST_SERVO180, GRAB_ST_SERVO270, GRAB_ST_ARM, GRAB_ST_LIFT,
       GRAB_ST_BELT, GRAB_ST_BELT_ARM, GRAB_ST_BELT_SERVO270_ARM };
typedef struct { uint8_t type; float deg; int32_t target; } GrabStep;
static const GrabStep grab_steps[] = {
    { GRAB_ST_SERVO180, RDK_RAW_CLAW_CLOSE_DEG, 0 }, /*原料区夹紧*/
    { GRAB_ST_LIFT,     0,      12500 }, /*夹住后抬起到原料区搬运高度*/
    { GRAB_ST_BELT_ARM, 18.0f, BELT_POS_GRAB_PUL },
    { GRAB_ST_LIFT,     0,      19500 },
    { GRAB_ST_SERVO180, RDK_RAW_SERVO180_DEG, 0 }, /*原料区张开*/
    { GRAB_ST_BELT_SERVO270_ARM, 290.0f, RDK_RAW_BELT_POS },
};
#define GRAB_STEP_NUM (sizeof(grab_steps) / sizeof(grab_steps[0]))

/* 270°舵机全车机械角度+30°：A/B/C槽=30/150/270°，每块入槽后转到下一空槽。 */
static const float grab_pick_servo270[3] = {
    SERVO270_SLOT_B_DEG, SERVO270_SLOT_C_DEG, SERVO270_SLOT_A_DEG
};
static const float grab_pick_servo270_drop[3] = {
    SERVO270_SLOT_A_DEG, SERVO270_SLOT_B_DEG, SERVO270_SLOT_C_DEG
};

/* 三区首组目标不同；仅右上夹紧后先RM450°，其余搬运/返回共用。 */
static const struct { uint8_t axes; int32_t belt; int16_t arm; } raw_region_target[3] = {
    { MOTOR_GRAB_WAIT_RM | MOTOR_GRAB_WAIT_LIFT | MOTOR_GRAB_WAIT_BELT, 7150, 180 }, /* 1中下 */
    { MOTOR_GRAB_WAIT_RM | MOTOR_GRAB_WAIT_LIFT | MOTOR_GRAB_WAIT_BELT, 6450, 610 }, /* 2左上 */
    { MOTOR_GRAB_WAIT_RM | MOTOR_GRAB_WAIT_LIFT | MOTOR_GRAB_WAIT_BELT, 7850, 610 }, /* 3右上 */
};
enum { RAW_REGION_TARGET, RAW_REGION_CLOSE, RAW_REGION_RIGHT_ARM, RAW_REGION_LIFT, RAW_REGION_TRANSFER,
       RAW_REGION_OPEN, RAW_REGION_RETURN, RAW_REGION_STEP_COUNT };
typedef struct {
    uint8_t axes, fixed_wait;
    uint16_t wait_ms;
    int32_t belt;
    uint32_t lift;
    int16_t arm;
    float claw;
} RawRegionStep;
static const RawRegionStep raw_region_steps[RAW_REGION_STEP_COUNT] = {
    { MOTOR_GRAB_WAIT_RM | MOTOR_GRAB_WAIT_LIFT | MOTOR_GRAB_WAIT_BELT, 0, 1000, 0, 22000u, 0, 0 },
    { 0, 1, GRAB_SERVO_DWELL_MS, 0, 0, 0, RDK_RAW_REGION_CLAW_CLOSE_DEG },
    { MOTOR_GRAB_WAIT_RM, 0, 1000, 0, 0, 450, 0 }, /* 仅右上：先回450°再抬丝杆 */
    { MOTOR_GRAB_WAIT_LIFT, 1, 300, 0, 14200u, 0, 0 },
    { MOTOR_GRAB_WAIT_BELT | MOTOR_GRAB_WAIT_RM, 0, 800, 1540, 0, 18, 0 },
    { 0, 1, 100, 0, 0, 0, RDK_RAW_SERVO180_DEG },
    { MOTOR_GRAB_WAIT_BELT | MOTOR_GRAB_WAIT_LIFT | MOTOR_GRAB_WAIT_RM,
      0, 2000, 7100, 2000u, 435, 0 }, /* 前两块返回及第三块收起均最多2秒 */
};
static struct {
    uint8_t step, started, done, axes;
    int32_t belt_target;
    uint32_t lift_target;
    int16_t arm_target;
} raw_region;

/* 圆环站点放料及粗加工取回；与原料区grab_steps独立，保存物料/环号/原槽位。 */
#define PLACE_LIFT_WAIT_LIMIT_MS 1000u   /* 两轮粗加工/暂存：收到9F提前继续，否则1秒放行(2026-11-14, 1.5s全部改1s) */
#define PLACE_LIFT_LOW_POS_PUL   5000u   /* 丝杆目标0~5000脉冲（含边界）使用短等待 */
#define PLACE_LIFT_LOW_WAIT_MS   200u    /* 上述低脉冲目标：最多等0.2秒(2026-11-14取料组要求) */
#define PLACE_BELT_WAIT_LIMIT_MS 500u    /* 同步带统一：收到9F提前继续，否则0.5秒放行(2026-11-15统一, 原按目标0.75/1.5s) */
#define PLACE_ARM_WAIT_LIMIT_MS  2000u   /* RM放料对环/到站190°：等待4→2秒 */
#define PLACE_ARM_WAIT_SHORT_MS  1000u   /* RM取料/取回/收回：等待2→1秒 */
#define PLACE_ARM_WAIT_TARGET_MS 800u    /* RM取回对环：最多800ms */
#define PLACE_ARM_WAIT_RETBELT_MS 800u   /* RM取回收回：最多800ms */
#define PLACE_SETTLE_MS          100u   /* 放料轴全部到位后，保持夹紧等待机构稳定(2026-11-15, 原800ms) */
#define PLACE_AXIS_BELT          1u
#define PLACE_AXIS_LIFT          2u
#define PLACE_AXIS_ARM           4u
#define PLACE_AXES_ALL           (PLACE_AXIS_BELT | PLACE_AXIS_LIFT | PLACE_AXIS_ARM)
#define CHASSIS_HOLD_SETTLE_MS   100u    /* 原料/圆环站点：停车后等待100ms再锁存四轮位置 */
enum { PLACE_CHASSIS_HOLD, PLACE_PICK, PLACE_CLOSE, PLACE_PICK_LIFT, PLACE_DROP_BELT_ARM, PLACE_DROP_LIFT, PLACE_SETTLE, PLACE_OPEN,
       PLACE_LIFT_HOME, PLACE_BELT_HOME, PLACE_STORAGE_FINAL_RETURN,
       PLACE_RETRIEVE_LIFT100, PLACE_RETRIEVE_TARGET, PLACE_RETRIEVE_DROP_LIFT, PLACE_RETRIEVE_CLOSE,
       PLACE_RETRIEVE_LIFT50, PLACE_RETRIEVE_BELT_ARM, PLACE_RETRIEVE_SLOT_LIFT,
       PLACE_RETRIEVE_OPEN, PLACE_RETRIEVE_AFTER_OPEN_LIFT, PLACE_ROUTE_RESUME, PLACE_FINISHED };
typedef struct {
    uint8_t color[3], ring[3], slot[3];
    uint8_t pick, step, started, axes, belt_done, lift_done, arm_done, manual, retrieve; /* 仅粗加工取回 */
    uint8_t batch, storage; /* 站点标识: 二暂(batch==2且storage)专属参数用 */
    uint8_t chassis_held; /* 位置保持指令已下发；保持期间不能重复FE或重设当前位置 */
    uint16_t lift_wait_ms; /* 按本阶段丝杆绝对目标选择1.5秒或0.5秒，不按移动距离 */
    uint16_t belt_wait_ms; /* 本阶段同步带等待上限：默认500ms，取回对环700ms/收回800ms */
    uint32_t drop_lift_target; /* 本次放料高度：二轮暂存32000，其余56750 */
    TickType_t arm_wait_start; /* 放料RM对环计时起点(DROP_BELT_ARM进入), 总窗1.5s跨两步 */
    uint32_t encoder_sequence; /* 里程计始终读取；离站前检查新在线读数，不暂停/恢复任务 */
    TickType_t tick;
} StationPlace;
static StationPlace station_place;
static const struct { int32_t belt; int16_t arm; } place_ring_target[3] = {
    {5390, 306}, {7200, 182}, {8960, 313}   /* 当前圆环位置标定；放料及粗加工取回共用 */
};

static uint8_t      debug_rdk_center_sent = 0;
static TickType_t   debug_rdk_center_start_tick = 0;
static uint8_t       debug_rdk_center_settle_count = 0;
static float         debug_rdk_center_yaw = 0.0f;
static uint32_t      debug_rdk_center_last_seq = 0;
static float         debug_rdk_center_vx = 0.0f;
static float         debug_rdk_center_vy = 0.0f;
static volatile uint8_t debug_rdk_pos_active = 0;
static volatile int32_t debug_rdk_pos_hold = 0;
static int32_t        debug_rdk_pos_raw = 0;
static uint32_t       debug_rdk_pos_sequence = 0;
static TickType_t     debug_rdk_resume_start_tick = 0;
static uint8_t        debug_rdk_resume_action = 0; /*0中止,2no next,3回原pos*/
static volatile uint8_t debug_rdk_grab_done = 0;
static uint8_t station_debug_selection = 0; /* 22独立视觉取放：已保存物料/环号，不消费路线 */
static uint8_t debug_rdk_ring_serial = 0; /* UART7 standalone completion returns to raw. */
static uint8_t debug_rdk_stack_mode = 0; /* 当前视觉输入为码垛单中心点mode5（路线或1D调试） */
static TickType_t debug_rdk_arm_start_tick = 0;
static TickType_t debug_ring_arm_start_tick = 0; /* 四个圆环站点190°预定位，共用取放RM的2秒等待上限 */
/* 只保存驶向本站的准备，不改变底盘phase、坐标、PID或中断优先级。 */
static struct {
    uint8_t point, batch, belt_done, lift_done, lift_active;
    uint16_t x, y;
    TickType_t tick;
} debug_approach;
static TickType_t manual_arm_start_tick = 0;
static int8_t       debug_qr_scan_dir = 1;
static uint16_t     debug_qr_scan_seq = 0;
static TaskHandle_t qr_report_handle = NULL;  /*到扫码区创建/成功后删除的显示转发任务*/
#define DEBUG_QR_SCAN_COUNTS 300000  /*到站后沿原方向低速找码，最多约458mm*/
static void task_step_done(void);
static void turn_run_start(float start_yaw, float delta, uint8_t accumulate);
static uint8_t debug_leg_load(void);
static void debug_route_start(uint8_t selection);
static void debug_leg_start(void);
static void debug_motion_start(void);
static void debug_rdk_center_return_done(void);
static void belt_pos_goto(int32_t target);
static void station_place_fail(void);

uint8_t Motor_NoNext(void)
{
    return no_next_alarm;
}

const char *Motor_NoNextState(void)
{
    static const char *const names[] = {"", "LOAD", "MOTION", "ODOM", "FAULT"};
    return names[no_next_retry];
}

/* 一键启动(批次1/2/3, 3=连续两批): 0x13帧与PI7事件均由MotorTask执行。
   启动失败显示IMU/UPLOAD/ROUTE/BUSY原因，不能绕过原路线完整性检查。
   route_debug_take 成功即锁定路线缓存, 保证"连续跑两批"按键的一次性语义:
   启动后重复触发失败, 须06中止并重新上传路线后才可能再次启动。 */
static void motor_start_message(uint8_t message)
{
    start_message_tick = xTaskGetTickCount();
    start_message = message;
}

const char *Motor_StartMessage(void)
{
    static const char *const message[] = {"", "START KEY WAIT", "START BUSY", "START IMU WAIT",
        "START UPLOAD WAIT", "START NEED ROUTE", "START ACCEPTED"};
    uint8_t current;
    taskENTER_CRITICAL();
    current = (xTaskGetTickCount() - start_message_tick < pdMS_TO_TICKS(5000)) ? start_message : START_MSG_NONE;
    taskEXIT_CRITICAL();
    return message[current];
}

void Motor_RequestDebugRun(uint8_t selection)
{
    if (selection < 1u || selection > 3u) return;
    taskENTER_CRITICAL();
    if (!key_start_request)
    {
        key_start_tick = xTaskGetTickCount();
        key_start_request = selection;
        motor_start_message(START_MSG_QUEUED);
    }
    taskEXIT_CRITICAL();
}

void Motor_DebugRunStart(uint8_t selection)
{
    if (!PHASE_CAN_START(phase) || debug_route_active || debug_pause_active)
    { motor_start_message(START_MSG_BUSY); return; }
    if (IMU_ConfigStatus() != IMU_CONFIG_READY)
    { motor_start_message(START_MSG_IMU); return; }
    if (route_upload_processing())
    { motor_start_message(START_MSG_UPLOAD); return; }
    if (!route_debug_take(selection))
    { motor_start_message(START_MSG_ROUTE); return; }
    motor_start_message(START_MSG_RUNNING);
    array_mode = 0;
    leg1_next_valid = 0;
    NVIC_SetPriority(UART7_IRQn, IRQ_PRIO_UART7_IDLE); /*一键启动后不再收路线, UART7让路*/
    debug_route_start(selection);
}

static void motor_key_start_poll(void)
{
    uint8_t selection;
    TickType_t requested;
    taskENTER_CRITICAL();
    selection = key_start_request;
    requested = key_start_tick;
    taskEXIT_CRITICAL();
    if (!selection) return;
    /* 只容忍按键恰好抢占最后ACK/IMU就绪的短暂窗口，不在未来上传完后突然发车。 */
    if (PHASE_CAN_START(phase) && !debug_route_active && !debug_pause_active &&
        (route_upload_processing() || IMU_ConfigStatus() != IMU_CONFIG_READY) &&
        xTaskGetTickCount() - requested < pdMS_TO_TICKS(3000))
    {
        motor_start_message(route_upload_processing() ? START_MSG_UPLOAD : START_MSG_IMU);
        return;
    }
    taskENTER_CRITICAL();
    key_start_request = 0;
    taskEXIT_CRITICAL();
    Motor_DebugRunStart(selection); /* 网页13/03和PI7在同一任务调用同一入口。 */
}

/* 到站：Encoder让路15；HWT101全程保持4。
   轮趣保留原圆环/暂存站降15策略，原料区IMU不降级。 */
static void irq_station_enter_raw(void)
{
    NVIC_SetPriority(USART2_IRQn, IRQ_PRIO_ENC_LOW);
}

static void irq_station_enter(void)
{
    NVIC_SetPriority(USART2_IRQn, IRQ_PRIO_ENC_LOW);
    if (IMU_Source() != IMU_SOURCE_HWT101)
        NVIC_SetPriority(USART6_IRQn, IRQ_PRIO_IMU_LOW);
}

/* 离站(回程/续路开始前调用): 恢复行车档 Encoder5 IMU4——行驶闭环依赖两者 */
static void irq_station_leave(void)
{
    NVIC_SetPriority(USART2_IRQn, IRQ_PRIO_ENC_RUN);
    NVIC_SetPriority(USART6_IRQn, IRQ_PRIO_IMU_RUN);
}

uint8_t Motor_RunFault(void)
{
    return run_fault;
}

uint8_t Motor_OdomWaiting(void)
{
    return odom_wait_active;
}

void Motor_RunGetFaultSnapshot(MotorRunFaultSnapshot_t *out)
{
    if (!out) return;
    taskENTER_CRITICAL();
    *out = run_fault_snapshot;
    taskEXIT_CRITICAL();
}

uint8_t Motor_GrabGetRegion(MotorGrabRegion_t *out)
{
    uint8_t active;
    if (!out) return 0;
    taskENTER_CRITICAL();
    active = (uint8_t)(phase == PHASE_RDK_GRAB_REGION);
    if (active) *out = grab_region;
    else memset(out, 0, sizeof(*out));
    taskEXIT_CRITICAL();
    return active;
}

static void grab_region_clear(void)
{
    taskENTER_CRITICAL();
    memset(&grab_region, 0, sizeof(grab_region));
    memset(&raw_region, 0, sizeof(raw_region));
    taskEXIT_CRITICAL();
    RDK_GrabArmEnable(0, 0);
}

void Motor_GrabGetFault(MotorGrabFault_t *out)
{
    taskENTER_CRITICAL();
    *out = grab_fault;
    taskEXIT_CRITICAL();
}

static uint8_t grab_arm_done(void);

void Motor_GrabGetStatus(MotorGrabStatus_t *out)
{
    if (!out) return;
    taskENTER_CRITICAL();
    memset(out, 0, sizeof(*out));
    out->vision_tx = (uint8_t)(debug_rdk_center_active && debug_rdk_center_sent);
    if (phase == PHASE_RDK_ARM_PRE || phase == PHASE_RDK_ARM_POST)
    {
        out->active = out->started = 1;
        out->stage = phase == PHASE_RDK_ARM_PRE ? 1u : 2u;
        out->axes = MOTOR_GRAB_WAIT_RM;
        if (!grab_arm_done())
            out->waiting |= MOTOR_GRAB_WAIT_RM;
        if (phase == PHASE_RDK_ARM_PRE)
        {
            out->axes |= MOTOR_GRAB_WAIT_BELT;
            out->belt_target = RDK_RAW_BELT_POS;
            if (!grab_parallel_belt_done && !Belt_ZDT_ReachedGet())
                out->waiting |= MOTOR_GRAB_WAIT_BELT;
            out->axes |= MOTOR_GRAB_WAIT_LIFT;
            out->lift_target = -(int32_t)RDK_RAW_PRE_LIFT_POS;
            if (!grab_parallel_lift_done && !Lift_ZDT_ReachedGet())
                out->waiting |= MOTOR_GRAB_WAIT_LIFT;
        }
        out->elapsed_ms = (uint32_t)((uint64_t)(xTaskGetTickCount() - grab_step_tick) *
                                    1000u / configTICK_RATE_HZ);
        if (Lift_ZDT_ReachedGet()) out->feedback |= MOTOR_GRAB_WAIT_LIFT;
        if (Belt_ZDT_ReachedGet()) out->feedback |= MOTOR_GRAB_WAIT_BELT;
    }
    if (phase == PHASE_RDK_GRAB_REGION && raw_region.step < RAW_REGION_STEP_COUNT)
    {
        const RawRegionStep *step = &raw_region_steps[raw_region.step];
        out->active = 1;
        out->stage = 3;
        out->region = grab_region.region;
        out->pick = grab_region.pick;
        out->step = (uint8_t)(raw_region.step + 1u);
        out->started = raw_region.started;
        out->servo = (uint8_t)!step->axes;
        out->axes = raw_region.started ? raw_region.axes :
            (raw_region.step == RAW_REGION_TARGET ? raw_region_target[grab_region.region - 1u].axes : step->axes);
        out->waiting = (uint8_t)(out->axes & ~raw_region.done);
        out->belt_target = raw_region.belt_target;
        out->lift_target = -(int32_t)raw_region.lift_target;
        out->elapsed_ms = (uint32_t)((uint64_t)(xTaskGetTickCount() - grab_step_tick) *
                                    1000u / configTICK_RATE_HZ);
        if (Lift_ZDT_ReachedGet()) out->feedback |= MOTOR_GRAB_WAIT_LIFT;
        if (Belt_ZDT_ReachedGet()) out->feedback |= MOTOR_GRAB_WAIT_BELT;
    }
    if (phase == PHASE_RDK_GRAB_SEQ && grab_step_index < GRAB_STEP_NUM)
    {
        const GrabStep *st = &grab_steps[grab_step_index];
        uint8_t pick = grab_pick_fixed != 0xFFu ? grab_pick_fixed : grab_pick_index;
        out->active = 1;
        out->pick = (uint8_t)(pick + 1u);
        out->step = (uint8_t)(grab_step_index + 1u);
        out->started = grab_sub;
        out->servo = (uint8_t)(st->type == GRAB_ST_SERVO180 || st->type == GRAB_ST_SERVO270);
        if (grab_sub)
            out->elapsed_ms = (uint32_t)((uint64_t)(xTaskGetTickCount() - grab_step_tick) *
                                        1000u / configTICK_RATE_HZ);
        if (st->type == GRAB_ST_LIFT || st->type == GRAB_ST_BELT_SERVO270_ARM)
        {
            out->axes |= MOTOR_GRAB_WAIT_LIFT;
            out->lift_target = st->type == GRAB_ST_LIFT ? -st->target :
                -(int32_t)(pick == 2u ? RDK_RAW_FINISH_LIFT_POS : RDK_RAW_LIFT_POS);
            if (!grab_parallel_lift_done && !Lift_ZDT_ReachedGet())
                out->waiting |= MOTOR_GRAB_WAIT_LIFT;
        }
        if (st->type == GRAB_ST_BELT || st->type == GRAB_ST_BELT_ARM ||
            st->type == GRAB_ST_BELT_SERVO270_ARM)
        {
            out->axes |= MOTOR_GRAB_WAIT_BELT;
            out->belt_target = st->type == GRAB_ST_BELT_SERVO270_ARM && pick == 2u ?
                               BELT_POS_GRAB_PUL : st->target;
            if (!grab_parallel_belt_done && !Belt_ZDT_ReachedGet())
                out->waiting |= MOTOR_GRAB_WAIT_BELT;
        }
        if (st->type == GRAB_ST_ARM || st->type == GRAB_ST_BELT_ARM ||
            st->type == GRAB_ST_BELT_SERVO270_ARM)
        {
            out->axes |= MOTOR_GRAB_WAIT_RM;
            if (!grab_arm_done())
                out->waiting |= MOTOR_GRAB_WAIT_RM;
        }
        if (Lift_ZDT_ReachedGet()) out->feedback |= MOTOR_GRAB_WAIT_LIFT;
        if (Belt_ZDT_ReachedGet()) out->feedback |= MOTOR_GRAB_WAIT_BELT;
    }
    taskEXIT_CRITICAL();
}

void Motor_RDK_GrabDone(void)
{
    debug_rdk_grab_done = 1;
}

typedef enum { POS_RUNNING, POS_REACHED, POS_FAILED, POS_PAUSED } PosRunState;

static void odom_motion_pause(TickType_t now, uint32_t sequence)
{
    if (odom_wait_active) return;
    odom_wait_active = 1;
    odom_wait_phase = phase;
    odom_wait_tick = now;
    odom_wait_sequence = sequence;
    run_fault = MOTOR_RUN_FAULT_ODOM;
    Motor_Stop();
}

/* 返回1时仅暂停当前动作，保持目标/路线游标/批次/相对起点；绝不释放上传路线。
   RTOS每10ms检查，编码器仍连续读取。恢复只认新的连续有效回包，不认旧ON标志。 */
static uint8_t odom_motion_wait(void)
{
    OdomData_t o;
    uint32_t sequence;
    TickType_t now = xTaskGetTickCount();
    if (phase != PHASE_LINE && phase != PHASE_HOME && phase != PHASE_SLOW &&
        phase != PHASE_DIAG && phase != PHASE_RDK_RETURN)
    {
        odom_wait_active = 0;
        return 0;
    }
    sequence = odometry_snapshot(&o);
    if (odom_wait_active && odom_wait_phase != phase) odom_wait_active = 0;
    if (!odom_wait_active)
    {
        if (o.online[ODOM_POS_AXIS]) return 0;
        odom_motion_pause(now, sequence);
        return 1;
    }
    if (!o.online[ODOM_POS_AXIS] || sequence == odom_wait_sequence ||
        o.valid_streak < ODOM_RESUME_VALID_FRAMES) return 1;

    /* 保留暂停前已用的总运行时间，只扣掉停等时间；进度检查从当前值重建。
       不能调用pos_run_start，否则会重置原始误差方向，影响跨目标结束判断。 */
    posrun_start += now - odom_wait_tick;
    pos_ramp_start = now;
    pos_progress_tick = now;
    pos_best_error = fabsf((float)pos_target - (float)o.enc_pos[ODOM_POS_AXIS]);
    pos_speed_last = 0.0f;
    pos_finish_active = 0;
    PID_Init(&PosPID);
    PID_Init(&YawPID);
    line_yaw_reset();
    odom_wait_active = 0;
    run_fault = MOTOR_RUN_FAULT_NONE;
    return 0;
}

/* 绝对帧直接使用累计目标；相对帧只在启动时加一次当前值。 */
static void pos_run_start(int32_t current)
{
    run_fault = MOTOR_RUN_FAULT_NONE;
    no_next_alarm = 0;
    pos_initial_error = (float)pos_target - (float)current;
    pos_best_error = fabsf(pos_initial_error);
    pos_progress_tick = posrun_start = xTaskGetTickCount();
    pos_ramp_start = posrun_start;
    odom_wait_active = 0;
    pos_speed_last = 0.0f;
    pos_finish_active = 0;
}

static float pos_speed_limit(float output, float error_abs)
{
    TickType_t elapsed = xTaskGetTickCount() - pos_ramp_start;
    TickType_t ramp_ticks = pdMS_TO_TICKS(POS_START_RAMP_MS);
    float limit = POS_OUT_MAX_RPM;

    if (elapsed < ramp_ticks)
        limit = POS_START_RPM + (POS_OUT_MAX_RPM - POS_START_RPM) *
                (float)elapsed / (float)ramp_ticks;
    if (error_abs < POS_DECEL_COUNTS)
    {
        float ratio = (error_abs - POS_QR_SLOW_COUNTS) /
                      (POS_DECEL_COUNTS - POS_QR_SLOW_COUNTS);
        float approach_limit;
        if (ratio < 0.0f) ratio = 0.0f;
        /*两端斜率为零，平滑衔接巡航和最终低速；与起步上限取较小值。*/
        approach_limit = POS_APPROACH_RPM +
                         (POS_OUT_MAX_RPM - POS_APPROACH_RPM) *
                         ratio * ratio * (3.0f - 2.0f * ratio);
        if (limit > approach_limit) limit = approach_limit;
    }
    if (error_abs <= POS_QR_SLOW_COUNTS)
    {
        float ratio = (error_abs - POS_TOL_COUNTS) /
                      (POS_QR_SLOW_COUNTS - POS_TOL_COUNTS);
        float final_limit;
        if (ratio < 0.0f) ratio = 0.0f;
        final_limit = POS_FINAL_RPM + (POS_APPROACH_RPM - POS_FINAL_RPM) *
                      ratio * ratio * (3.0f - 2.0f * ratio);
        if (limit > final_limit) limit = final_limit;
    }
    /*空间限速始终是硬上限，不能为平滑指令在接近终点时抬高速度。*/
    if (output > limit) output = limit;
    if (output < -limit) output = -limit;
    /*位置PID的输出可能因量化反馈在相邻周期突降；只限制同向减速，
       加速仍由原位置环决定，反向和异常情况交给上层保护立即处理。*/
    if (pos_speed_last != 0.0f && output * pos_speed_last > 0.0f &&
        fabsf(output) < fabsf(pos_speed_last))
    {
        float max_drop = POS_DECEL_SLEW_RPM_S * 0.01f;
        if (fabsf(pos_speed_last) - fabsf(output) > max_drop)
            output = pos_speed_last > 0.0f ? pos_speed_last - max_drop :
                                             pos_speed_last + max_drop;
    }
    pos_speed_last = output;
    return output;
}

/*正常低速到位的短收尾；在线/超时检查仍在pos_run_check优先执行。*/
static uint8_t pos_finish_soft(int32_t current)
{
    float error = (float)pos_target - (float)current;
    TickType_t elapsed;
    float rpm;
    if ((pos_initial_error > 0.0f && error <= 0.0f) ||
        (pos_initial_error < 0.0f && error >= 0.0f) ||
        fabsf(pos_speed_last) > POS_APPROACH_RPM || pos_speed_last == 0.0f)
        return 1; /*跨目标/反馈突变/本来就在目标：立即完成，绝不继续滑行*/
    if (!pos_finish_active)
    {
        pos_finish_active = 1;
        pos_finish_tick = xTaskGetTickCount();
        }
    elapsed = xTaskGetTickCount() - pos_finish_tick;
    if (elapsed >= pdMS_TO_TICKS(POS_FINISH_MS))
    {
        pos_finish_active = 0;
        return 1;
    }
    rpm = pos_speed_last * (1.0f - (float)elapsed / (float)pdMS_TO_TICKS(POS_FINISH_MS));
    Motor_Move(0, line_rpm_round(rpm), 0);
    return 0;
}

static PosRunState pos_run_check(int32_t current, uint8_t online)
{
    TickType_t now = xTaskGetTickCount();
    float err = (float)pos_target - (float)current;
    float aerr = fabsf(err);

    if (!online)
    {
        /* 顶层检查后刚好掉线的快照竞争：本拍即锁存暂停，恢复仍需3帧。 */
        odom_motion_pause(now, odometry_encoder_sequence());
        return POS_PAUSED;
    }
    if ((now - posrun_start) >= pdMS_TO_TICKS(POS_RUN_TIMEOUT_MS))
    {
        run_fault = MOTOR_RUN_FAULT_TIMEOUT;
        return POS_FAILED;
    }
    /* 低速到位或跨过目标即停，避免采样跳过容差带后反复追赶。 */
    if (pos_finish_active || aerr <= POS_TOL_COUNTS ||
        (pos_initial_error > 0.0f && err <= 0.0f) ||
        (pos_initial_error < 0.0f && err >= 0.0f))
        return POS_REACHED;
    if (aerr <= pos_best_error - POS_PROGRESS_COUNTS)
    {
        pos_best_error = aerr;
        pos_progress_tick = now;
    }
    else if ((now - pos_progress_tick) >= pdMS_TO_TICKS(POS_PROGRESS_MS))
    {
        /* 先发布快照再置故障，LCD不会把停车后的静止值当作触发时数据。 */
        taskENTER_CRITICAL();
        run_fault_snapshot.current = current;
        run_fault_snapshot.target = pos_target;
        run_fault_snapshot.error = (int32_t)err;
        run_fault_snapshot.progress = (int32_t)(pos_best_error - aerr);
        run_fault_snapshot.wait_ms = (uint32_t)((uint64_t)(now - pos_progress_tick) *
                                               1000u / configTICK_RATE_HZ);
        run_fault_snapshot.command_rpm = (int16_t)pos_speed_last;
        run_fault_snapshot.phase = (uint8_t)phase;
        run_fault = MOTOR_RUN_FAULT_PROGRESS;
        taskEXIT_CRITICAL();
        return POS_FAILED;
    }
    return POS_RUNNING;
}

static void debug_approach_stop(void)
{
    if (!debug_approach.point) return;
    if (debug_approach.lift_active) Emm_V5_Stop_Now(LIFT_ZDT_ADDR, false);
    debug_approach.point = 0;
    Emm_V5_Stop_Now(BELT_ZDT_ADDR, false);
    Arm_M2006_Stop();
}

static void debug_route_release(void)
{
    debug_approach_stop();
    if (debug_route_active) route_debug_release(debug_route_batch);
    if (debug_route_next_batch) route_debug_release(debug_route_next_batch);
    debug_route_next_batch = 0;
}

/* 退出圆环/码垛视觉；里程计全程连续读取。 */
static void debug_rdk_ring_stop(void)
{
    station_debug_selection = 0;
    debug_rdk_ring_serial = 0;
    debug_rdk_stack_mode = 0;
    RingAlign_Stop();
    if (RDK_Ring_Active() || RDK_Stack_Active()) RDK_Center_SendIdle();
}

static void pos_run_abort(void)
{
    grab_region_clear();
    key_start_request = 0;
    no_next_retry = NEXT_RETRY_NONE;
    uint8_t resume_rdk_pos = debug_rdk_pos_active;
    odom_wait_active = 0;
    debug_rdk_ring_stop();
    Arm_M2006_Stop();
    debug_rdk_standalone = 0;
    array_mode = 0;
    debug_route_release();
    debug_route_active = 0;
    debug_route_step = 0;
    if (debug_pause_active) Buzzer_Off();
    debug_pause_active = 0;
    debug_pause_next = 0;
    debug_qr_scan_active = 0;
    if (debug_rdk_center_active || debug_rdk_center_sent)
        RDK_Center_SendIdle();
    debug_rdk_center_active = 0;
    debug_rdk_center_sent = 0;
    debug_rdk_center_settle_count = 0;
    debug_rdk_center_vx = 0.0f;
    debug_rdk_center_vy = 0.0f;
    if (debug_rdk_pos_active)
    {
        debug_rdk_pos_sequence = odometry_encoder_sequence();
        debug_rdk_resume_start_tick = xTaskGetTickCount();
        debug_rdk_resume_action = 0;
    }
    RDK_Center_Clear();
    RDK_Center_Enable(0);
    debug_turn_pos_active = 0;
    leg1_next_valid = 0;
    phase = resume_rdk_pos ? PHASE_RDK_RESUME : PHASE_IDLE;
    Motor_Stop();
}

/* 各斜走入口统一捕获起点和航向，避免沿用上一段的状态。 */
static void diag_run_start(uint8_t direction, int32_t distance, float heading)
{
    OdomData_t o;
    run_fault = MOTOR_RUN_FAULT_NONE;
    odometry_get(&o);
    no_next_alarm = 0;
    diag_start = o.enc_pos[ODOM_POS_AXIS];
    leg_dir = direction;
    leg1_end = distance;
    yaw_hold = heading;
    posrun_start = xTaskGetTickCount();
    pos_ramp_start = posrun_start;
    odom_wait_active = 0;
    PID_Init(&YawPID);
    line_yaw_reset();
    YawPID.Kp = YAW_PID_KP;  YawPID.Ki = YAW_PID_KI;  YawPID.Kd = YAW_PID_KD;
    YawPID.ErrorIntMax = YAW_ERR_INT_MAX;  YawPID.ErrorIntMin = -YAW_ERR_INT_MAX;
    YawPID.OutMax = YAW_OUT_MAX_RPM;  YawPID.OutMin = -YAW_OUT_MAX_RPM;
    phase = PHASE_DIAG;
}

/* 到点蜂鸣时长: 0=取消到点蜂鸣, 停车后下一拍(10ms)直接衔接下一步(2026-11-15)。
   no next 的 5000ms 故障蜂鸣与独立调试/取回串行完成的 500ms 收尾蜂鸣不受影响。 */
#define DEBUG_POINT_BEEP_MS      0u

static void debug_point_pause_start(uint8_t next_step, uint32_t duration_ms)
{
    Motor_Stop();
    if (duration_ms) Buzzer_On();   /*0ms时完全无声, 直接衔接*/

    debug_pause_active = 1;
    debug_pause_next = next_step;
    debug_pause_until = xTaskGetTickCount() + pdMS_TO_TICKS(duration_ms);
}

static void debug_no_next_start(void)
{
    grab_region_clear();
    debug_approach_stop();
    if (debug_route_active)
    {
        no_next_retry = NEXT_RETRY_FAULT;
        no_next_retry_tick = xTaskGetTickCount();
    }
    if (phase != PHASE_RDK_GRAB_SEQ && phase != PHASE_RDK_GRAB_REGION && phase != PHASE_RDK_ARM_POST &&
        phase != PHASE_RDK_ARM_PRE && phase != PHASE_RDK_GRAB_TRIG &&
        phase != PHASE_RDK_CHASSIS_HOLD)
        grab_fault.reason = MOTOR_GRAB_FAULT_NONE;
    debug_rdk_ring_stop();
    if (phase == PHASE_RDK_ARM_EXTEND || phase == PHASE_RDK_ARM_PRE ||
        phase == PHASE_RDK_ARM_POST || phase == PHASE_RDK_GRAB_SEQ || phase == PHASE_RDK_GRAB_REGION ||
        phase == PHASE_RDK_CHASSIS_HOLD)
        Arm_M2006_Stop();
    raw_debug_full = 0;
    debug_rdk_standalone = 0;
    no_next_alarm = 1;
    if (debug_rdk_center_active || debug_rdk_pos_active)
    {
        Motor_Stop();
        if (debug_rdk_center_active || debug_rdk_center_sent)
            RDK_Center_SendIdle();
        debug_rdk_center_active = 0;
        debug_rdk_center_sent = 0;
        debug_rdk_center_settle_count = 0;
        debug_rdk_center_vx = 0.0f;
        debug_rdk_center_vy = 0.0f;
        RDK_Center_Clear();
        RDK_Center_Enable(0);
        if (debug_rdk_pos_active)
        {
            debug_rdk_pos_sequence = odometry_encoder_sequence();
            debug_rdk_resume_start_tick = xTaskGetTickCount();
            debug_rdk_resume_action = 2;
            phase = PHASE_RDK_RESUME;
            debug_point_pause_start(0, 5000);
            return;
        }
    }
    debug_point_pause_start(0, 5000);
}

/* 只登记原操作的重试点，不能从头重做夹放，也不能再消费一次已加载路段。 */
static void debug_no_next_retry_start(uint8_t operation)
{
    debug_no_next_start();
    if (debug_route_active) no_next_retry = operation;
}

/* 原料区驶入与独立1E共用；只发100脉冲绝对目标，不回零、不等待。 */
static void debug_raw_lift_prepare(void)
{
    Lift_ZDT_ReachedClear();
    Emm_V5_Pos_Control(LIFT_ZDT_ADDR, LIFT_POS_DOWN_DIR,
                       LIFT_POS_SPEED_RPM, LIFT_POS_ACC,
                       RDK_RAW_PRE_LIFT_POS, true, false);
}

/* 每个运动阶段重新锁存各轴到位，避免先到位的轴被其他轴覆盖。 */
static void grab_wait_start(void)
{
    grab_step_tick = xTaskGetTickCount();
    grab_parallel_belt_done = grab_parallel_lift_done = 0;
}

static void grab_record_fault(uint8_t reason)
{
    taskENTER_CRITICAL();
    grab_fault.pick = (uint8_t)(1u + ((grab_pick_fixed != 0xFFu) ?
                                     grab_pick_fixed : grab_pick_index));
    if (grab_fault.pick > 3u) grab_fault.pick = 3u;
    grab_fault.step = phase == PHASE_RDK_GRAB_REGION ? (uint8_t)(raw_region.step + 1u) :
        ((phase == PHASE_RDK_GRAB_SEQ) ? (uint8_t)(grab_step_index + 1u) : 0u);
    grab_fault.reason = reason;
    if (reason == MOTOR_GRAB_FAULT_ARM)
    {
        /* RM故障快照: 实际角/目标角/反馈有效——区分"没到位"与"反馈丢失" */
        grab_fault.rm_angle = (int16_t)Arm_M2006_GetAngle();
        grab_fault.rm_target = (int16_t)Arm_M2006_GetTargetDeg();
        grab_fault.rm_ready = Arm_M2006_GetReady();
    }
    taskEXIT_CRITICAL();
}

static void grab_fail(uint8_t reason)
{
    grab_record_fault(reason);
    Emm_V5_Stop_Now(BELT_ZDT_ADDR, false);
    Emm_V5_Stop_Now(LIFT_ZDT_ADDR, false);
    Arm_M2006_Stop();
    debug_no_next_start();
}

/* 路线、1E和带任务码的1A共用：先开接收再发一次颜色，
   首个匹配READY只锁存区域并等机构参数；不重发颜色，ACK仅诊断。 */
static void grab_color_start(uint8_t pick)
{
    uint8_t color = UART8_Color(debug_route_batch, pick);

    if (!UART8_Valid() || g_link.bad)
    {
        grab_fail(MOTOR_GRAB_FAULT_RDK);
        return;
    }
    grab_region_clear();
    taskENTER_CRITICAL();
    grab_region.batch = debug_route_batch;
    grab_region.pick = (uint8_t)(pick + 1u); /* 保存实际发出的块序，不从环号或旧手动状态推算。 */
    taskEXIT_CRITICAL();
    grab_expect_color = color;
    grab_trig_tick = xTaskGetTickCount();
    RDK_GrabArmEnable(1, color);   /*先开通道，避免RDK快速回复的ACK/状态帧被丢弃。*/
    if (!RDK_Center_SendGrabColor(color))
    {
        RDK_GrabArmEnable(0, 0);
        grab_fail(MOTOR_GRAB_FAULT_RDK);
        return;
    }
    phase = PHASE_RDK_GRAB_TRIG;
}

/* 原料区轴等待：仅9F主动到位帧；丝杆1秒/带2秒未确认则锁存放行。不查询位置。
   放行只结束本轴等待，不发送停止命令；RM使用独立1.5秒放行条件。 */
static uint8_t grab_axis_done(uint8_t lift)
{
    TickType_t elapsed = xTaskGetTickCount() - grab_step_tick;
    uint8_t *done = lift ? &grab_parallel_lift_done : &grab_parallel_belt_done;
    TickType_t wait_limit = pdMS_TO_TICKS(lift ? RAW_LIFT_WAIT_LIMIT_MS : RAW_BELT_WAIT_LIMIT_MS);

    *done |= lift ? Lift_ZDT_ReachedGet() : Belt_ZDT_ReachedGet();
    if (!*done && elapsed >= wait_limit)
        *done = 1; /*用户指定超时放行；不能据此认定机械实际到位。*/
    return *done;
}

/* 原料区共用：反馈正常且到位或本组已满1.5秒即可推进；反馈失效不允许超时放行。
   不停止RM控制器、不重发目标；放行不代表实际角度已到位。 */
static uint8_t grab_arm_done(void)
{
    return (uint8_t)(Arm_M2006_Ready() &&
        (Arm_M2006_AtTargetWithin(RAW_ARM_POS_TOL_DEG) ||
         xTaskGetTickCount() - grab_step_tick >= pdMS_TO_TICKS(RAW_ARM_WAIT_LIMIT_MS)));
}

static uint8_t grab_parallel_belt_arm_done(void)
{
    if (!Arm_M2006_Ready())
    {
        grab_fail(MOTOR_GRAB_FAULT_ARM);
        return 0;
    }
    return (uint8_t)(grab_axis_done(0) &&
                     grab_arm_done());
}

/* 一个物料放入车载槽且完成返回之后，原路线/1E/1A共用续接。 */
static void grab_pick_complete(void)
{
    UART8_GrabOk();   /*一块夹取序列完成(夹紧+抬到位): 抓取正确+1*/
    if (grab_manual)
    {
        /* 网页/串口手动触发: 00模式连续夹完本批剩余的块(1→2→3)后停车,
           01~03固定块序只跑一次。每块衔接同样经RDK(发颜色等触发帧,
           30s超时no next), 任务码未注入时退回直接夹取。 */
        if (grab_pick_fixed == 0xFFu)
        {
            grab_pick_index++;
            grab_step_index = 0;
            grab_sub = 0;
            if (grab_pick_index < 3u)
            {
                uint8_t pick = (grab_pick_index < 3u) ?
                               grab_pick_index : 2u;
                if (UART8_Valid() && !g_link.bad)
                    grab_color_start(pick);
                else if (phase == PHASE_RDK_GRAB_REGION)
                    grab_fail(MOTOR_GRAB_FAULT_RDK); /* 新序列不能停在完成步反复计数。 */
            }
            else
                phase = PHASE_DONE;
        }
        else
            phase = PHASE_DONE;
    }
    else
    {
        grab_pick_index++;         /*本块序列完成, 下一次用下一块参数*/
        debug_rdk_grab_done = 0;
        if (grab_pick_index < 3u)
        {
            /* 第二/三块与第一块一样：发送本块颜色后重新开接收，
               清除旧请求/半帧，等RDK首个有效触发帧再夹取。 */
            grab_color_start(grab_pick_index);
        }
        else if (raw_debug_full)
        {
            raw_debug_full = 0;
            RDK_Center_SendIdle();
            debug_rdk_pos_active = 0;
            phase = PHASE_DONE; /*独立网页完成：保持收起，不返回保存位置或续路线*/
            debug_point_pause_start(0, DEBUG_POINT_BEEP_MS);
        }
        else if (grab_trig_auto)
        {
            /* 第三块(RDK触发流程): 完成后不等夹取通知, 直接回程走下一段。 */
            irq_station_leave();   /*回程pos/yaw闭环依赖Encoder/IMU, 先恢复行车档*/
            debug_rdk_pos_sequence = odometry_encoder_sequence();
            debug_rdk_resume_start_tick = xTaskGetTickCount();
            debug_rdk_resume_action = 3;
            phase = PHASE_RDK_RESUME;
        }
        else
            phase = PHASE_RDK_GRAB_WAIT;   /*RDK触发: 等待夹取完成通知*/
    }
}

static void raw_region_start(void)
{
    if (!grab_manual)
    {
        grab_pick_fixed = 0xFFu;
        grab_trig_auto = 1;
    }
    if (grab_region.region < RDK_GRAB_REGION_NEAR || grab_region.region > RDK_GRAB_REGION_RIGHT ||
        grab_region.pick < 1u || grab_region.pick > 3u)
    {
        grab_fail(MOTOR_GRAB_FAULT_RDK);
        return;
    }
    memset(&raw_region, 0, sizeof(raw_region));
    /* 先确认当前物料对应的车载槽，区域号不能代替槽位序号。舵机不等待。 */
    Servo270_SetAngle(grab_pick_servo270_drop[grab_region.pick - 1u]);
    phase = PHASE_RDK_GRAB_REGION;
}

static void raw_region_run(void)
{
    const RawRegionStep *step;
    TickType_t elapsed;
    if (!raw_chassis_held) Motor_Stop();
    if (!Arm_M2006_Ready())
    {
        grab_fail(MOTOR_GRAB_FAULT_ARM);
        return;
    }
    if (raw_region.step >= RAW_REGION_STEP_COUNT)
    {
        grab_pick_complete();
        return;
    }
    step = &raw_region_steps[raw_region.step];
    if (!raw_region.started)
    {
        raw_region.started = 1;
        raw_region.done = 0;
        raw_region.axes = step->axes;
        raw_region.belt_target = step->belt;
        raw_region.lift_target = step->lift;
        raw_region.arm_target = step->arm;
        if (raw_region.step == RAW_REGION_TARGET)
        {
            uint8_t region = (uint8_t)(grab_region.region - 1u);
            raw_region.axes = raw_region_target[region].axes;
            raw_region.belt_target = raw_region_target[region].belt;
            raw_region.arm_target = raw_region_target[region].arm;
        }
        /* 第三块共用原离站收起目标；前两块返回7100/2000/435继续等颜色。 */
        if (raw_region.step == RAW_REGION_RETURN && grab_region.pick == 3u)
        {
            raw_region.belt_target = BELT_POS_GRAB_PUL;
            raw_region.lift_target = RDK_RAW_FINISH_LIFT_POS;
            raw_region.arm_target = RDK_RAW_FINISH_ARM_DEG;
        }
        grab_wait_start();
        if (!raw_region.axes) Servo_SetAngle(step->claw);
        else
        {
            if ((raw_region.axes & MOTOR_GRAB_WAIT_RM) &&
                !Arm_M2006_SetTargetDeg(raw_region.arm_target))
            {
                grab_fail(MOTOR_GRAB_FAULT_ARM);
                return;
            }
            if (raw_region.axes & MOTOR_GRAB_WAIT_BELT) belt_pos_goto(raw_region.belt_target);
            if (raw_region.axes & MOTOR_GRAB_WAIT_LIFT)
            {
                Lift_ZDT_ReachedClear();
                Emm_V5_Pos_Control(LIFT_ZDT_ADDR, LIFT_POS_DOWN_DIR,
                                   LIFT_POS_SPEED_RPM, LIFT_POS_ACC,
                                   raw_region.lift_target, true, false);
            }
        }
        if (raw_region.step == RAW_REGION_RETURN && grab_region.pick < 3u)
            Servo270_SetAngle(grab_pick_servo270[grab_region.pick - 1u]); /* 前两块仍立即换槽。 */
        return;
    }
    elapsed = xTaskGetTickCount() - grab_step_tick;
    if (raw_region.axes & MOTOR_GRAB_WAIT_BELT)
        if (Belt_ZDT_ReachedGet()) raw_region.done |= MOTOR_GRAB_WAIT_BELT;
    if (raw_region.axes & MOTOR_GRAB_WAIT_LIFT)
        if (Lift_ZDT_ReachedGet()) raw_region.done |= MOTOR_GRAB_WAIT_LIFT;
    if (raw_region.axes & MOTOR_GRAB_WAIT_RM)
        if (Arm_M2006_AtTargetWithin(RAW_ARM_POS_TOL_DEG)) raw_region.done |= MOTOR_GRAB_WAIT_RM;
    /* 第三块三轴收起下发满2秒才转A槽；仅结束本任务当前阶段的等待，不阻塞RTOS。 */
    if (elapsed < pdMS_TO_TICKS(step->wait_ms) &&
        (step->fixed_wait || (raw_region.step == RAW_REGION_RETURN && grab_region.pick == 3u) ||
         (raw_region.done & raw_region.axes) != raw_region.axes)) return;
    if (raw_region.step == RAW_REGION_RETURN && grab_region.pick == 3u)
        Servo270_SetAngle(grab_pick_servo270[grab_region.pick - 1u]); /* 只下发一次，不等舵机到位。 */
    raw_region.step++;
    if (raw_region.step == RAW_REGION_RIGHT_ARM && grab_region.region != RDK_GRAB_REGION_RIGHT)
        raw_region.step++; /* 中下/左上不下发450°，直接抬丝杆。 */
    raw_region.started = 0;
}

static void debug_rdk_center_return_done(void)
{
    Motor_Stop();
    debug_rdk_pos_active = 0;
    if (debug_leg_load())
    {
        if (debug_route_leg.station) debug_leg_start();
        else debug_point_pause_start(1, DEBUG_POINT_BEEP_MS);
    }
    else
    {
        debug_no_next_retry_start(NEXT_RETRY_LOAD);
        grab_record_fault(MOTOR_GRAB_FAULT_NEXT);
    }
}

static void debug_qr_scan_found(void)
{
    debug_qr_scan_active = 0;
    debug_qr_scan_done = 1;
    /* 转发任务(TLink)不再删除: 全程保留1s轮询, 夹取/放置计数一变即发屏(最多1s延迟)。
       此处主动补发一行任务码, 不依赖轮询, 扫码即上屏。 */
    UART8_SendTaskCode();
    debug_point_pause_start(4, DEBUG_POINT_BEEP_MS);
}

static void debug_line_start(int32_t value, uint8_t relative, float heading)
{
    OdomData_t o;
    int32_t current;

    odometry_get(&o);
    current = o.enc_pos[ODOM_POS_AXIS];
    pos_target = relative ? current + value : value - debug_pos_offset;
    pos_run_start(current);
    yaw_hold = yaw_abs360_to_signed(heading);
    PID_Init(&PosPID);
    PosPID.Kp = POS_PID_KP;  PosPID.Ki = POS_PID_KI;  PosPID.Kd = POS_PID_KD;
    PosPID.ErrorIntMax = POS_ERR_INT_MAX;  PosPID.ErrorIntMin = -POS_ERR_INT_MAX;
    PosPID.OutMax = POS_OUT_MAX_RPM;  PosPID.OutMin = -POS_OUT_MAX_RPM;
    PID_Init(&YawPID);
    line_yaw_reset();
    YawPID.Kp = YAW_PID_KP;  YawPID.Ki = YAW_PID_KI;  YawPID.Kd = YAW_PID_KD;
    YawPID.ErrorIntMax = YAW_ERR_INT_MAX;  YawPID.ErrorIntMin = -YAW_ERR_INT_MAX;
    YawPID.OutMax = YAW_OUT_MAX_RPM;  YawPID.OutMin = -YAW_OUT_MAX_RPM;
    phase = PHASE_LINE;
}

static void debug_qr_scan_start(void)
{
    if (!MOTOR_ROUTE_QR_SCAN_ENABLE || debug_qr_scan_done)
    {
        debug_point_pause_start(3, DEBUG_POINT_BEEP_MS);
        return;
    }
    debug_qr_scan_seq = UART8_Seq();
    if (qr_report_handle == NULL &&
        xTaskCreate(UART8_ReportTask, "TLink", 256, NULL, 2,
                    &qr_report_handle) != pdPASS)
        qr_report_handle = NULL;         /*创建失败不阻塞扫码, 下次到站重试*/
    debug_line_start(debug_qr_scan_dir * DEBUG_QR_SCAN_COUNTS, 1, 0.0f);
    debug_qr_scan_active = 1;
    phase = PHASE_SLOW;
}

static int rdk_center_axis_speed(float error)
{
    float speed = fabsf(error) * RDK_CENTER_KP_RPM_PX;
    if (speed < (float)RDK_CENTER_MIN_RPM) speed = (float)RDK_CENTER_MIN_RPM;
    if (speed > (float)RDK_CENTER_MAX_RPM) speed = (float)RDK_CENTER_MAX_RPM;
    speed *= RDK_CENTER_SPEED_SCALE;
    /* Fine approach inside 2x tolerance (~6px): ~0.4 RPM/px, min 1.2 (rounds
       to 2 RPM), so the tight +/-3 px window is entered at crawl speed and the
       stopping distance stays well inside one window width - no ping-pong. */
    if (fabsf(error) <= 2.0f * RDK_CENTER_TOL_PX)
    {
        speed = fabsf(error) * 0.4f;
        if (speed < 1.2f) speed = 1.2f;
    }
    return (error >= 0.0f) ? (int)(speed + 0.5f) : -(int)(speed + 0.5f);
}

static float rdk_center_slew(float current, float target)
{
    float step = RDK_CENTER_SLEW_RPM_S * RDK_CENTER_SPEED_SCALE * 0.01f;
    if (target > current + step) return current + step;
    if (target < current - step) return current - step;
    return target;
}

static void debug_rdk_center_found(void)
{
    Motor_Stop();
    RDK_Center_SendIdle();
    RDK_Center_Clear();
    RDK_Center_Enable(0);
    debug_rdk_center_active = 0;
    debug_rdk_center_sent = 0;
    debug_rdk_center_settle_count = 0;
    debug_rdk_center_vx = 0.0f;
    debug_rdk_center_vy = 0.0f;
    debug_rdk_grab_done = 0;
    if (debug_rdk_standalone)
    {
        /* 14独立调试: 对位完成即恢复里程计结束，不驱动夹爪——避免夹爪在大角度
           (如455°)时被绝对120°目标突然收回。伸出动作已前移到路线到站流程。 */
        debug_rdk_standalone = 0;
        debug_rdk_pos_sequence = odometry_encoder_sequence();
        debug_rdk_resume_start_tick = xTaskGetTickCount();
        debug_rdk_resume_action = 0;
        phase = PHASE_RDK_RESUME;
    }
    else
    {
        raw_chassis_hold_tick = xTaskGetTickCount();
        phase = PHASE_RDK_CHASSIS_HOLD;
    }
}

/* 真实原料区及1E：转盘定位后建立位置保持，仅RM预定位，丝杆保持提前准备的2000。 */
static void debug_raw_chassis_hold_run(void)
{
    Motor_Stop();
    if (!Arm_M2006_Ready())
    {
        grab_fail(MOTOR_GRAB_FAULT_ARM);
        return;
    }
    if ((xTaskGetTickCount() - raw_chassis_hold_tick) <
        pdMS_TO_TICKS(CHASSIS_HOLD_SETTLE_MS)) return;
    if (!motor_hold_current())
    {
        grab_fail(MOTOR_GRAB_FAULT_CHASSIS);
        return;
    }
    raw_chassis_held = 1;
    if (!Arm_M2006_SetTargetDeg(RDK_RAW_ARM_GRAB_DEG))
    {
        grab_fail(MOTOR_GRAB_FAULT_ARM);
        return;
    }
    grab_wait_start();
    phase = PHASE_RDK_ARM_POST;
}

static void debug_rdk_arm_run(void)
{
    Motor_Stop();
    if (!Arm_M2006_Ready() ||
        (xTaskGetTickCount() - debug_rdk_arm_start_tick) >=
            pdMS_TO_TICKS(ARM_M2006_MOVE_TIMEOUT_MS))
    {
        debug_no_next_start();
        return;
    }
    if (!Arm_M2006_AtTarget()) return;
    phase = PHASE_RDK_GRAB_WAIT;
    if (debug_rdk_standalone)
    {
        debug_rdk_standalone = 0;
        debug_rdk_pos_sequence = odometry_encoder_sequence();
        debug_rdk_resume_start_tick = xTaskGetTickCount();
        debug_rdk_resume_action = 0;
        phase = PHASE_RDK_RESUME;
    }
}

static void debug_rdk_return_start(const OdomData_t *o)
{
    pos_target = debug_rdk_pos_raw;
    pos_run_start(o->enc_pos[ODOM_POS_AXIS]);
    yaw_hold = debug_rdk_center_yaw;
    PID_Init(&PosPID);
    PosPID.Kp = POS_PID_KP;  PosPID.Ki = POS_PID_KI;  PosPID.Kd = POS_PID_KD;
    PosPID.ErrorIntMax = POS_ERR_INT_MAX;  PosPID.ErrorIntMin = -POS_ERR_INT_MAX;
    PosPID.OutMax = POS_OUT_MAX_RPM;  PosPID.OutMin = -POS_OUT_MAX_RPM;
    PID_Init(&YawPID);
    line_yaw_reset();
    YawPID.Kp = YAW_PID_KP;  YawPID.Ki = YAW_PID_KI;  YawPID.Kd = YAW_PID_KD;
    YawPID.ErrorIntMax = YAW_ERR_INT_MAX;  YawPID.ErrorIntMin = -YAW_ERR_INT_MAX;
    YawPID.OutMax = YAW_OUT_MAX_RPM;  YawPID.OutMin = -YAW_OUT_MAX_RPM;
    phase = PHASE_RDK_RETURN;
}

static void debug_rdk_center_resume_run(void)
{
    OdomData_t o;
    uint32_t sequence = odometry_snapshot(&o);
    if (sequence == debug_rdk_pos_sequence || !o.online[ODOM_POS_AXIS])
    {
        if ((xTaskGetTickCount() - debug_rdk_resume_start_tick) >=
            pdMS_TO_TICKS(RDK_CENTER_RESUME_MS))
        {
            if (debug_rdk_resume_action == 3)
            {
                debug_rdk_resume_action = 2;
                debug_no_next_start();
                grab_record_fault(MOTOR_GRAB_FAULT_ODOM);
            }
            else if (debug_rdk_resume_action == 2)
            {
                debug_rdk_pos_active = 0;
                debug_rdk_resume_action = 0;
                phase = PHASE_DONE;
            }
            else
            {
                debug_rdk_pos_active = 0;
                debug_rdk_resume_action = 0;
                phase = PHASE_IDLE;
            }
        }
        return;
    }
    if (debug_rdk_resume_action == 2)
    {
        debug_pos_offset += debug_rdk_pos_raw - o.enc_pos[ODOM_POS_AXIS];
        debug_rdk_pos_active = 0;
        debug_rdk_resume_action = 0;
        debug_point_pause_start(0, 5000);
        return;
    }
    if (debug_rdk_resume_action == 3)
    {
        debug_rdk_resume_action = 0;
        debug_rdk_return_start(&o);
    }
    else
    {
        debug_rdk_pos_active = 0;
        debug_rdk_resume_action = 0;
        phase = (debug_route_active && no_next_alarm) ? PHASE_DONE : PHASE_IDLE;
    }
}

static void debug_rdk_center_start(float target_yaw, int8_t x_sign, int8_t y_sign)
{
    no_next_alarm = 0;
    debug_rdk_x_sign = x_sign;
    debug_rdk_y_sign = y_sign;
    Motor_Stop();
    RDK_Center_Clear();
    RDK_Center_Enable(1);
    debug_rdk_center_active = 1;
    debug_rdk_center_sent = 0;
    debug_rdk_center_start_tick = xTaskGetTickCount();
    debug_rdk_center_last_seq = RDK_CenterSequence();
    debug_rdk_center_settle_count = 0;
    debug_rdk_center_vx = 0.0f;
    debug_rdk_center_vy = 0.0f;
    {
        OdomData_t o;
        debug_rdk_pos_sequence = odometry_snapshot(&o); /*只保存对位前位置，不停止里程计。*/
        debug_rdk_pos_raw = o.enc_pos[ODOM_POS_AXIS];
        debug_rdk_pos_hold = debug_rdk_pos_raw + debug_pos_offset;
        debug_rdk_pos_active = 1;
    }
    debug_rdk_center_yaw = target_yaw;
    line_yaw_reset();
    phase = PHASE_RDK_CENTER;
    RDK_Center_SendTurntable();
    debug_rdk_center_sent = 1;
}

static void debug_rdk_center_run(void)
{
    TickType_t now = xTaskGetTickCount();
    uint32_t age = RDK_CenterAgeMs();
    uint32_t sequence;
    int error_x, error_y;
    int vx, vy, w, translation;
    uint8_t in_tol;

    if (RDK_VisionWaitingFirst())
    {
        debug_rdk_center_start_tick = now;
        return; /* 底盘在入口已停；首帧一直未到则持续500ms重发，不进入FAULT。 */
    }
    if ((now - debug_rdk_center_start_tick) >= pdMS_TO_TICKS(RDK_CENTER_TIMEOUT_MS))
    {
        debug_no_next_start();
        return;
    }
    if (!RDK_CenterValid() || age > RDK_CENTER_STALE_MS)
    {
        if (debug_rdk_center_vx != 0.0f || debug_rdk_center_vy != 0.0f)
            Motor_Stop();
        debug_rdk_center_vx = 0.0f;
        debug_rdk_center_vy = 0.0f;
        debug_rdk_center_settle_count = 0;
        return;
    }

    /* 相机安装轴与车体轴交叉：图像Y控制左右，图像X控制前后。
       禁用车体轴既不输出平移，也不参与到位确认。目标仍按图像X=320/Y=240。 */
    error_x = debug_rdk_x_sign ? (int)RDK_CenterY() - RDK_CENTER_TARGET_Y : 0;
    error_y = debug_rdk_y_sign ? (int)RDK_CenterX() - RDK_CENTER_TARGET_X : 0;
    in_tol = (uint8_t)(abs(error_x) <= RDK_CENTER_TOL_PX &&
                       abs(error_y) <= RDK_CENTER_TOL_PX);
    sequence = RDK_CenterSequence();
    if (sequence != debug_rdk_center_last_seq)
    {
        debug_rdk_center_last_seq = sequence;
        if (in_tol)
        {
            if (++debug_rdk_center_settle_count >= 5)
            {
                debug_rdk_center_found();
                return;
            }
        }
        else
            debug_rdk_center_settle_count = 0;
    }
    if (in_tol)
    {
        Motor_Stop();
        debug_rdk_center_vx = 0.0f;
        debug_rdk_center_vy = 0.0f;
        return;
    }

    vx = rdk_center_axis_speed((float)(debug_rdk_x_sign * error_x));
    vy = rdk_center_axis_speed((float)(debug_rdk_y_sign * error_y));
    if (abs(error_x) <= RDK_CENTER_TOL_PX) vx = 0;
    if (abs(error_y) <= RDK_CENTER_TOL_PX) vy = 0;
    debug_rdk_center_vx = rdk_center_slew(debug_rdk_center_vx, (float)vx);
    debug_rdk_center_vy = rdk_center_slew(debug_rdk_center_vy, (float)vy);
    translation = (int)fmaxf(fabsf(debug_rdk_center_vx), fabsf(debug_rdk_center_vy));
    w = line_yaw_output(debug_rdk_center_yaw, translation > 0 ? translation : 1);
    Motor_Move(line_rpm_round(debug_rdk_center_vx),
               line_rpm_round(debug_rdk_center_vy), w);
}

static void debug_turn_start(float target_yaw)
{
    if (debug_route_active)
    {
        OdomData_t o;
        odometry_get(&o);
        debug_turn_pos_raw = o.enc_pos[ODOM_POS_AXIS];
        debug_turn_pos_hold = debug_turn_pos_raw + debug_pos_offset;
        debug_turn_pos_active = 1;
    }
    float now_yaw = IMU_GetYaw();
    turn_run_start(now_yaw, turn_angdiff(yaw_abs360_to_signed(target_yaw), now_yaw), 0);
}

/*转弯结束后抵消里程计1在原地转向期间产生的计数。
  编码器原始pos只读，路线坐标通过offset完成等效清零。*/
static void debug_turn_pos_commit(void)
{
    OdomData_t o;
    if (!debug_turn_pos_active) return;
    odometry_get(&o);
    debug_pos_offset += debug_turn_pos_raw - o.enc_pos[ODOM_POS_AXIS];
    debug_turn_pos_active = 0;
}

typedef enum
{
    RP_UNKNOWN, RP_START1, RP_START2, RP_DIAG_RU, RP_DIAG_RD, RP_QR, RP_RAW,
    RP_ROUGH, RP_STORAGE, RP_RU, RP_RD, RP_LD, RP_LU, RP_CENTER
} RoutePoint;

typedef struct
{
    uint16_t x, y;
    RoutePoint point;
} RoutePointCoord;

/*上位机发送坐标以左下角为原点，不使用绘图内部的向下Y坐标。*/
static const RoutePointCoord debug_points[] = {
    {2200,2200,RP_START1}, {2200,200,RP_START2},
    {2000,2000,RP_DIAG_RU}, {2000,400,RP_DIAG_RD},
    {2000,1200,RP_QR}, {1200,1900,RP_RAW}, {1200,400,RP_ROUGH},
    {400,1200,RP_STORAGE}, {2000,1900,RP_RU}, {2000,500,RP_RD},
    {400,400,RP_LD}, {400,1900,RP_LU}, {1200,1200,RP_CENTER}
};

typedef struct
{
    uint8_t sub;                               /*08帧子命令: 0直线/1右前斜行/2绝对转向*/
    uint8_t relative;
    int32_t value;
    int16_t yaw;                               /*标定航向；-1的出发直线以0°标定*/
} DebugMotion;

typedef struct
{
    RoutePoint from, to;
    uint8_t count;
    DebugMotion motion[3];
} DebugPointRule;

/*仅添加用户提供的点对；保持标定行程，上传航向相反时只反转直线正负。*/
static const DebugPointRule debug_rules[] = {
    {RP_START1,RP_DIAG_RU,1,{{1,1,120000,-1}}},
    {RP_START2,RP_DIAG_RD,1,{{5,1,120000,0}}},
    {RP_DIAG_RU,RP_QR,1,{{0,1,500000,-1}}},
    {RP_DIAG_RD,RP_QR,1,{{0,1,-500000,0}}},
    {RP_QR,RP_RU,1,{{0,0,110000,-1}}},
    {RP_QR,RP_CENTER,3,{{0,0,725000,-1},{2,0,90,-1},{0,1,610000,90}}},  /*段3: 2026-11-15改620000→610000*/
    {RP_CENTER,RP_RAW,1,{{0,1,615000,180}}},  /*中心点→原料区: 2026-11-15改625000→615000*/
    {RP_RU,RP_RAW,1,{{0,1,600000,90}}},  /*右上角→原料区: 2026-11-15改615000→605000→600000*/
    {RP_RAW,RP_ROUGH,1,{{0,1,1215000,0}}},  /*原料区→粗加工区: 2026-11-15改1225000→1215000*/
    {RP_ROUGH,RP_LD,1,{{0,1,-625000,270}}},
    {RP_LD,RP_STORAGE,1,{{0,1,-612000,0}}},  /*左下角→暂存区: 2026-11-15改615000→612000*/
    {RP_STORAGE,RP_LU,1,{{0,1,-625000,0}}},
    {RP_LU,RP_RAW,1,{{0,1,-620000,90}}},   /*左上角→原料区: 2026-11-15改620000*/
    {RP_LU,RP_RU,1,{{0,1,1250000,270}}},
    {RP_STORAGE,RP_QR,1,{{0,1,1250000,270}}},
    {RP_STORAGE,RP_LD,1,{{0,1,625000,0}}},
    {RP_LD,RP_RD,1,{{0,1,1250000,270}}},
    {RP_RAW,RP_CENTER,1,{{0,1,625000,0}}},
    {RP_CENTER,RP_STORAGE,1,{{0,1,625000,90}}},
    {RP_LD,RP_ROUGH,1,{{0,1,625000,270}}},
    {RP_LU,RP_LD,1,{{0,1,1230000,0}}},
    {RP_ROUGH,RP_CENTER,1,{{0,1,625000,180}}},
    {RP_RAW,RP_LU,1,{{0,1,625000,90}}},
    {RP_CENTER,RP_QR,1,{{0,1,625000,270}}},
    {RP_CENTER,RP_ROUGH,1,{{0,1,625000,0}}},
    {RP_LU,RP_STORAGE,1,{{0,1,625000,0}}},
    {RP_RAW,RP_RU,1,{{0,1,-625000,90}}},
    {RP_RU,RP_RD,1,{{0,1,1250000,0}}},
    {RP_RD,RP_ROUGH,1,{{0,1,-625000,270}}}
};
#define DEBUG_RULE_COUNT (sizeof(debug_rules) / sizeof(debug_rules[0]))

static RoutePoint debug_point_at(uint16_t x, uint16_t y)
{
    uint8_t i;
    if (x == 2000 && y >= 1100 && y <= 1300) return RP_QR;
    if (y == 1900 && x >= 1100 && x <= 1300) return RP_RAW;
    for (i = 0; i < sizeof(debug_points) / sizeof(debug_points[0]); i++)
        if (debug_points[i].x == x && debug_points[i].y == y)
            return debug_points[i].point;
    return RP_UNKNOWN;
}

static int debug_station_heading(RoutePoint point)
{
    if (point == RP_QR || point == RP_STORAGE) return 0;
    if (point == RP_RAW) return 90;
    if (point == RP_ROUGH) return 270;
    return -1;
}

/*保留标定时的正负方向，仅在上传航向相反时翻转。*/
static int32_t debug_relative_value(int32_t value, uint16_t calibrated_heading)
{
    if ((calibrated_heading + 180) % 360 == debug_route_heading)
        return -value;
    return value;
}

static uint8_t debug_leg_load(void)
{
    uint8_t i, valid;
    RoutePoint from, to;
    if (!route_debug_peek(debug_route_batch, &debug_route_leg))
    {
        if (!debug_route_next_batch || !route_debug_finished(debug_route_batch) ||
            debug_route_leg.to_x != 400 || debug_route_leg.to_y != 1200 ||
            debug_route_heading != 0) return 0;
        route_debug_release(debug_route_batch);
        debug_route_batch = debug_route_next_batch;
        debug_route_next_batch = 0;
        grab_pick_index = 0;                     /*第二批开始, 夹取序号归零*/
        if (!route_debug_peek(debug_route_batch, &debug_route_leg)) return 0;
    }
    from = debug_point_at(debug_route_leg.from_x, debug_route_leg.from_y);
    to = debug_point_at(debug_route_leg.to_x, debug_route_leg.to_y);
    debug_rule_index = DEBUG_RULE_COUNT;
    debug_motion_index = 0;
    for (i = 0; i < DEBUG_RULE_COUNT; i++)
        if (debug_rules[i].from == from && debug_rules[i].to == to)
        {
            debug_rule_index = i;
            break;
        }
    debug_goal_heading = debug_route_heading;
    if (debug_route_leg.action == CMD_ACT_CW)
        debug_goal_heading = (debug_route_heading + 90) % 360;
    else if (debug_route_leg.action == CMD_ACT_CCW)
        debug_goal_heading = (debug_route_heading + 270) % 360;
    if (debug_route_leg.from_x == debug_route_leg.to_x &&
        debug_route_leg.from_y == debug_route_leg.to_y)
        valid = (uint8_t)(debug_route_leg.station &&
                         debug_route_leg.action != CMD_ACT_BACK &&
                         debug_station_heading(from) == debug_goal_heading);
    else
        valid = (uint8_t)(debug_rule_index < DEBUG_RULE_COUNT ||
                          debug_route_leg.action == CMD_ACT_CW ||
                          debug_route_leg.action == CMD_ACT_CCW);
    /* 检查失败不推进缓存游标，300ms重试仍然是同一条路线。 */
    return valid ? route_debug_next(debug_route_batch, &debug_route_leg) : 0;
}

uint8_t Motor_RouteGetStatus(MotorRouteStatus *out)
{
    static const char *const read_reason[] = {"CHECK", "UNLOCK", "END", "GAP", "INDEX"};
    RouteDebugStatus cache;
    RouteDebugLeg leg;
    MotorRouteStatus status = {0};
    uint8_t i;
    RoutePoint from, to;
    if (!out) return 0;
    taskENTER_CRITICAL();
    if (!no_next_alarm || (no_next_retry != NEXT_RETRY_LOAD && no_next_retry != NEXT_RETRY_MOTION))
    {
        taskEXIT_CRITICAL();
        return 0;
    }
    status.batch = debug_route_batch;
    status.next_batch = debug_route_next_batch;
    status.heading = debug_route_heading;
    status.goal = debug_goal_heading;
    route_debug_status(status.batch, &cache);
    status.segment = cache.segment;
    status.count = cache.count;
    status.point = cache.point;
    status.busy = cache.busy;
    status.upload_error = cache.error;
    status.previous_x = cache.previous_x;
    status.previous_y = cache.previous_y;
    status.has_leg = (uint8_t)(no_next_retry == NEXT_RETRY_MOTION || cache.has_leg);
    leg = no_next_retry == NEXT_RETRY_MOTION ? debug_route_leg : cache.leg;
    if (no_next_retry == NEXT_RETRY_LOAD && cache.has_leg)
    {
        status.goal = status.heading;
        if (leg.action == CMD_ACT_CW) status.goal = (status.heading + 90u) % 360u;
        else if (leg.action == CMD_ACT_CCW) status.goal = (status.heading + 270u) % 360u;
    }
    status.reason = cache.state < sizeof(read_reason) / sizeof(read_reason[0]) ?
        read_reason[cache.state] : "INDEX";
    if (no_next_retry == NEXT_RETRY_MOTION || cache.state == ROUTE_READ_OK)
    {
        from = debug_point_at(leg.from_x, leg.from_y);
        to = debug_point_at(leg.to_x, leg.to_y);
        for (i = 0; i < DEBUG_RULE_COUNT; i++)
            if (debug_rules[i].from == from && debug_rules[i].to == to) break;
        status.rule = (uint8_t)(i < DEBUG_RULE_COUNT);
        status.reason = leg.station ? "STATION" : (status.rule ? "HEADING" : "NO RULE");
        if (cache.state == ROUTE_READ_OK && no_next_retry == NEXT_RETRY_LOAD &&
            ((leg.from_x == leg.to_x && leg.from_y == leg.to_y) ?
                (leg.station && leg.action != CMD_ACT_BACK && debug_station_heading(from) == status.goal) :
                (status.rule || leg.action == CMD_ACT_CW || leg.action == CMD_ACT_CCW)))
            status.reason = "READY"; /* 常见为等待编码器连续3帧；不代表已恢复执行。 */
    }
    status.action = leg.action;
    status.station = leg.station;
    status.from_x = leg.from_x;
    status.from_y = leg.from_y;
    status.to_x = leg.to_x;
    status.to_y = leg.to_y;
    taskEXIT_CRITICAL();
    *out = status;
    return 1;
}

/* 只在最后一段直行、且下一缓存项明确是同坐标到站作业时提前准备。
   peek不消费路线；途经站点、转弯段、其他手动运动均不触发。 */
static void debug_approach_start(void)
{
    RouteDebugLeg next;
    RoutePoint point;
    int16_t arm;
    if (!debug_route_active || debug_route_leg.station || phase != PHASE_LINE ||
        debug_motion_index + 1u != debug_rules[debug_rule_index].count ||
        !route_debug_peek(debug_route_batch, &next) || !next.station ||
        next.from_x != debug_route_leg.to_x || next.from_y != debug_route_leg.to_y ||
        next.to_x != next.from_x || next.to_y != next.from_y) return;
    point = debug_point_at(next.from_x, next.from_y);
    if (point != RP_RAW && point != RP_ROUGH && point != RP_STORAGE) return;
    if (debug_approach.point) return; /* 每次驶入只发一次，恢复里程计不重发。 */
    debug_approach.point = (uint8_t)point;
    debug_approach.batch = debug_route_batch;
    debug_approach.x = next.from_x;
    debug_approach.y = next.from_y;
    debug_approach.belt_done = debug_approach.lift_done = 0;
    debug_approach.lift_active = (uint8_t)(point == RP_RAW);
    debug_approach.tick = xTaskGetTickCount();
    arm = point == RP_RAW ? RDK_RAW_ARM_PRE_DEG : RING_STN_ARM_DEG;
    if (!Arm_M2006_SetTargetDeg(arm))
    {
        if (point == RP_RAW) grab_fail(MOTOR_GRAB_FAULT_ARM);
        else station_place_fail();
        return;
    }
    belt_pos_goto(point == RP_RAW ? RDK_RAW_BELT_POS : RING_STN_BELT_POS);
    Servo_SetAngle(point == RP_RAW ? RDK_RAW_SERVO180_DEG : RING_STN_SERVO180_DEG);
    if (point == RP_RAW) debug_raw_lift_prepare();
    /* 180°张开不等待；原料丝杆同时到2000，其余站点丝杆及270°保持。 */
}

static uint8_t debug_approach_matches(RoutePoint point)
{
    return (uint8_t)(debug_route_active && debug_approach.point == (uint8_t)point &&
        debug_approach.batch == debug_route_batch &&
        debug_approach.x == debug_route_leg.from_x &&
        debug_approach.y == debug_route_leg.from_y);
}

static void debug_approach_poll(void)
{
    if (!debug_approach.point) return;
    if (!Arm_M2006_Ready())
    {
        if (debug_approach.point == RP_RAW) grab_fail(MOTOR_GRAB_FAULT_ARM);
        else station_place_fail();
        return;
    }
    debug_approach.belt_done |= Belt_ZDT_ReachedGet();
    if (debug_approach.lift_active) debug_approach.lift_done |= Lift_ZDT_ReachedGet();
}

static void debug_motion_start(void)
{
    const DebugMotion *motion;
    if (debug_rule_index >= DEBUG_RULE_COUNT)
    {
        debug_no_next_retry_start(NEXT_RETRY_MOTION);
        return;
    }
    motion = &debug_rules[debug_rule_index].motion[debug_motion_index];
    if (motion->sub == 1 || motion->sub == 5)
    {
        debug_route_step = 1;
        debug_qr_scan_dir = (motion->sub == 5) ? -1 : 1;
        diag_run_start((motion->sub == 1) ? 1 : 3, motion->value,
                       motion->yaw < 0 ? (float)debug_route_heading : (float)motion->yaw);
    }
    else if (motion->sub == 2)
    {
        debug_route_step = 4;
        if (debug_route_leg.action != CMD_ACT_CW && debug_route_leg.action != CMD_ACT_CCW)
            debug_goal_heading = (uint16_t)motion->value; /*未附带转向动作时沿用标定绝对90°*/
        debug_turn_start((float)debug_goal_heading);
    }
    else
    {
        int32_t value = motion->value;
        uint8_t relative = motion->relative;
        uint16_t calibrated_heading = motion->yaw < 0 ? 0 : (uint16_t)motion->yaw;
        debug_route_step = (debug_rules[debug_rule_index].from == RP_DIAG_RU) ? 2 :
                           (debug_motion_index == 2 ? 5 : 3);
        /*扫码区位置随机，首次去原料区前保持绝对校准。后续用标定扫码pos
          620000换算原行程，避免连续路线累计pos改变后追赶旧的绝对目标。*/
        if (!relative && debug_first_raw_reached &&
            debug_rules[debug_rule_index].from == RP_QR)
        {
            value -= 620000;
            relative = 1;
        }
        if (debug_route_heading != calibrated_heading &&
            debug_route_heading != (calibrated_heading + 180) % 360)
        {
            debug_no_next_retry_start(NEXT_RETRY_MOTION);
            return;
        }
        if (relative) value = debug_relative_value(value, calibrated_heading);
        debug_line_start(value, relative, (float)debug_route_heading);
        debug_qr_scan_dir = (pos_initial_error < 0) ? -1 : 1;
        debug_approach_start();
    }
}

static uint32_t ring_now_ms(void)
{
    return (uint32_t)((uint64_t)xTaskGetTickCount() * 1000u / configTICK_RATE_HZ);
}

static void debug_rdk_ring_start(void)
{
    station_debug_selection = 0;
    /* 两轮粗加工及第一轮暂存均只启动视觉，不暂停编码器轮询或计数。 */
    debug_rdk_ring_serial = 0;
    debug_rdk_stack_mode = 0;
    Motor_Stop();
    no_next_alarm = 0;
    RDK_Ring_Start();
    RingAlign_Start(ring_now_ms());
    phase = PHASE_RDK_RING_WAIT;
}

/* 模式5码垛对位：路线完成后接第二轮暂存放料；独立1D只对位停车。 */
static void debug_rdk_stack_start(uint8_t standalone)
{
    station_debug_selection = 0;
    debug_rdk_ring_serial = standalone;
    debug_rdk_stack_mode = 1;
    Motor_Stop();
    no_next_alarm = 0;
    RDK_Stack_Start();
    /* 恢复默认yaw校正方向: 0x15 sub=2的方向覆盖(含0=禁用)是圆环调试遗留,
       若残留0, Update第一拍就把对位打回WAIT_DIR(车永不动, LCD显SET DIR)。 */
    RingAlign_ResetYawSign();
    RingAlign_StartMode(5, 0, ring_now_ms());
    phase = PHASE_RDK_RING_WAIT;
}

/* 可在到站入口立即检查，避免提前准备已完成还多等一轮任务。 */
static void debug_raw_prepare_run(void)
{
    uint8_t belt_done, lift_done;
    if (!Arm_M2006_Ready())
    {
        grab_fail(MOTOR_GRAB_FAULT_ARM);
        return;
    }
    belt_done = grab_axis_done(0); /* 提前张爪不能绕过机构等待。 */
    lift_done = grab_axis_done(1); /* 提前准备的2000：到位或自下发满原1秒上限。 */
    if (!belt_servo_done && belt_done)
    {
        belt_servo_done = 1;
        Servo_SetAngle(RDK_RAW_SERVO180_DEG); /* 不驻留 */
    }
    if (belt_done && lift_done && belt_servo_done && grab_arm_done())
        debug_rdk_center_start(yaw_abs360_to_signed(RDK_CENTER_RAW_YAW),
                               RDK_CENTER_X_SIGN, RDK_CENTER_Y_SIGN);
}

/* 路线站点和1E独立网页共用；调用前仍须确认90°作业航向。 */
static void debug_raw_prepare(void)
{
    irq_station_enter_raw();               /*原料区: Encoder让路15, IMU保持4*/
    run_fault = MOTOR_RUN_FAULT_NONE;
    Motor_Stop();
    grab_fault.reason = MOTOR_GRAB_FAULT_NONE;
    debug_first_raw_reached = 1;
    debug_rdk_standalone = 0;
    Servo270_SetAngle(grab_pick_servo270_drop
        [grab_pick_index < 3u ? grab_pick_index : 2u]);
    grab_wait_start();
    belt_servo_done = 0; /* 无提前准备时仍等同步带后张开，不驻留。 */
    if (debug_approach_matches(RP_RAW))
    {
        grab_step_tick = debug_approach.tick;
        grab_parallel_belt_done = debug_approach.belt_done;
        grab_parallel_lift_done = debug_approach.lift_done;
        belt_servo_done = 1; /* 行驶途中已张开，到站不重发。 */
        debug_approach.point = 0; /* 接管原目标，不重发、不清到位标志。 */
    }
    else
    {
        belt_pos_goto(RDK_RAW_BELT_POS);
        if (!Arm_M2006_SetTargetDeg(RDK_RAW_ARM_PRE_DEG))
        {
            grab_fail(MOTOR_GRAB_FAULT_ARM);
            return;
        }
        debug_raw_lift_prepare(); /* 未提前准备的独立1E/站点同样先到2000。 */
    }
    debug_rdk_arm_start_tick = grab_step_tick;
    phase = PHASE_RDK_ARM_PRE;
    debug_raw_prepare_run(); /* 路上已到位或已满原等待上限则立即发模式4。 */
}

/* Both already-aligned and turn-completed station paths enter here once. */
static void debug_station_work_start(void)
{
    RoutePoint point = debug_point_at(debug_route_leg.from_x, debug_route_leg.from_y);
    if (point == RP_QR)
        debug_qr_scan_start();
    else if (point == RP_RAW)
    {
        raw_debug_full = 0;
        debug_raw_prepare();
    }
    else if (point == RP_ROUGH || point == RP_STORAGE)
    {
        irq_station_enter();               /* Encoder让路15；HWT101保持4，轮趣沿用降级 */
        /* 圆环站点到站预定位: RM电机190°/同步带7100/180°舵机25°, 发完即进校准,
           机构动作与视觉校准并行(校准本身不动机构)。 */
        if (debug_approach_matches(point))
        {
            debug_ring_arm_start_tick = debug_approach.tick;
            debug_approach.point = 0;
        }
        else
        {
            debug_ring_arm_start_tick = xTaskGetTickCount();
            if (!Arm_M2006_SetTargetDeg(RING_STN_ARM_DEG))
            {
                station_place_fail();
                return;
            }
            belt_pos_goto(RING_STN_BELT_POS);
            Servo_SetAngle(RING_STN_SERVO180_DEG);
        }
        if (point == RP_STORAGE && debug_route_batch == 2u)
            debug_rdk_stack_start(0); /* 二轮暂存发模式5，完成后自动接32000高度放料。 */
        else
            debug_rdk_ring_start();
        /* 里程计持续更新；视觉平均XY→Yaw→平均XY，圆环站点不返回保存位置。 */
    }
    else
    {
        debug_point_pause_start(3, DEBUG_POINT_BEEP_MS);
    }
}

/* 失败立即停止三轴，不松爪、不推进路线；沿用no next报警和清理。 */
static void station_place_fail(void)
{
    station_place.step = PLACE_FINISHED;
    Arm_M2006_Stop();
    Emm_V5_Stop_Now(BELT_ZDT_ADDR, false);
    Emm_V5_Stop_Now(LIFT_ZDT_ADDR, false);
    debug_no_next_start();
}

static uint8_t station_place_begin(uint8_t batch, uint8_t storage, uint8_t manual)
{
    uint8_t i, colors = 0, rings = 0, valid;

    run_fault = MOTOR_RUN_FAULT_NONE;
    station_place.pick = 0;
    station_place.step = PLACE_CHASSIS_HOLD;
    station_place.started = 0;
    station_place.chassis_held = 0;
    station_place.manual = manual;
    station_place.retrieve = (uint8_t)!storage;
    station_place.batch = batch;
    station_place.storage = storage;
    station_place.drop_lift_target = (storage && batch == 2u) ? 32000u : 56750u;
    phase = PHASE_STATION_PLACE;
    Motor_Stop();
    /* 只在短临界区内读取任务码，防摄像头中断发布新码造成半旧半新。 */
    taskENTER_CRITICAL();
    valid = (uint8_t)(UART8_Valid() && !g_link.bad);
    for (i = 0; i < 3; ++i)
    {
        station_place.color[i] = UART8_Color(batch, i);
        station_place.ring[i] = storage ?
            (batch == 2u ? (uint8_t)UART8_StackRing(i) : UART8_StoreRing(i)) :
            UART8_Ring(batch, i);
        /* 物料按扫码顺序装入A/B/C槽；槽位与目标圆环独立保存，
           取回时不能按圆环号推算槽位。 */
        station_place.slot[i] = i;
    }
    taskEXIT_CRITICAL();
    for (i = 0; i < 3; ++i)
    {
        uint8_t color = station_place.color[i], ring = station_place.ring[i];
        if (color < 1 || color > 6 || ring < 1 || ring > 3 ||
            (colors & (1u << color)) || (rings & (1u << ring)))
            valid = 0;
        else
        {
            colors |= (uint8_t)(1u << color);
            rings |= (uint8_t)(1u << ring);
        }
    }
    if (!valid) station_place_fail();
    return valid;
}

static void station_place_start(void)
{
    RoutePoint point = debug_point_at(debug_route_leg.from_x, debug_route_leg.from_y);
    if (station_debug_selection)
    {
        /* 复用22入口已校验/保存的同一批计划；独立完成只能停车。 */
        float heading = station_place.storage ? 0.0f : 270.0f;
        debug_rdk_ring_stop();
        irq_station_leave();
        IMU_SetYaw(heading);
        yaw_hold = yaw_abs360_to_signed(heading);
        line_yaw_reset();
        phase = PHASE_STATION_PLACE;
        return;
    }
    if (!debug_route_active ||
        !((point == RP_ROUGH && (debug_route_batch == 1 || debug_route_batch == 2)) ||
          (point == RP_STORAGE && (debug_route_batch == 1 || debug_route_batch == 2))))
    {
        station_place_fail();
        return;
    }
    if (!station_place_begin(debug_route_batch, (uint8_t)(point == RP_STORAGE), 0)) return;

    /* 仅真实路线在视觉XY/Yaw全部完成后校正所选陀螺仪软件基准，不产生转向动作。
       粗加工实际作业朝向记270°，两轮暂存记0°；只执行一次，独立1C不改yaw。 */
    {
        uint16_t heading = (uint16_t)debug_station_heading(point);
        IMU_SetYaw((float)heading);
        debug_route_heading = debug_goal_heading = heading;
        yaw_hold = yaw_abs360_to_signed((float)heading);
        line_yaw_reset(); /* 避免旧yaw差分/积分引起离站时的瞬间纠偏。 */
    }
}

/* 独立站点全流程：机构准备与视觉并行，完成后接原1C手动取放状态机。 */
static uint8_t station_debug_start(uint8_t selection)
{
    uint8_t batch = (selection == 2u || selection == 4u) ? 2u : 1u;
    uint8_t storage = (uint8_t)(selection >= 3u);
    if (selection < 1u || selection > 4u || !PHASE_CAN_START(phase) ||
        debug_route_active || debug_pause_active || RDK_Ring_Active() || RDK_Stack_Active() ||
        !Arm_M2006_Ready() || IMU_ConfigStatus() != IMU_CONFIG_READY || !IMU_UpdateSequence())
        return 0;
    /* 开始前校验并保存三块颜色/环号/原槽，避免校准完再发现任务无效。 */
    if (!station_place_begin(batch, storage, 1)) return 0;
    array_mode = 0;
    leg1_next_valid = 0;
    Motor_Enable(true);
    IMU_SetYaw(storage ? 0.0f : 270.0f); /* 当前摆放朝向作为本站名义航向，不额外转弯。 */
    irq_station_enter();
    debug_ring_arm_start_tick = xTaskGetTickCount();
    if (!Arm_M2006_SetTargetDeg(RING_STN_ARM_DEG))
    {
        station_place_fail();
        return 0;
    }
    belt_pos_goto(RING_STN_BELT_POS);
    Servo_SetAngle(RING_STN_SERVO180_DEG);
    RingAlign_ResetYawSign();
    if (storage && batch == 2u) debug_rdk_stack_start(0);
    else debug_rdk_ring_start();
    station_debug_selection = selection;
    return 1;
}

/* 一组目标在同一拍下发，中间不等待；axes允许单独抬升/收同步带。 */
static void station_place_move(uint8_t axes, int32_t belt, int16_t arm, uint32_t lift)
{
    station_place.axes = axes;
    station_place.belt_wait_ms = PLACE_BELT_WAIT_LIMIT_MS; /* 每组先恢复默认，再由阶段覆盖 */
    station_place.lift_wait_ms = (lift <= PLACE_LIFT_LOW_POS_PUL) ?
        PLACE_LIFT_LOW_WAIT_MS : PLACE_LIFT_WAIT_LIMIT_MS;

    station_place.belt_done = station_place.lift_done = station_place.arm_done = 0;
    station_place.tick = xTaskGetTickCount();
    if ((axes & PLACE_AXIS_ARM) && !Arm_M2006_SetTargetDeg(arm))
    {
        station_place_fail();
        return;
    }
    if (axes & PLACE_AXIS_BELT) belt_pos_goto(belt);
    if (axes & PLACE_AXIS_LIFT)
    {
        Lift_ZDT_ReachedClear();
        Emm_V5_Pos_Control(LIFT_ZDT_ADDR, LIFT_POS_DOWN_DIR,
                           LIFT_POS_SPEED_RPM, LIFT_POS_ACC, lift, true, false);
    }
}

static uint8_t station_place_move_done(void)
{
    TickType_t elapsed = xTaskGetTickCount() - station_place.tick;
    uint8_t axes = station_place.axes;

    /* 各轴独立结束等待；只用主动反馈和RTOS计时，不再回读位置。
       超时只放行状态机，不停止轴；并行组仍等其他轴确认或各自超时。 */
    if (axes & PLACE_AXIS_BELT)
        station_place.belt_done |= (uint8_t)(Belt_ZDT_ReachedGet() ||
            elapsed >= pdMS_TO_TICKS(station_place.belt_wait_ms));
    if (axes & PLACE_AXIS_LIFT)
        station_place.lift_done |= (uint8_t)(Lift_ZDT_ReachedGet() ||
            elapsed >= pdMS_TO_TICKS(station_place.lift_wait_ms));
    if (axes & PLACE_AXIS_ARM)
    {
        /* RM等待按步骤分流：放料对环2秒 / 取回对环0.8秒 / 其余(取料/取回/收回)1秒 */
        uint16_t arm_wait;
        if (station_place.step == PLACE_DROP_BELT_ARM)
            arm_wait = PLACE_ARM_WAIT_LIMIT_MS;
        else if (station_place.step == PLACE_RETRIEVE_TARGET)
            arm_wait = PLACE_ARM_WAIT_TARGET_MS;
        else if (station_place.step == PLACE_RETRIEVE_BELT_ARM)
            arm_wait = PLACE_ARM_WAIT_RETBELT_MS;
        else
            arm_wait = PLACE_ARM_WAIT_SHORT_MS;
        station_place.arm_done |= (uint8_t)(Arm_M2006_AtTargetWithin(GRAB_ARM_POS_TOL_DEG) ||
            elapsed >= pdMS_TO_TICKS(arm_wait));
    }
    return (uint8_t)((!(axes & PLACE_AXIS_BELT) || station_place.belt_done) &&
                     (!(axes & PLACE_AXIS_LIFT) || station_place.lift_done) &&
                     (!(axes & PLACE_AXIS_ARM) || station_place.arm_done));
}

static void station_place_run(void)
{
    uint8_t done;
    /* FE会打断位置保持；固定目标只在入口设置一次，直到离站或中止。 */
    if (!station_place.chassis_held) Motor_Stop();
    if (station_place.step == PLACE_FINISHED) return;
    if (!Arm_M2006_Ready())
    {
        station_place_fail();
        return;
    }
    if (!station_place.started)
    {
        station_place.started = 1;
        station_place.tick = xTaskGetTickCount();
        switch (station_place.step)
        {
        case PLACE_CHASSIS_HOLD: break;
        case PLACE_PICK:
            /* 仅首块执行取料定位；后续已在返回阶段全部到位，直接进入夹紧。 */
            Servo270_SetAngle(grab_pick_servo270_drop[station_place.slot[station_place.pick]]);
            station_place_move(PLACE_AXES_ALL, BELT_POS_GRAB_PUL, 18, 21500u);
            break;
        case PLACE_CLOSE: Servo_SetAngle(g_place_claw_close_deg); break;
        case PLACE_PICK_LIFT:
            /* 二轮暂存抬到1000(接近顶, 2026-11-14)；其余站15000；等待统一0.3s */
            station_place_move(PLACE_AXIS_LIFT, 0, 0,
                               (station_place.batch == 2u && station_place.storage)
                                   ? 1000u : 15000u);
            station_place.lift_wait_ms = 300u;
            break;
        case PLACE_DROP_BELT_ARM:
        {
            uint8_t ring = (uint8_t)(station_place.ring[station_place.pick] - 1u);
            /* RM同拍下发, 但本步只等同步带(统一0.5s放行)——RM对环判定
               延到DROP_LIFT(总窗1.5s从本步起算, 2026-11-15)。 */
            Arm_M2006_SetTargetDeg(place_ring_target[ring].arm);
            station_place.arm_wait_start = xTaskGetTickCount();
            station_place_move(PLACE_AXIS_BELT, place_ring_target[ring].belt, 0, 0);
            station_place.belt_wait_ms = 500u;   /*带统一0.5s放行*/
            break;
        }
        case PLACE_DROP_LIFT:
        {
            station_place_move(PLACE_AXIS_LIFT, 0, 0, station_place.drop_lift_target);
            /* 随丝杆单独下放同拍发送下一物料槽位，不等待舵机。 */
            if (station_place.pick < 2u)
                Servo270_SetAngle(grab_pick_servo270_drop[station_place.slot[station_place.pick + 1u]]);
            break;
        }
        case PLACE_SETTLE: break; /* 保持夹紧，不阻塞任务；06及故障仍逐轮检查。 */
        case PLACE_OPEN: Servo_SetAngle(g_place_claw_open_deg); break; /* 直接到完全松开角度，无缓张开。 */
        case PLACE_LIFT_HOME:
            if (station_place.pick < 2u)
            {
                /* 前两块: 丝杆回取料高度21500, 带/RM在BELT_HOME收 */
                station_place_move(PLACE_AXIS_LIFT, 0, 0, 21500u);
            }
            else
            {
                /* 暂存第三块先单独抬丝杆，固定1秒后才收带/RM及转槽。 */
                station_place_move(PLACE_AXIS_LIFT, 0, 0, 12500u);
            }
            break;
        case PLACE_STORAGE_FINAL_RETURN:
            irq_station_leave();
            Servo270_SetAngle(SERVO270_SLOT_A_DEG); /* 与带/RM同阶段下发，不等反馈。 */
            station_place_move(PLACE_AXIS_BELT | PLACE_AXIS_ARM, BELT_POS_GRAB_PUL, 18, 0);
            break;
        case PLACE_BELT_HOME:
            /* 前两块: 同步带1540与RM18°并行返回, 到位后直接夹紧下一块。
               暂存第三块另走丝杆固定1秒后收带/RM的独立阶段。 */
            station_place_move(PLACE_AXIS_BELT | PLACE_AXIS_ARM,
                               BELT_POS_GRAB_PUL, 18, 0);
            break;
        case PLACE_RETRIEVE_LIFT100:
            /* 第三块放开后先抬到取回准备高度，同步带保持在上一目标环位置；
               等待0.5s(覆盖按目标的1s档, 2026-11-15) */
            station_place_move(PLACE_AXIS_LIFT, 0, 0, 25000u);
            station_place.lift_wait_ms = 500u;
            break;
        case PLACE_RETRIEVE_TARGET:
        {
            uint8_t ring = (uint8_t)(station_place.ring[station_place.pick] - 1u);
            /* 只同步带和RM同时对准圆环；270°舵机改到丝杆下放同拍再转(2026-11-15)。
               取回对环：同步带最多700ms、RM最多800ms。 */
            station_place_move(PLACE_AXIS_BELT | PLACE_AXIS_ARM, place_ring_target[ring].belt,
                               place_ring_target[ring].arm, 0);
            station_place.belt_wait_ms = 700u;
            break;
        }
        case PLACE_RETRIEVE_DROP_LIFT:
        {
            /* 丝杆下放到放料高度的同拍，270°舵机转到该物料原槽位
               (接收位置)——与放料序列的下放预转模式对称，不等待舵机。
               等待覆盖1.8s(取回下放，不走默认1秒档) */
            station_place_move(PLACE_AXIS_LIFT, 0, 0, station_place.drop_lift_target);
            station_place.lift_wait_ms = 1800u;  /* 取回下放56750：主动到位提前继续，否则1.8秒放行 */
            Servo270_SetAngle(grab_pick_servo270_drop[station_place.slot[station_place.pick]]);
            break;
        }
        case PLACE_RETRIEVE_CLOSE:
            Servo_SetAngle(g_place_claw_close_deg);
            break;
        case PLACE_RETRIEVE_LIFT50:
            /* 提起到14500；主动到位提前继续，否则最多600ms */
            station_place_move(PLACE_AXIS_LIFT, 0, 0, 14500u);
            station_place.lift_wait_ms = 600u;
            break;
        case PLACE_RETRIEVE_BELT_ARM:
            /* 取回收回：同步带最多500ms、RM最多800ms */
            station_place_move(PLACE_AXIS_BELT | PLACE_AXIS_ARM,
                               BELT_POS_GRAB_PUL, 18, 0);
            station_place.belt_wait_ms = 500u;
            break;
        case PLACE_RETRIEVE_SLOT_LIFT:
            station_place_move(PLACE_AXIS_LIFT, 0, 0, 20000u);
            station_place.lift_wait_ms = 600u;
            break;
        case PLACE_RETRIEVE_AFTER_OPEN_LIFT:
            station_place_move(PLACE_AXIS_LIFT, 0, 0, 14000u);
            station_place.lift_wait_ms = 500u;
            break;
        case PLACE_RETRIEVE_OPEN:
            Servo_SetAngle(g_place_claw_open_deg);
            break;
        case PLACE_ROUTE_RESUME:
            /* 先恢复行车档(Encoder5/IMU4)再记录序号等新数据——让路状态下刷新受抢占 */
            irq_station_leave();
            station_place.encoder_sequence = odometry_encoder_sequence();
            debug_rdk_ring_stop();
            break;
        }
        return;
    }
    if (station_place.step == PLACE_CHASSIS_HOLD)
    {
        if ((xTaskGetTickCount() - station_place.tick) <
            pdMS_TO_TICKS(CHASSIS_HOLD_SETTLE_MS)) return;
        if (!motor_hold_current())
        {
            station_place_fail();
            return;
        }
        station_place.chassis_held = 1;
        done = 1;
    }
    else if (station_place.step == PLACE_ROUTE_RESUME)
    {
        OdomData_t o;
        done = (uint8_t)(odometry_encoder_sequence() != station_place.encoder_sequence);
        if (done)
        {
            odometry_get(&o);
            done = o.online[ODOM_POS_AXIS];
        }
        if (!done && (xTaskGetTickCount() - station_place.tick) >=
                     pdMS_TO_TICKS(RDK_CENTER_RESUME_MS))
        {
            /* 第三块已收起：保留步骤、原编码序号及路线，每300ms等恢复。 */
            debug_no_next_retry_start(NEXT_RETRY_DEPART);
            return;
        }
    }
    else if (station_place.step == PLACE_LIFT_HOME && station_place.pick == 2u && station_place.storage)
        done = (uint8_t)((xTaskGetTickCount() - station_place.tick) >= pdMS_TO_TICKS(1000u));
    else if (station_place.step == PLACE_SETTLE)
        done = (uint8_t)((xTaskGetTickCount() - station_place.tick) >=
                         pdMS_TO_TICKS(PLACE_SETTLE_MS));
    else if (station_place.step == PLACE_CLOSE)
        done = (uint8_t)((xTaskGetTickCount() - station_place.tick) >=
                         pdMS_TO_TICKS(GRAB_SERVO_DWELL_MS));   /*放料夹紧驻留400ms*/
    else if (station_place.step == PLACE_RETRIEVE_CLOSE)
        done = (uint8_t)((xTaskGetTickCount() - station_place.tick) >=
                         pdMS_TO_TICKS(PLACE_RETRIEVE_CLOSE_DWELL_MS)); /*取回夹紧驻留300ms*/
    else if (station_place.step == PLACE_OPEN || station_place.step == PLACE_RETRIEVE_OPEN)
        done = (uint8_t)((xTaskGetTickCount() - station_place.tick) >=
                         pdMS_TO_TICKS(PLACE_OPEN_DWELL_MS));   /*松爪驻留100ms(2026-11-15)*/
    else
        done = station_place_move_done();
    /* 放料RM跨步骤判定(2026-11-15): DROP_LIFT完成需RM到位, 或从DROP_BELT_ARM
       起算的总窗1.5s放行(第四步带窗口0.5s+第五步RM最多1s)——RM超时只放行不阻塞。 */
    if (station_place.step == PLACE_DROP_LIFT && done &&
        !Arm_M2006_AtTargetWithin(GRAB_ARM_POS_TOL_DEG) &&
        (int32_t)(xTaskGetTickCount() - station_place.arm_wait_start) <
            pdMS_TO_TICKS(1500u))
        done = 0;   /*RM未到位且总窗未满: 等RM(丝杆已好, 不阻塞)*/
    if (!done) return;
    station_place.started = 0;
    if (station_place.step == PLACE_OPEN && station_place.pick == 2u &&
        station_place.retrieve)
    {
        /* 第三块放开后直接进入取回准备，不先回50000/1540。 */
        UART8_PlaceOk();   /*粗加工第三块已放到站点环上: 放置正确+1*/
        station_place.pick = 0;
        station_place.step = PLACE_RETRIEVE_LIFT100;
    }
    else if (station_place.step == PLACE_LIFT_HOME)
    {
        station_place.step = station_place.pick < 2u ? PLACE_BELT_HOME : PLACE_STORAGE_FINAL_RETURN;
    }
    else if (station_place.step == PLACE_STORAGE_FINAL_RETURN)
    {
        station_place.step = PLACE_FINISHED;
        if (station_place.manual) phase = PHASE_DONE;
        else
        {
            debug_rdk_ring_stop();
            if (debug_leg_load()) debug_leg_start();
            else debug_no_next_start();
        }
    }
    else if (station_place.step == PLACE_BELT_HOME)
    {
        /* 前两块回位完成(放置正确+1); 暂存第三块收尾在LIFT_HOME完成分支起步 */
        UART8_PlaceOk();
        if (++station_place.pick == 3u)
        {
            if (station_place.manual)
            {
                station_place.step = PLACE_FINISHED;
                phase = PHASE_DONE;
            }
            else
                station_place.step = PLACE_ROUTE_RESUME;
        }
        else
            /* 丝杆21500、同步带1540、RM18°已分别确认，直接夹紧下一块。 */
            station_place.step = PLACE_CLOSE;
    }
    else if (station_place.step == PLACE_RETRIEVE_AFTER_OPEN_LIFT)
    {
        /* 松爪后先抬至14000并结束等待，再计数、转下一块或续路。 */
        UART8_GrabOk();   /*一块粗加工取回完成(物料已落回车上槽位): 抓取正确+1*/
        if (++station_place.pick == 3u)
        {
            station_place.step = station_place.manual ? PLACE_FINISHED : PLACE_ROUTE_RESUME;
            if (station_place.manual) phase = PHASE_DONE;
        }
        else
            station_place.step = PLACE_RETRIEVE_TARGET;
    }
    else if (station_place.step == PLACE_ROUTE_RESUME)
    {
        station_place.step = PLACE_FINISHED;
        if (debug_leg_load())
        {
            if (debug_route_leg.station) debug_leg_start();
            else debug_point_pause_start(1, DEBUG_POINT_BEEP_MS);
        }
        else
            debug_no_next_retry_start(NEXT_RETRY_LOAD);
    }
    else
        station_place.step++;
}

static void debug_leg_start(void)
{
    /*中心点复合规则自带转向，先校准pos，随后消费本段04转向；不能重复转两次。*/
    uint8_t has_internal_turn = (uint8_t)(debug_rule_index < DEBUG_RULE_COUNT &&
                                debug_rules[debug_rule_index].count > 1 &&
                                debug_rules[debug_rule_index].motion[1].sub == 2);
    if (debug_route_leg.station)
    {
        debug_route_step = 7;
        if (debug_goal_heading != debug_route_heading ||
            fabsf(turn_angdiff(yaw_abs360_to_signed((float)debug_goal_heading),
                              IMU_GetYaw())) > TURN_TOL_DEG)
        {
            debug_turn_start((float)debug_goal_heading);
            RoutePoint point = debug_point_at(debug_route_leg.from_x, debug_route_leg.from_y);
            if (point == RP_ROUGH || point == RP_STORAGE)
                turn_output_ratio = TURN_RING_STATION_RATIO;
        }
        else
            debug_station_work_start();
        return;
    }
    if (!has_internal_turn &&
        (debug_route_leg.action == CMD_ACT_CW || debug_route_leg.action == CMD_ACT_CCW))
    {
        debug_route_step = 6;
        debug_turn_start((float)debug_goal_heading);
    }
    else
        debug_motion_start();
}

static void debug_motion_done(void)
{
    Motor_Stop();
    if (phase == PHASE_TURN)
    {
        debug_route_heading = debug_goal_heading;
        if (debug_route_step == 7)
        {
            debug_station_work_start();
            return;
        }
        if (debug_route_step == 6)
        {
            if (debug_rule_index < DEBUG_RULE_COUNT)
                debug_point_pause_start(2, DEBUG_POINT_BEEP_MS);
            else
                debug_no_next_retry_start(NEXT_RETRY_MOTION);
            return;
        }
    }
    debug_motion_index++;
    if (debug_motion_index < debug_rules[debug_rule_index].count)
        debug_motion_start();                        /*复合点对内部动作，不提前记为到点*/
    else
    {
        if (debug_leg_load())
        {
            if (debug_route_leg.station) debug_leg_start();
            else debug_point_pause_start(1, DEBUG_POINT_BEEP_MS);
        }
        else
            debug_no_next_retry_start(NEXT_RETRY_LOAD);
    }
}

static void debug_route_start(uint8_t selection)
{
    grab_region_clear();
    OdomData_t o;
    uint8_t have_leg;
    no_next_retry = NEXT_RETRY_NONE;
    if (selection == 3u)
    {
        Motor_Stop();
        IMU_SetZero(); /* 13/03已通过启动检查：所选陀螺仪当前朝向归零，再启动第一批。 */
    }
    debug_rdk_ring_stop();
    debug_route_active = 1;
    debug_route_batch = (selection == 2) ? 2 : 1;
    debug_route_next_batch = (selection == 3) ? 2 : 0;
    debug_first_raw_reached = (uint8_t)(selection == 2);
    if (selection != 2) debug_qr_scan_done = 0;
    grab_pick_index = 0;                             /*每批路线开始, 夹取序号归零*/
    debug_qr_scan_active = 0;
    RDK_Center_SendIdle();
    debug_rdk_center_active = 0;
    debug_rdk_center_sent = 0;
    debug_rdk_center_settle_count = 0;
    debug_rdk_center_vx = 0.0f;
    debug_rdk_center_vy = 0.0f;
    RDK_Center_Clear();
    RDK_Center_Enable(0);
    debug_qr_scan_dir = 1;
    debug_route_heading = 0;                         /*两启停区车头均朝下，出发yaw=0*/
    if (selection == 2 || debug_route_batch == 1) debug_pos_offset = 0;
    debug_turn_pos_active = 0;
    no_next_alarm = 0;
    Motor_Enable(true);
    have_leg = debug_leg_load();
    if (debug_route_batch == 1 &&
        debug_point_at(debug_route_leg.from_x, debug_route_leg.from_y) == RP_START2)
    {
        odometry_get(&o);
        debug_pos_offset = 1480000 - o.enc_pos[ODOM_POS_AXIS];
    }
    if (have_leg)
    {
        debug_leg_start();
    }
    else
        debug_no_next_retry_start(NEXT_RETRY_LOAD);
}

/* no next保留路线锁及原操作，300ms检查一次；中止命令在本函数之前处理。 */
static uint8_t debug_no_next_retry_run(void)
{
    TickType_t now = xTaskGetTickCount();
    uint8_t operation = no_next_retry;
    OdomData_t o;
    if (!no_next_alarm || !debug_route_active || operation == NEXT_RETRY_NONE)
    {
        no_next_retry = NEXT_RETRY_NONE;
        return 0;
    }
    /* 原5秒报警结束后静音等待，不能走原pause分支释放路线。 */
    if (debug_pause_active && (int32_t)(now - debug_pause_until) >= 0)
    {
        Buzzer_Off();
        debug_pause_active = 0;
    }
    if (now - no_next_retry_tick < pdMS_TO_TICKS(NO_NEXT_RETRY_MS)) return 1;
    no_next_retry_tick = now;
    /* 机械/RDK超时等中途故障不能靠重放整步恢复，保持原停止状态。 */
    if (operation == NEXT_RETRY_FAULT) return 1;
    odometry_get(&o);
    if (!o.online[ODOM_POS_AXIS] || o.valid_streak < ODOM_RESUME_VALID_FRAMES ||
        IMU_ConfigStatus() != IMU_CONFIG_READY) return 1;
    if (operation == NEXT_RETRY_LOAD)
    {
        if (!debug_leg_load()) return 1;
    }
    else if (operation == NEXT_RETRY_DEPART)
    {
        if (!Arm_M2006_Ready() ||
            odometry_encoder_sequence() == station_place.encoder_sequence) return 1;
    }
    else if (operation == NEXT_RETRY_MOTION)
    {
        const DebugMotion *motion;
        uint16_t heading;
        if (debug_rule_index >= DEBUG_RULE_COUNT) return 1;
        motion = &debug_rules[debug_rule_index].motion[debug_motion_index];
        heading = motion->yaw < 0 ? 0 : (uint16_t)motion->yaw;
        if (motion->sub == 0 && debug_route_heading != heading &&
            debug_route_heading != (heading + 180) % 360) return 1;
    }
    no_next_retry = NEXT_RETRY_NONE;
    no_next_alarm = 0;
    grab_fault.reason = MOTOR_GRAB_FAULT_NONE;
    debug_pause_active = debug_pause_next = 0;
    Buzzer_Off();
    if (operation == NEXT_RETRY_LOAD)
    {
        if (debug_route_leg.station) debug_leg_start();
        else debug_point_pause_start(1, DEBUG_POINT_BEEP_MS);
    }
    else if (operation == NEXT_RETRY_MOTION)
        debug_motion_start();
    else
    {
        /* 只恢复离站检查，不回到取放入口，不重复第三块，也不重置参考序号。 */
        station_place.tick = now;
        phase = PHASE_STATION_PLACE;
    }
    return 1;
}

int32_t Motor_RoutePos(int32_t encoder_pos)
{
    if (debug_turn_pos_active)
        return debug_turn_pos_hold;
    if (debug_rdk_pos_active)
        return debug_rdk_pos_hold;
    return encoder_pos + debug_pos_offset;
}

/* 所有原地转向复用同一初始化，防止累计角/限斜率沿用上一次状态。 */
static void turn_run_start(float start_yaw, float delta, uint8_t accumulate)
{
    run_fault = MOTOR_RUN_FAULT_NONE;
    no_next_alarm = 0;
    yaw_start = turn_yaw_last = start_yaw;
    turn_target = delta;
    turn_accumulate = accumulate;
    turn_travel = 0.0f;
    turn_timeout_ms = (accumulate && fabsf(delta) >= 180.0f) ?
                      TURN_FULL_TIMEOUT_MS : TURN_TIMEOUT_MS;
    posrun_start = xTaskGetTickCount();
    PID_Init(&TurnPID);
    TurnPID.Kp = TURN_PID_KP;  TurnPID.Ki = TURN_PID_KI;  TurnPID.Kd = TURN_PID_KD;
    TurnPID.ErrorIntMax = TURN_ERR_INT_MAX;  TurnPID.ErrorIntMin = -TURN_ERR_INT_MAX;
    TurnPID.OutMax = TURN_OUT_MAX_RPM;  TurnPID.OutMin = -TURN_OUT_MAX_RPM;
    turn_w_last = 0;
    turn_output_ratio = 1.0f;
    turn_settle_ticks = 0;
    turn_sample_sequence = IMU_UpdateSequence();
    Motor_Enable(true);
    phase = PHASE_TURN;
}

/* 同步带绝对坐标0~10000：上电位0、抓取1540、中心7050。
   dir表示绝对目标符号，不按当前到目标的增减切换；驱动器自行决定运动方向。 */
static void belt_pos_goto(int32_t target)
{
    if (target < 0 || target > (int32_t)BELT_POS_MAX_PUL) return;
    Belt_ZDT_ReachedClear();
    Emm_V5_En_Control(BELT_ZDT_ADDR, true, false);
    Emm_V5_Pos_Control(BELT_ZDT_ADDR, BELT_POS_POSITIVE_DIR,
                       BELT_POS_SPEED_RPM, BELT_POS_ACC, (uint32_t)target, true, false);
}

/* ---- 启动一个步骤: 按类型设置对应阶段参数并切换 phase ---- */
static void task_step_start(const TaskStep *st)
{
    posrun_start = xTaskGetTickCount();
    Motor_Enable(true);

    switch (st->type)
    {
    case STEPT_DIAG:                        /*45°斜走(带yaw保持, 段末判定走过|target|)*/
    {
        uint8_t direction = (st->dir == 5) ? 1 : (st->dir == 4) ? 2 : (st->dir == 7) ? 3 : 4;
        leg1_next_valid = 0;
        diag_run_start(direction, st->target, IMU_GetYaw());
        break;
    }

    case STEPT_LINE:                        /*直线PID(里程计1反馈/前后通道), target=相对步长*/
    {
        OdomData_t o0;
        odometry_get(&o0);
        pos_target = o0.enc_pos[ODOM_POS_AXIS] + st->target;
        pos_run_start(o0.enc_pos[ODOM_POS_AXIS]);
        yaw_hold    = IMU_GetYaw();        /*本步保持朝向=启动时刻车头*/
        PID_Init(&PosPID);
        PosPID.Kp = POS_PID_KP;  PosPID.Ki = POS_PID_KI;  PosPID.Kd = POS_PID_KD;
        PosPID.ErrorIntMax =  POS_ERR_INT_MAX;  PosPID.ErrorIntMin = -POS_ERR_INT_MAX;
        PosPID.OutMax =  POS_OUT_MAX_RPM;       PosPID.OutMin = -POS_OUT_MAX_RPM;
        PID_Init(&YawPID);
        line_yaw_reset();
            YawPID.Kp = YAW_PID_KP;  YawPID.Ki = YAW_PID_KI;  YawPID.Kd = YAW_PID_KD;
        YawPID.ErrorIntMax =  YAW_ERR_INT_MAX;  YawPID.ErrorIntMin = -YAW_ERR_INT_MAX;
        YawPID.OutMax =  YAW_OUT_MAX_RPM;       YawPID.OutMin = -YAW_OUT_MAX_RPM;
        phase = PHASE_LINE;
        break;
    }

    case STEPT_SLOW:                        /*缓行+扫码(里程计1反馈), target=相对步长*/
    {
        OdomData_t o0;
        odometry_get(&o0);
        pos_target = o0.enc_pos[ODOM_POS_AXIS] + st->target;
        pos_run_start(o0.enc_pos[ODOM_POS_AXIS]);
        yaw_hold    = IMU_GetYaw();        /*本步保持朝向=启动时刻车头*/
        PID_Init(&YawPID);
        line_yaw_reset();
            YawPID.Kp = YAW_PID_KP;  YawPID.Ki = YAW_PID_KI;  YawPID.Kd = YAW_PID_KD;
        YawPID.ErrorIntMax =  YAW_ERR_INT_MAX;  YawPID.ErrorIntMin = -YAW_ERR_INT_MAX;
        YawPID.OutMax =  YAW_OUT_MAX_RPM;       YawPID.OutMin = -YAW_OUT_MAX_RPM;
        phase = PHASE_SLOW;
        break;
    }

    case STEPT_TURN:                        /*原地转90°(dir=3顺/-90, 4逆/+90)*/
        turn_run_start(IMU_GetYaw(), (st->dir == 3) ? -90.0f : 90.0f, 0);
        break;

    default:
        phase = PHASE_IDLE;
        break;
    }
}

/* ---- 步骤完成: 推进索引; 一键模式自动启动下一步, 单步模式停车等下一帧 ---- */
static void task_step_done(void)
{
    Motor_Stop();
    step_idx++;
    if (step_idx >= TASK_Z1_N)
    {
        step_idx = 0;                               /*数组走完回0, 下条链从头*/
        array_mode = 0;                             /*本次连跑结束，停车等下一帧*/
    }
    if (array_mode)
        task_step_start(&task_z1[step_idx]);          /*一键: 无缝启动下一步*/
    else
        phase = PHASE_DONE;                           /*单步: 停车等下一动作帧*/
}

void MotorTask(void *pvParameters)
{
    (void)pvParameters;

    /* RTOS启动后等待1s，留出电机驱动器准备时间；上电提示音已在main中完成。 */
    vTaskDelay(pdMS_TO_TICKS(1000));

#if MOTOR_TASK_AUTO_DEMO
    Motor_Enable(true);   /* 四轮使能；若电机上电即自使能，这一步可注释掉 */

    for (;;)
    {
        motor_demo_step(0,  MOTOR_DEF_VEL, 0, 1000);   /* 前进      （vy 前后轴） */
        motor_demo_step(0, -MOTOR_DEF_VEL, 0, 1000);   /* 后退      （vy 前后轴） */
        motor_demo_step( MOTOR_DEF_VEL, 0, 0, 1000);   /* 左平移    （vx 左右轴） */
        motor_demo_step(-MOTOR_DEF_VEL, 0, 0, 1000);   /* 右平移    （vx 左右轴） */

        motor_demo_diag( 1,  1, 1000);                 /* 左前 45°  */
        motor_demo_diag( 1, -1, 1000);                 /* 右前 45°  */
        motor_demo_diag(-1,  1, 1000);                 /* 左后 45°  */
        motor_demo_diag(-1, -1, 1000);                 /* 右后 45°  */

        motor_demo_step(0, 0,  MOTOR_DEF_VEL, 1000);   /* 原地逆时针 */
        motor_demo_step(0, 0, -MOTOR_DEF_VEL, 1000);   /* 原地顺时针 */

        vTaskDelay(pdMS_TO_TICKS(2000));               /* 停 2s 再来一轮 */
    }
#else
    /* ---- 位置/转向闭环状态(串口 05/06 帧控制) ----
       当前阶段见 TaskPhase 枚举(0空闲/1斜走/2直线/3完成/4转向) */

    PID_Init(&PosPID);                         /*清运行状态, 增益随后重设*/
    PosPID.Kp = POS_PID_KP;  PosPID.Ki = POS_PID_KI;  PosPID.Kd = POS_PID_KD;
    PosPID.ErrorIntMax =  POS_ERR_INT_MAX;  PosPID.ErrorIntMin = -POS_ERR_INT_MAX;
    PosPID.OutMax =  POS_OUT_MAX_RPM;       PosPID.OutMin = -POS_OUT_MAX_RPM;
    PID_Init(&YawPID);
    line_yaw_reset();
    YawPID.Kp = YAW_PID_KP;  YawPID.Ki = YAW_PID_KI;  YawPID.Kd = YAW_PID_KD;
    YawPID.ErrorIntMax =  YAW_ERR_INT_MAX;  YawPID.ErrorIntMin = -YAW_ERR_INT_MAX;
    YawPID.OutMax =  YAW_OUT_MAX_RPM;       YawPID.OutMin = -YAW_OUT_MAX_RPM;

    for (;;)
    {
        ZbeeCmd c;

        IMU_Poll(ring_now_ms()); /* 保留统一轮询接口；两款均只接收，不自动配置。 */
        RDK_Center_Poll();

        if (cmd_get(&c))
        {
            /* 所选航向源未就绪时不启动闭环；两款均等待首个有效航向帧。 */
            if (IMU_ConfigStatus() != IMU_CONFIG_READY &&
                (c.type == CMD_POS_GO || c.type == CMD_POS_ADV || c.type == CMD_CHAIN ||
                 c.type == CMD_RUNALL ||
                 c.type == CMD_RDK_CENTER || c.type == CMD_RAW_RUN))
                Motor_Stop();
            else switch (c.type)
            {
            case CMD_ACTION:                       /*04路线动作只作上传记录/回传，不直接发车；由13帧启动*/
                break;

            case CMD_PULSE:
                if (!PHASE_CAN_START(phase)) break;        /*任务运行中: 忽略脉冲命令*/
                no_next_alarm = 0;
                cmd_tx_ack(&c);
                Motor_Enable(true);                        /* 确保四轮使能(FOC抱死) */
                vTaskDelay(pdMS_TO_TICKS(1000));           /* 接收后 1s 启动 */
                {
                    int32_t pulse4[MOTOR_NUM];
                    pulse_build(pulse4, c.dir, c.param);
                    Motor_MovePulses(pulse4);              /* 位置模式四轮同步, 走完自停 */
                }
                break;

            case CMD_POS_GO:
                cmd_tx_ack(&c);
                if (debug_pause_active || phase == PHASE_STATION_PLACE) break;
                if (PHASE_CAN_START(phase)) array_mode = 0;
                if (c.sub == 2 || c.sub == 3)          /*原地转向闭环: 2=左转 3=右转, param=角度deg*/
                {
                    float d = (float)c.param;
                    if (d < 1.0f)   d = 1.0f;
                    if (d > 179.0f) d = 179.0f;
                    if (!PHASE_CAN_START(phase)) break;    /*任务运行中: 忽略*/
                    turn_run_start(IMU_GetYaw(), (c.sub == 2) ? d : -d, 0);
                }
                else if (c.sub == 1 || c.sub == 4)     /*45°斜走段: 1=右前 4=左前, param=段末位置*/
                {
                    if (!PHASE_CAN_START(phase)) break;
                    leg1_next_valid = 0;
                    diag_run_start((c.sub == 1) ? 1 : 2, (int32_t)c.param, IMU_GetYaw());
                    Motor_Enable(true);
                }
                else if (c.sub == 5)                   /*缓行扫码段(单独触发): param=缓行目标pos*/
                {
                    OdomData_t o;
                    if (!PHASE_CAN_START(phase)) break;
                    odometry_get(&o);
                    pos_target   = (int32_t)c.param;
                    pos_run_start(o.enc_pos[ODOM_POS_AXIS]);
                    yaw_hold = IMU_GetYaw();
                    PID_Init(&YawPID);                 /*缓行段yaw保持用*/
                    line_yaw_reset();
                                    YawPID.Kp = YAW_PID_KP;  YawPID.Ki = YAW_PID_KI;  YawPID.Kd = YAW_PID_KD;
                    YawPID.ErrorIntMax =  YAW_ERR_INT_MAX;  YawPID.ErrorIntMin = -YAW_ERR_INT_MAX;
                    YawPID.OutMax =  YAW_OUT_MAX_RPM;       YawPID.OutMin = -YAW_OUT_MAX_RPM;
                    Motor_Enable(true);
                    phase = PHASE_SLOW;
                }
                else if (phase == PHASE_DIAG)          /*斜走段运行中: 排队为下一段(段末自动衔接直线)*/
                {
                    leg1_next       = (int32_t)c.param;
                    leg1_next_valid = 1;
                }
                else                                   /*直线PID段: param=目标位置*/
                {
                    OdomData_t o;
                    odometry_get(&o);
                    pos_target   = (int32_t)c.param;
                    pos_run_start(o.enc_pos[ODOM_POS_AXIS]);
                    yaw_hold = IMU_GetYaw();
                    PID_Init(&PosPID);                         /* 清运行状态 */
                    PosPID.Kp = POS_PID_KP;  PosPID.Ki = POS_PID_KI;  PosPID.Kd = POS_PID_KD;
                    PosPID.ErrorIntMax =  POS_ERR_INT_MAX;  PosPID.ErrorIntMin = -POS_ERR_INT_MAX;
                    PosPID.OutMax =  POS_OUT_MAX_RPM;       PosPID.OutMin = -POS_OUT_MAX_RPM;
                    PID_Init(&YawPID);
                    line_yaw_reset();
                                    YawPID.Kp = YAW_PID_KP;  YawPID.Ki = YAW_PID_KI;  YawPID.Kd = YAW_PID_KD;
                    YawPID.ErrorIntMax =  YAW_ERR_INT_MAX;  YawPID.ErrorIntMin = -YAW_ERR_INT_MAX;
                    YawPID.OutMax =  YAW_OUT_MAX_RPM;       YawPID.OutMin = -YAW_OUT_MAX_RPM;
                    Motor_Enable(true);
                    phase = PHASE_LINE;
                }
                break;

            case CMD_POS_ADV:                         /*08高级运动: 直线/斜行/原地转向*/
                cmd_tx_ack(&c);
                if (!PHASE_CAN_START(phase)) break;
                if (c.sub == 2)                         /*绝对目标yaw: P=0, FLAGS=bit1*/
                {
                    float now_yaw = IMU_GetYaw();
                    float target_yaw = yaw_abs360_to_signed((float)c.yaw_target);
                    array_mode = 0;
                    turn_run_start(now_yaw, turn_angdiff(target_yaw, now_yaw), 0);
                    break;
                }
                if (c.sub == 3 || c.sub == 4)            /*相对左/右转0~360°，保持所选方向*/
                {
                    float delta = (float)c.param;
                    array_mode = 0;
                    turn_run_start(IMU_GetYaw(), (c.sub == 3) ? delta : -delta, 1);
                    break;
                }
                if (c.sub == 1 || c.sub == 5)
                {
                    array_mode = 0;
                    leg1_next_valid = 0;
                    diag_run_start((c.sub == 1) ? 1 : 3, (int32_t)c.param, (c.flags & 0x02) ?
                                   yaw_abs360_to_signed((float)c.yaw_target) : IMU_GetYaw());
                    Motor_Enable(true);
                    break;
                }
                if (c.sub != 0) break;
                {
                    OdomData_t o;
                    int32_t current;
                    odometry_get(&o);
                    array_mode = 0;
                    current = o.enc_pos[ODOM_POS_AXIS];
                    if (c.flags & 0x01)
                        pos_target = current + (int32_t)c.param; /*相对当前位置累加*/
                    else
                        pos_target = (int32_t)c.param;            /*绝对编码目标*/
                    yaw_hold = (c.flags & 0x02) ?
                               yaw_abs360_to_signed((float)c.yaw_target) : IMU_GetYaw();
                    pos_run_start(current);
                    PID_Init(&PosPID);
                    PosPID.Kp = POS_PID_KP;  PosPID.Ki = POS_PID_KI;  PosPID.Kd = POS_PID_KD;
                    PosPID.ErrorIntMax =  POS_ERR_INT_MAX;  PosPID.ErrorIntMin = -POS_ERR_INT_MAX;
                    PosPID.OutMax =  POS_OUT_MAX_RPM;  PosPID.OutMin = -POS_OUT_MAX_RPM;
                    PID_Init(&YawPID);
                    line_yaw_reset();
                                    YawPID.Kp = YAW_PID_KP;  YawPID.Ki = YAW_PID_KI;  YawPID.Kd = YAW_PID_KD;
                    YawPID.ErrorIntMax =  YAW_ERR_INT_MAX;  YawPID.ErrorIntMin = -YAW_ERR_INT_MAX;
                    YawPID.OutMax = YAW_OUT_MAX_RPM;  YawPID.OutMin = -YAW_OUT_MAX_RPM;
                    Motor_Enable(true);
                    phase = PHASE_LINE;
                }
                break;

            case CMD_RUNALL:                       /*0x12一键: 按任务数组从step_idx连跑至尾*/
                cmd_tx_ack(&c);
                if (!PHASE_CAN_START(phase)) break;
                array_mode = 1;
                NVIC_SetPriority(UART7_IRQn, IRQ_PRIO_UART7_IDLE); /*启动后不再收路线*/
                task_step_start(&task_z1[step_idx]);
                break;

            case CMD_DEBUG_RUN:                    /*0x13: 路线已上传后的调试一键启动*/
                cmd_tx_ack(&c);
                Motor_DebugRunStart((uint8_t)c.param);
                break;

            case CMD_RING_ALIGN:
                if (c.sub == 3 || c.sub == 4)
                {
                    if (phase != PHASE_RDK_RING_WAIT &&
                        (!PHASE_CAN_START(phase) || debug_route_active || debug_pause_active)) break;
                    array_mode = 0;
                    leg1_next_valid = 0;
                    Motor_Enable(true);
                    debug_rdk_ring_start();
                    debug_rdk_ring_serial = 1;
                    RingAlign_StartMode(c.sub, (int8_t)c.dir, ring_now_ms());
                    cmd_tx_ack(&c);
                    break;
                }
                if (c.sub == 0)
                {
                    if (phase != PHASE_RDK_RING_WAIT &&
                        (!PHASE_CAN_START(phase) || debug_route_active || debug_pause_active)) break;
                    array_mode = 0;
                    leg1_next_valid = 0;
                    Motor_Enable(true);
                    debug_rdk_ring_start();
                    debug_rdk_ring_serial = 1;
                    cmd_tx_ack(&c);
                }
                else
                {
                    RDK_RingResult result;
                    if (c.sub == 1 && phase != PHASE_RDK_RING_WAIT) break;
                    if (c.sub == 2 && phase != PHASE_RDK_RING_WAIT &&
                        (!PHASE_CAN_START(phase) || debug_route_active || debug_pause_active)) break;
                    RDK_Ring_GetResult(&result);
                    if (RingAlign_Command(c.sub, (int8_t)c.dir, &result, ring_now_ms()))
                    {
                        Motor_Stop();
                        cmd_tx_ack(&c);
                    }
                }
                break;

            case CMD_STACK_ALIGN:              /*0x1D: 启动码垛对位(0x14单中心点)*/
                if (!PHASE_CAN_START(phase) || debug_route_active || debug_pause_active ||
                    RDK_Ring_Active() || RDK_Stack_Active())
                    break;
                Motor_Enable(true);   /*防御: 独立入口自行保证四轮使能*/
                debug_rdk_stack_start(1);
                cmd_tx_ack(&c);
                break;

            case CMD_ARM_ANGLE:
                if (!PHASE_CAN_START(phase) || debug_route_active || debug_pause_active ||
                    RDK_Ring_Active())
                    break;
                if (Arm_M2006_SetTargetDeg((int16_t)(int32_t)c.param))
                {
                    manual_arm_start_tick = xTaskGetTickCount();
                    phase = PHASE_ARM_MANUAL;
                    Motor_Stop();
                    cmd_tx_ack(&c);
                }
                break;

            case CMD_LIFT_POS:                   /*0x17: 丝杆绝对位置调试，速度共用LIFT_POS_SPEED_RPM*/
                if (!PHASE_CAN_START(phase) || debug_route_active || debug_pause_active ||
                    RDK_Ring_Active())
                    break;
                Motor_Stop();                    /*调试期间底盘保持停车，ZDT闭环到位自停*/
                /* Emm_V5_Pos_Control的clk是无符号幅值，方向由dir字节单独携带
                   (0=CW向上,1=CCW向下)，绝对模式目标=±clk，不能发补码。
                   帧值直接使用新丝杆脉冲，不再除以4；下行方向由LIFT_POS_DOWN_DIR选择。
                   堵转保护由上电回零流程负责解除(回零前后各一次)；运行中顶到机械
                   端点锁存(屏显Protect!)时，重新上电回零或按驱动器按键清除。 */
                Emm_V5_Pos_Control(LIFT_ZDT_ADDR, LIFT_POS_DOWN_DIR,
                                   LIFT_POS_SPEED_RPM, LIFT_POS_ACC,
                                   c.param, true, false);
                cmd_tx_ack(&c);
                break;

            case CMD_SERVO_ANGLE:              /*0x18: 两舵机角度调试(180°PA1/270°PA0)*/
                if (!PHASE_CAN_START(phase) || debug_route_active || debug_pause_active ||
                    RDK_Ring_Active())
                    break;
                Motor_Stop();                /*调试期间底盘保持停车*/
                Servo_SetAngle((float)(c.param & 0xFFFFu));
                Servo270_SetAngle((float)(c.param2 & 0xFFFFu));
                cmd_tx_ack(&c);
                break;

            case CMD_BELT_POS:                /*0x19: 同步带坐标调试(地址5, FD绝对)*/
                if (!PHASE_CAN_START(phase) || debug_route_active || debug_pause_active ||
                    RDK_Ring_Active())
                    break;
                Motor_Stop();                /*调试期间底盘保持停车*/
                /* 新坐标: 上电位0、抓取1540、中心7050，绝对定位。 */
                belt_pos_goto((int32_t)c.param);
                cmd_tx_ack(&c);
                break;

            case CMD_TASK_INJECT:                /*0x1B: 网页注入任务码(模拟扫码)*/
            {
                /* 6字节解包回12位数字: D_i=digit[2i]<<4|digit[2i+1]。合法注入并
                   回ACK; 含非法数字(0/7~9)不ACK, 网页无回显即失败。 */
                uint8_t digits[TASK_LINK_DIGITS];
                uint8_t pack[6];
                uint8_t i;
                pack[0] = (uint8_t)(c.param >> 24); pack[1] = (uint8_t)(c.param >> 16);
                pack[2] = (uint8_t)(c.param >> 8);  pack[3] = (uint8_t)(c.param);
                pack[4] = (uint8_t)(c.param2 >> 8); pack[5] = (uint8_t)(c.param2);
                for (i = 0; i < 6; i++)
                {
                    digits[2 * i]     = (uint8_t)('0' + (pack[i] >> 4));
                    digits[2 * i + 1] = (uint8_t)('0' + (pack[i] & 0x0Fu));
                }
                if (UART8_InjectTask(digits))
                    cmd_tx_ack(&c);
            }
            break;

            case CMD_STATION_RUN: /*22：独立粗加工/暂存：视觉对位后取放，结束停车*/
                if (station_debug_start((uint8_t)c.param)) cmd_tx_ack(&c);
                break;

            case CMD_PLACE_SEQ:                  /*0x1C: 独立放三块，粗加工再取回；不做视觉、不消费路线*/
                if (!PHASE_CAN_START(phase) || debug_route_active || debug_pause_active ||
                    RDK_Ring_Active() || c.param < 1u || c.param > 4u)
                    break;
                if (station_place_begin((c.param == 2u || c.param == 4u) ? 2u : 1u,
                                        (uint8_t)(c.param >= 3u), 1))
                    cmd_tx_ack(&c);
                break;

            case CMD_RAW_RUN: /*1E：摆在原料区，启动朝向记90°，完成三块后停车*/
                if (!PHASE_CAN_START(phase) || debug_route_active || debug_pause_active ||
                    RDK_Ring_Active() || RDK_Stack_Active() ||
                    c.param < 1u || c.param > 2u || !UART8_Valid() || g_link.bad ||
                    !Arm_M2006_Ready() || IMU_UpdateSequence() == 0u)
                    break;
                Motor_Stop();
                IMU_SetYaw(RDK_CENTER_RAW_YAW);
                array_mode = 0;
                leg1_next_valid = 0;
                raw_debug_full = 1;
                debug_route_batch = (uint8_t)c.param;
                grab_pick_index = 0;
                grab_pick_fixed = 0xFFu;
                grab_manual = grab_trig_auto = 0;

                debug_raw_prepare();
                if (phase == PHASE_RDK_ARM_PRE)
                {
                    NVIC_SetPriority(UART7_IRQn, IRQ_PRIO_UART7_IDLE); /*启动后不再收路线*/
                    cmd_tx_ack(&c);
                }
                break;

            case CMD_GRAB_SEQ:                   /*0x1A: 网页手动触发统一夹取序列*/
                if (!PHASE_CAN_START(phase) || debug_route_active || debug_pause_active ||
                    RDK_Ring_Active())
                    break;
                Motor_Stop();
                grab_fault.reason = MOTOR_GRAB_FAULT_NONE;
                grab_manual = 1;                 /*手动: 序列完成后直接停车结束*/
                grab_pick_fixed = (c.param >= 1u && c.param <= 3u) ?
                                  (uint8_t)(c.param - 1u) : 0xFFu;
                {
                    /* 2026-11-03: 任务码已注入时与路线流程一致——发颜色给RDK,
                       等新版0x11三区报告，匹配后按区域自动抓取(30s首报告超时no next);
                       未注入/无效时无颜色可发, 退回纯动作调试直接夹。 */
                    uint8_t pick = (grab_pick_fixed != 0xFFu) ? grab_pick_fixed :
                                   (grab_pick_index < 3u ? grab_pick_index : 2u);

                    if (UART8_Valid() && !g_link.bad)
                        grab_color_start(pick);
                    else
                    {
                        grab_step_index = 0;
                        grab_sub = 0;
                        phase = PHASE_RDK_GRAB_SEQ;
                    }
                }
                cmd_tx_ack(&c);
                break;

            case CMD_RDK_CENTER:                   /*0x14: 上电后直接视觉定位，保持当前航向*/
                if (!PHASE_CAN_START(phase) || debug_route_active || debug_pause_active)
                    break;
                array_mode = 0;
                leg1_next_valid = 0;
                debug_rdk_standalone = 1;
                Motor_Enable(true);
                debug_rdk_center_start(IMU_GetYaw(),
                    (c.flags & 1) ? (int8_t)c.dir : RDK_CENTER_X_SIGN,
                    (c.flags & 1) ? (int8_t)c.sub : RDK_CENTER_Y_SIGN);
                cmd_tx_ack(&c);
                break;

            case CMD_CHAIN:                        /*一键任务链: dir=1右前/4左前
                                                     param=45°段末, param2=直线目标;
                                                     后续自动: 衔接直线->缓行->扫码停车*/
                cmd_tx_ack(&c);
                if (!PHASE_CAN_START(phase)) break;    /*任务运行中: 忽略(先发06中止)*/
                array_mode = 0;
                diag_run_start((c.dir == 4) ? 2 : 1, (int32_t)c.param, IMU_GetYaw());
                leg1_next       = (int32_t)c.param2;
                leg1_next_valid = 1;
                Motor_Enable(true);
                break;

            case CMD_POS_ABORT:
                grab_region_clear();
                key_start_request = 0;
                start_message = START_MSG_NONE;
                no_next_retry = NEXT_RETRY_NONE;
                odom_wait_active = 0; /*06取消自动续跑；后续有效回包不能重新启动车辆*/
                run_fault = MOTOR_RUN_FAULT_NONE;
                NVIC_SetPriority(UART7_IRQn, IRQ_PRIO_UART7_UPLOAD); /*06后回到可上传状态*/
                raw_debug_full = 0;

                grab_fault.reason = MOTOR_GRAB_FAULT_NONE;
                Arm_M2006_Stop();
                Emm_V5_Stop_Now(LIFT_ZDT_ADDR, false);   /*丝杆调试运动同样可被06中止*/
                Emm_V5_Stop_Now(BELT_ZDT_ADDR, false);   /*同步带调试运动同理*/
                debug_rdk_ring_stop();
                debug_rdk_standalone = 0;
                array_mode = 0;
                no_next_alarm = 0;
                debug_route_release();
                debug_route_active = 0;
                debug_route_step = 0;
                if (debug_pause_active) Buzzer_Off();
                debug_pause_active = 0;
                debug_pause_next = 0;
                debug_qr_scan_active = 0;
                if (debug_rdk_center_active || debug_rdk_center_sent)
                    RDK_Center_SendIdle();
                debug_rdk_center_active = 0;
                debug_rdk_center_sent = 0;
                debug_rdk_center_settle_count = 0;
                debug_rdk_center_vx = 0.0f;
                debug_rdk_center_vy = 0.0f;
                RDK_Center_Clear();
                RDK_Center_Enable(0);
                if (debug_rdk_pos_active)
                {
                    debug_rdk_pos_sequence = odometry_encoder_sequence();
                    debug_rdk_resume_start_tick = xTaskGetTickCount();
                    debug_rdk_resume_action = 0;
                    phase = PHASE_RDK_RESUME;
                }
                else
                    phase = PHASE_IDLE;
                debug_turn_pos_active = 0;
                leg1_next_valid = 0;
                turn_w_last = 0;
                Motor_Stop();                              /* 中止: 立即停车 */
                cmd_tx_ack(&c);                             /*先停车再应答*/
                break;

            default:
                break;
            }
        }

        motor_key_start_poll();
        debug_approach_poll();

        if (debug_no_next_retry_run())
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (debug_pause_active)
        {
            if ((int32_t)(xTaskGetTickCount() - debug_pause_until) >= 0)
            {
                Buzzer_Off();   /*双声序列已删, 恒执行*/
                debug_pause_active = 0;
                if (debug_pause_next == 1)
                {
                    debug_pause_next = 0;
                    debug_leg_start();
                }
                else if (debug_pause_next == 2)
                {
                    debug_pause_next = 0;
                    debug_motion_start();
                }
                else if (debug_pause_next == 3)
                {
                    debug_pause_next = 0;
                    if (debug_leg_load()) debug_leg_start();
                    else debug_no_next_retry_start(NEXT_RETRY_LOAD);
                }
                else if (debug_pause_next == 4)
                {
                    /* 扫码成功：2026-11-15取消双声序列, 直接无缝衔接下一段路线。 */
                    debug_pause_next = 0;
                    debug_leg_start();
                }
                else
                {
                    debug_pause_next = 0;
                    debug_route_release();
                    debug_route_active = 0;
                    debug_route_step = 0;
                    phase = debug_rdk_pos_active ? PHASE_RDK_RESUME : PHASE_DONE;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (odom_motion_wait())
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (phase == PHASE_RDK_RING_WAIT)
        {
            RDK_RingResult result;
            RingAlignMotion motion;
            /* 真实站点与内部取放一致：反馈失效停车，不按2秒到位超时放行。 */
            if (!debug_rdk_ring_serial && !Arm_M2006_Ready())
            {
                station_place_fail();
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            if (debug_rdk_stack_mode)
            {
                /* 码垛0x14帧 -> 单点RingResult: id=1, flags兼当yaw_valid
                   (探照帧FLAGS=0时mode5停车等待)。 */
                RDK_StackResult stack;
                RDK_Stack_GetResult(&stack);
                result.points[0].id = 1;
                result.points[0].x = stack.x;
                result.points[0].y = stack.y;
                result.count = 1;
                result.identified = 1;
                result.yaw_valid = stack.flags;
                result.yaw_cdeg = stack.yaw_cdeg;
                result.fresh = stack.fresh;
                result.age_ms = stack.age_ms;
                result.sequence = stack.sequence;
            }
            else
                RDK_Ring_GetResult(&result);
            if (RDK_VisionWaitingFirst()) RingAlign_ResetTimeout(ring_now_ms());
            RingAlign_Update(&result, ring_now_ms(), &motion);
            if (!debug_rdk_ring_serial && RingAlign_Done())
            {
                /* 视觉仍并行；只在视觉先结束时等190°到位，或从下发起满2秒。
                   超时仅结束等待，不停止RM，随后取料目标沿原流程接管。 */
                if (Arm_M2006_AtTargetWithin(GRAB_ARM_POS_TOL_DEG) ||
                    (xTaskGetTickCount() - debug_ring_arm_start_tick) >=
                    pdMS_TO_TICKS(PLACE_ARM_WAIT_LIMIT_MS))
                    station_place_start();
                else
                    Motor_Stop();
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            if (debug_rdk_ring_serial && RingAlign_Done())
            {
                Motor_Stop();
                debug_rdk_ring_stop();
                phase = PHASE_DONE;
                debug_point_pause_start(0, 500); /* Nonblocking completion beep, no route advance. */
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            if (!motion.vx && !motion.vy && !motion.w) Motor_Stop();
            else Motor_Move(motion.vx, motion.vy, motion.w);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (phase == PHASE_STATION_PLACE)
        {
            station_place_run();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (phase == PHASE_ARM_MANUAL)
        {
            Motor_Stop();
            if (!Arm_M2006_Ready() ||
                (xTaskGetTickCount() - manual_arm_start_tick) >=
                    pdMS_TO_TICKS(ARM_M2006_MOVE_TIMEOUT_MS))
            {
                Arm_M2006_Stop();
                phase = PHASE_DONE;
            }
            else if (Arm_M2006_AtTarget())
                phase = PHASE_DONE;
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (phase == PHASE_RDK_CHASSIS_HOLD)
        {
            debug_raw_chassis_hold_run();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /*---- 转盘定位后机构预定位：底盘保持固定位置，里程计持续读取 ----*/
        if (phase == PHASE_RDK_ARM_POST)
        {
            /* 对位完成仅等待RM435°，丝杆保持2000，不下发21000也不再等待丝杆。 */

            if (!raw_chassis_held) Motor_Stop();
            if (!Arm_M2006_Ready())
            {
                Arm_M2006_Stop();
                grab_fail(MOTOR_GRAB_FAULT_ARM);
            }
            else
            {
                if (grab_arm_done())
                {
                    /* 预定位完成: 按本批块序发颜色, 开触发通道, 等RDK触发帧。 */
                    uint8_t pick = grab_pick_index;

                    if (pick > 2u) pick = 2u;
                    grab_color_start(pick);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (phase == PHASE_RDK_CENTER)
        {
            debug_rdk_center_run();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (phase == PHASE_RDK_ARM_PRE)
        {
            Motor_Stop();
            debug_raw_prepare_run();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (phase == PHASE_RDK_GRAB_TRIG)
        {
            RDK_GrabReport report;
            if (!raw_chassis_held) Motor_Stop();
            if (RDK_GrabTakeReport(&report) && report.state == RDK_GRAB_STATE_READY &&
                report.color == grab_expect_color &&
                report.region >= RDK_GRAB_REGION_NEAR && report.region <= RDK_GRAB_REGION_RIGHT)
            {
                taskENTER_CRITICAL();
                grab_region.color = report.color;
                grab_region.region = report.region;
                taskEXIT_CRITICAL();
                RDK_GrabArmEnable(0, 0); /* 先关闭重复报告，再按已锁存区域开始一次抓取。 */
                raw_region_start();
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            if ((xTaskGetTickCount() - grab_trig_tick) >= pdMS_TO_TICKS(GRAB_TRIG_TIMEOUT_MS))
                grab_fail(MOTOR_GRAB_FAULT_RDK); /* 等首个匹配报告仍限30秒。 */
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (phase == PHASE_RDK_GRAB_REGION)
        {
            raw_region_run();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (phase == PHASE_RDK_GRAB_SEQ)
        {
            const GrabStep *st;
            uint8_t pk;

            if (!raw_chassis_held) Motor_Stop();
            if (grab_step_index >= GRAB_STEP_NUM)
            {
                grab_pick_complete();
            }
            else
            {
                uint8_t step_done = 0;

                pk = (grab_pick_fixed != 0xFFu) ? grab_pick_fixed :
                     (grab_pick_index < 3u ? grab_pick_index : 2u);
                st = &grab_steps[grab_step_index];
                if (grab_sub == 0)
                {
                    grab_wait_start();
                    grab_sub = 1;
                    switch (st->type)
                    {
                    case GRAB_ST_SERVO180: Servo_SetAngle(st->deg); break;
                    case GRAB_ST_SERVO270:
                    {
                        /* 末步270°舵机取角: 固定块序或按本块序号, 150/270/30。 */
                        Servo270_SetAngle(grab_pick_servo270[pk]);
                        break;
                    }
                    case GRAB_ST_ARM:
                        if (!Arm_M2006_SetTargetDeg((int16_t)st->deg))
                            grab_fail(MOTOR_GRAB_FAULT_ARM);
                        break;
                    case GRAB_ST_BELT_ARM:
                        grab_parallel_belt_done = 0;
                        if (!Arm_M2006_SetTargetDeg((int16_t)st->deg))
                            grab_fail(MOTOR_GRAB_FAULT_ARM);
                        else
                            belt_pos_goto(st->target);
                        break;
                    case GRAB_ST_BELT_SERVO270_ARM:
                    {
                        uint8_t last = (uint8_t)(pk == 2u);
                        grab_parallel_belt_done = 0;
                        if (!Arm_M2006_SetTargetDeg(last ? RDK_RAW_FINISH_ARM_DEG : (int16_t)st->deg))
                            grab_fail(MOTOR_GRAB_FAULT_ARM);
                        else
                        {
                            /* 第三块(收尾)同步带保持1540取料位不回7040(2026-11-07)；
                               前两块仍回7040原料区中心。 */
                            belt_pos_goto(last ? BELT_POS_GRAB_PUL : st->target);
                            /* 粗加工先放扫码块序0，复用其车载槽位映射，不按环号选槽；
                               180°及270°舵机只下发目标，不增加驻留/到位等待。 */
                            if (last)
                            {
                                Servo_SetAngle(RDK_RAW_SERVO180_DEG);
                                Servo270_SetAngle(grab_pick_servo270_drop[0]);
                            }
                            else
                                Servo270_SetAngle(grab_pick_servo270[pk]);
                            /* 丝杆第三块回12500，前两块仍回预定位21000；回读目标=-clk。 */
                            Lift_ZDT_ReachedClear();
                            Emm_V5_Pos_Control(LIFT_ZDT_ADDR, LIFT_POS_DOWN_DIR,
                                               LIFT_POS_SPEED_RPM, LIFT_POS_ACC,
                                               last ? RDK_RAW_FINISH_LIFT_POS : RDK_RAW_LIFT_POS,
                                               true, false);
                        }
                        break;
                    }
                    case GRAB_ST_LIFT:
                        /* clk=无符号幅值, 方向由dir字节携带(1=CCW向下), 绝对目标。
                           发命令前清到位回传标志, 随后等9F到位帧。 */
                        Lift_ZDT_ReachedClear();
                        Emm_V5_Pos_Control(LIFT_ZDT_ADDR, LIFT_POS_DOWN_DIR,
                                           LIFT_POS_SPEED_RPM, LIFT_POS_ACC,
                                           st->target, true, false);
                        break;
                    default: /* GRAB_ST_BELT */
                        belt_pos_goto(st->target);
                        break;
                    }
                }
                else if (st->type == GRAB_ST_SERVO270)
                {
                    step_done = 1;   /* 270°舵机交给硬件执行，不驻留。 */
                }
                else if (st->type == GRAB_ST_SERVO180)
                {
                    step_done = (uint8_t)((xTaskGetTickCount() - grab_step_tick) >=
                                          pdMS_TO_TICKS(GRAB_SERVO_DWELL_MS));
                }
                else if (st->type == GRAB_ST_ARM)
                {
                    if (!Arm_M2006_Ready())
                        grab_fail(MOTOR_GRAB_FAULT_ARM);
                    else if (grab_arm_done())
                        step_done = 1;
                }
                else if (st->type == GRAB_ST_BELT_ARM)
                {
                    step_done = grab_parallel_belt_arm_done();
                }
                else if (st->type == GRAB_ST_BELT_SERVO270_ARM)
                {
                    /* 舵机只发目标、不等到位；丝杆/同步带等9F或1/2秒放行，RM到位或1.5秒放行。 */
                    step_done = grab_parallel_belt_arm_done();
                    if (phase == PHASE_RDK_GRAB_SEQ)
                    {
                        uint8_t lift_done = grab_axis_done(1);
                        step_done = (uint8_t)(step_done && lift_done);
                    }
                }
                else /* GRAB_ST_LIFT / GRAB_ST_BELT */
                    step_done = grab_axis_done((uint8_t)(st->type == GRAB_ST_LIFT));
                if (step_done) { grab_step_index++; grab_sub = 0; }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (phase == PHASE_RDK_ARM_EXTEND)
        {
            debug_rdk_arm_run();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (phase == PHASE_RDK_GRAB_WAIT)
        {
            if (!raw_chassis_held) Motor_Stop();
            if (debug_rdk_grab_done)
            {
                debug_rdk_grab_done = 0;
                debug_rdk_pos_sequence = odometry_encoder_sequence();
                debug_rdk_resume_start_tick = xTaskGetTickCount();
                debug_rdk_resume_action = 3;
                phase = PHASE_RDK_RESUME;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (phase == PHASE_RDK_RESUME)
        {
            debug_rdk_center_resume_run();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (phase == PHASE_RDK_RETURN)
        {
            OdomData_t o;
            float err, aerr, out;
            PosRunState state;
            odometry_get(&o);
            err = (float)pos_target - (float)o.enc_pos[ODOM_POS_AXIS];
            aerr = fabsf(err);
            state = pos_run_check(o.enc_pos[ODOM_POS_AXIS], o.online[ODOM_POS_AXIS]);
            if (state == POS_PAUSED)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            if (state == POS_REACHED)
            {
                if (!pos_finish_soft(o.enc_pos[ODOM_POS_AXIS]))
                {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue;
                }
                debug_rdk_center_return_done();
            }
            else if (state == POS_FAILED)
                pos_run_abort();
            else
            {
                PosPID.Target = (float)pos_target;
                PosPID.Actual = (float)o.enc_pos[ODOM_POS_AXIS];
                PID_Update(&PosPID);
                out = pos_speed_limit(PosPID.Out, aerr);
                Motor_Move(0, (int)out,
                           line_yaw_output(yaw_hold, (int)out));
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /*---- 45°斜走段(匀速+航向保持; 本段纵向行程到位 -> 停车或衔接直线) ----
           移动方向由 vx:vy=1:1 轮速比决定, 车头朝向由 yaw 闭环独立锁住 —— 麦轮两通道解耦 */
        if (phase == PHASE_DIAG)
        {
            OdomData_t o;
            float travel, remaining;
            int diagonal_rpm = POS_LEG1_RPM;
            odometry_get(&o);
            travel = fabsf((float)o.enc_pos[ODOM_POS_AXIS] - (float)diag_start);
            remaining = fabsf((float)leg1_end) - travel;
            if (remaining <= POS_LEG1_APPROACH_COUNTS)
            {
                float ratio = remaining > 0.0f ? remaining / POS_LEG1_APPROACH_COUNTS : 0.0f;
                /*smoothstep：减速带两端斜率为0，避免60→40的突降；
                  轮速按偶数取整，与下面两个平移分量的整数除2一致。*/
                float speed = POS_LEG1_APPROACH_RPM +
                              (POS_LEG1_RPM - POS_LEG1_APPROACH_RPM) *
                              ratio * ratio * (3.0f - 2.0f * ratio);
                diagonal_rpm = 2 * (int)(speed / 2.0f + 0.5f);
            }

            {
                float start_ratio = (float)(xTaskGetTickCount() - pos_ramp_start) /
                                    (float)pdMS_TO_TICKS(POS_LEG1_START_MS);
                float error = turn_angdiff(yaw_hold, IMU_GetYaw());
                float radians, cosine, sine;
                int vx, vy, w;
                if (start_ratio < 1.0f)
                {
                    float start_cap = POS_LEG1_START_RPM +
                        (POS_LEG1_RPM - POS_LEG1_START_RPM) *
                        start_ratio * start_ratio * (3.0f - 2.0f * start_ratio);
                    int start_rpm = 2 * (int)(start_cap / 2.0f + 0.5f);
                    if (diagonal_rpm > start_rpm) diagonal_rpm = start_rpm;
                }
                if (debug_route_active && debug_route_step == 1u)
                    diagonal_rpm = 2 * (int)(diagonal_rpm * POS_ROUTE_DIAG_SPEED_SCALE / 2.0f + 0.5f);
                vx = (leg_dir == 1 || leg_dir == 3) ? -diagonal_rpm / 2 : diagonal_rpm / 2;
                vy = (leg_dir == 1 || leg_dir == 2) ? diagonal_rpm / 2 : -diagonal_rpm / 2;
                if (error > LINE_DIRECTION_MAX_DEG) error = LINE_DIRECTION_MAX_DEG;
                if (error < -LINE_DIRECTION_MAX_DEG) error = -LINE_DIRECTION_MAX_DEG;
                radians = error * 0.01745329252f;
                cosine = cosf(radians);
                sine = sinf(radians);
                /*同直行坐标变换，四个斜向均保持目标yaw下的平移方向。
                  修正限幅以半轮速为基准，减速时避免旋转盖过平移。*/
                w = line_yaw_output(yaw_hold, diagonal_rpm / 2);
                Motor_Move(line_rpm_round(vx * cosine - vy * sine),
                           line_rpm_round(vx * sine + vy * cosine), w);
            }

            if (travel >= fabsf((float)leg1_end))
            {
                if (!array_mode && leg1_next_valid)
                {
                    /*07旧链: 无缝衔接直线段(里程计累计不清, 目标绝对值直接生效)*/
                    pos_target   = leg1_next;
                    pos_run_start(o.enc_pos[ODOM_POS_AXIS]);
                    leg1_next_valid = 0;
                    PID_Init(&PosPID);
                    PosPID.Kp = POS_PID_KP;  PosPID.Ki = POS_PID_KI;  PosPID.Kd = POS_PID_KD;
                    PosPID.ErrorIntMax =  POS_ERR_INT_MAX;  PosPID.ErrorIntMin = -POS_ERR_INT_MAX;
                    PosPID.OutMax =  POS_OUT_MAX_RPM;       PosPID.OutMin = -POS_OUT_MAX_RPM;
                    PID_Init(&YawPID);
                    line_yaw_reset();
                                    YawPID.Kp = YAW_PID_KP;  YawPID.Ki = YAW_PID_KI;  YawPID.Kd = YAW_PID_KD;
                    YawPID.ErrorIntMax =  YAW_ERR_INT_MAX;  YawPID.ErrorIntMin = -YAW_ERR_INT_MAX;
                    YawPID.OutMax =  YAW_OUT_MAX_RPM;       YawPID.OutMin = -YAW_OUT_MAX_RPM;
                    phase = PHASE_LINE;                /*无缝衔接: 进入直线行驶段*/
                }
                else
                {
                    if (debug_route_active)
                    {
                        debug_motion_done();
                    }
                    else
                    {
                        task_step_done();              /*段末: 推进步骤(一键自动下一步)*/
                    }
                }
            }
            else if ((xTaskGetTickCount() - posrun_start) > pdMS_TO_TICKS(POS_RUN_TIMEOUT_MS))
            {
                run_fault = MOTOR_RUN_FAULT_TIMEOUT;
                pos_run_abort();
            }
        }

        /*---- 原地转向闭环(10ms周期; 直接读所选陀螺仪新角度帧，连续3帧确认到位) ----*/
        if (phase == PHASE_TURN)
        {
            float rel_now, err, aerr;
            float turn_scale = 1.0f;
            float turn_transition = 0.0f;
            float turn_min_rpm;
            float turn_speed_cap;
            IMU_Sample_t sample;
            IMU_GetSample(&sample);
            uint8_t fresh_sample = (uint8_t)(sample.sequence != turn_sample_sequence);
            turn_sample_sequence = sample.sequence;

            if (turn_accumulate)
            {
                float current_yaw = sample.yaw_deg;
                turn_travel += turn_angdiff(current_yaw, turn_yaw_last);
                turn_yaw_last = current_yaw;
                rel_now = turn_travel;
                err = turn_target - rel_now;          /*270/360°不能折算成最短角差*/
            }
            else
            {
                rel_now = turn_angdiff(sample.yaw_deg, yaw_start);
                err = turn_angdiff(turn_target - rel_now, 0.0f);
            }
            aerr = fabsf(err);

            /*远段使用TURN_SPEED_SCALE倍率；进入最后15°前平滑过渡到低速爬行。*/
            TurnPID.Kp = TURN_PID_KP;
            TurnPID.OutMax = TURN_OUT_MAX_RPM;
            if (aerr > TURN_FAST_DEG)
            {
                TurnPID.Kp += TURN_FAST_KP * (aerr - TURN_FAST_DEG) / aerr;
                TurnPID.OutMax = TURN_FAST_MAX_RPM;
            }
            if (aerr > TURN_CREEP_DEG)
            {
                /* 30°到15°平滑降倍率，衔接TURN_CREEP_RPM低速逼近。 */
                if (aerr < TURN_FAST_DEG)
                    turn_transition = (aerr - TURN_CREEP_DEG) /
                                      (TURN_FAST_DEG - TURN_CREEP_DEG);
                else
                    turn_transition = 1.0f;
                turn_transition = turn_transition * turn_transition *
                                  (3.0f - 2.0f * turn_transition);
                turn_scale = 1.0f + (TURN_SPEED_SCALE - 1.0f) * turn_transition;
                TurnPID.Kp *= turn_scale;
                TurnPID.Kd = TURN_PID_KD * turn_scale;
                TurnPID.OutMax *= turn_scale;
            }
            else
                TurnPID.Kd = TURN_PID_KD;
            TurnPID.OutMin = -TurnPID.OutMax;
            turn_min_rpm = TURN_CREEP_RPM +
                           (TURN_MIN_RPM * turn_scale - TURN_CREEP_RPM) * turn_transition;
            turn_speed_cap = TurnPID.OutMax;
            if (aerr < TURN_FAST_DEG)
                turn_speed_cap = TURN_CREEP_RPM +
                    (TURN_OUT_MAX_RPM * TURN_SPEED_SCALE - TURN_CREEP_RPM) * turn_transition;

            if ((xTaskGetTickCount() - posrun_start) >= pdMS_TO_TICKS(turn_timeout_ms))
            {
                run_fault = MOTOR_RUN_FAULT_TIMEOUT;
                turn_w_last = 0;
                pos_run_abort();                     /*所有转向区间均适用超时停车*/
            }
            else if (aerr <= (float)TURN_TOL_DEG)
            {
                turn_w_last = 0;
                if (turn_settle_ticks == 0)
                    Motor_Stop();                     /*只发一次急停，避免重复占用CAN*/
                if (fresh_sample && ++turn_settle_ticks >= TURN_SETTLE_TICKS)
                {
                    turn_settle_ticks = 0;
                    if (debug_route_active)
                    {
                        debug_turn_pos_commit();
                        debug_motion_done();
                    }
                    else
                    {
                        task_step_done();                  /*稳定到位: 推进步骤*/
                    }
                }
            }
            else if (aerr <= (float)TURN_CREEP_DEG)
            {
                /*低速爬行段: 方向必须与PID段使用同一个输出符号*/
                int creep_dir = ((err > 0) ? 1 : -1) * (int)TURN_OUT_SIGN;
                turn_settle_ticks = 0;
                TurnPID.Target = turn_target;
                TurnPID.Actual = turn_target - err;    /*rel_now的连续展开: 环绕跳变被err归一吸收*/
                PID_Update(&TurnPID);
                TurnPID.ErrorInt = 0;
                /*爬行速度同样过限斜率, 与PID段衔接连续无跳变*/
                if (creep_dir * TURN_CREEP_RPM > turn_w_last + TURN_SLEW_RPM)
                    turn_w_last += TURN_SLEW_RPM;
                else if (creep_dir * TURN_CREEP_RPM < turn_w_last - TURN_SLEW_RPM)
                    turn_w_last -= TURN_SLEW_RPM;
                else
                    turn_w_last = creep_dir * TURN_CREEP_RPM;
                Motor_Move(0, 0, line_rpm_round((float)turn_w_last * turn_output_ratio * TURN_OUTPUT_SPEED_RATIO));
            }
            else
            {
                {
                    int w_raw;
                    turn_settle_ticks = 0;
                    TurnPID.Target = turn_target;
                    TurnPID.Actual = turn_target - err;
                    PID_Update(&TurnPID);
                    /*软件限斜率(移植自开源工程): 每拍速度最多变化TURN_SLEW_RPM,
                      命令永不阶跃, 不激励ZDT内部斜率跟随, 消除惯性甩尾超调*/
                    w_raw = (int)(TurnPID.Out * TURN_OUT_SIGN);
                    if (w_raw > 0 && w_raw < (int)turn_min_rpm)
                        w_raw = (int)turn_min_rpm;
                    if (w_raw < 0 && w_raw > -(int)turn_min_rpm)
                        w_raw = -(int)turn_min_rpm;
                    /*过渡段也渐进降低上限，保证15°边界衔接低速逼近。*/
                    if (w_raw > (int)turn_speed_cap) w_raw = (int)turn_speed_cap;
                    if (w_raw < -(int)turn_speed_cap) w_raw = -(int)turn_speed_cap;
                    if (w_raw > turn_w_last + TURN_SLEW_RPM)      w_raw = turn_w_last + TURN_SLEW_RPM;
                    else if (w_raw < turn_w_last - TURN_SLEW_RPM) w_raw = turn_w_last - TURN_SLEW_RPM;
                    turn_w_last = w_raw;
                    Motor_Move(0, 0, line_rpm_round((float)w_raw * turn_output_ratio * TURN_OUTPUT_SPEED_RATIO));
                }
            }
        }

        /*---- 回右上角点位段(10ms 周期): 直线同款PID算法,
           目标由出发区自动换算(区一200000 / 区二1250000, 见sub=7分支) ----
           方向由误差自动决定: 启停区一(右前去程)回程误差为负=观感后退;
           启停区二(左前去程)车头朝向不同=观感直行 —— 控制上同一套闭环。
           接近目标<QR_SLOW时降为缓行速度(不扫码), 到位±TOL停车进DONE ---- */
        if (phase == PHASE_HOME)
        {
            OdomData_t o;
            odometry_get(&o);
            float err = (float)pos_target - (float)o.enc_pos[ODOM_POS_AXIS];
            float aerr = fabsf(err);
            PosRunState state = pos_run_check(o.enc_pos[ODOM_POS_AXIS], o.online[ODOM_POS_AXIS]);
            if (state == POS_PAUSED)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            if (state == POS_REACHED)
            {
                if (!pos_finish_soft(o.enc_pos[ODOM_POS_AXIS]))
                {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue;
                }
                phase = PHASE_DONE;                        /*已到右上角点位*/
                Motor_Stop();
            }
            else if (state == POS_FAILED)
            {
                pos_run_abort();
            }
            else
            {
                int vy;
                PosPID.Target = (float)pos_target;
                PosPID.Actual = (float)o.enc_pos[ODOM_POS_AXIS];
                PID_Update(&PosPID);

                vy = (int)pos_speed_limit(PosPID.Out, aerr);
                line_move(vy, yaw_hold);
            }
        }

        /*---- 直线行驶段(10ms 周期; 里程计1/vy前后通道) ----
           位置PID+30e53d1航向PID及方向补偿; 接近目标自动限速; 到位±TOL停车推进步骤 */
        if (phase == PHASE_LINE)
        {
            OdomData_t o;
            int32_t cur;
            float err, aerr, out;
            PosRunState state;
            odometry_get(&o);
            cur  = o.enc_pos[ODOM_POS_AXIS];
            err  = (float)pos_target - (float)cur;           /*与PID使用同一累计坐标*/
            aerr = fabsf(err);
            state = pos_run_check(cur, o.online[ODOM_POS_AXIS]);
            if (state == POS_PAUSED)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            if (state == POS_REACHED)
            {
                if (!pos_finish_soft(cur))
                {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue;
                }
                if (debug_route_active)
                {
                    debug_motion_done();
                }
                else
                {
                    task_step_done();                  /*到位: 推进步骤(停车在done内)*/
                }
            }
            else if (state == POS_FAILED)
            {
                pos_run_abort();
            }
            else
            {
                PosPID.Target = (float)pos_target;
                PosPID.Actual = (float)cur;
                PID_Update(&PosPID);
                out = PosPID.Out;
                out = pos_speed_limit(out, aerr);
                line_move((int)out, yaw_hold);
            }
        }

        /*---- 缓行直行+扫码段(10ms 周期; 里程计1/vy前后通道) ----
           低速逼近+二维码判停; 扫到码或到位即完成步骤(码的位置即最终停靠点) */
        if (phase == PHASE_SLOW)
        {
            OdomData_t o;
            int32_t cur;
            float err;
            PosRunState state;
            odometry_get(&o);
            cur  = o.enc_pos[ODOM_POS_AXIS];
            err  = (float)pos_target - (float)cur;
            state = pos_run_check(cur, o.online[ODOM_POS_AXIS]);
            if (state == POS_PAUSED)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            if (debug_route_active && debug_qr_scan_active)
            {
                if (state == POS_FAILED)
                {
                    debug_qr_scan_active = 0;
                    debug_no_next_start();
                }
                else if (UART8_Seq() != debug_qr_scan_seq &&
                         UART8_CamHasTarget() && UART8_Valid() && !g_link.bad)
                    debug_qr_scan_found();
                else if (state == POS_REACHED)
                {
                    debug_qr_scan_active = 0;
                    debug_no_next_start();
                }
                else
                {
                    line_move(debug_qr_scan_dir * POS_QR_SLOW_RPM, yaw_hold);
                }
            }
            else if (state == POS_FAILED)
            {
                pos_run_abort();
            }
            else if (UART8_CamHasTarget() || state == POS_REACHED)
            {
                task_step_done();                      /*扫到码/到位: 推进步骤*/
            }
            else
            {
                int8_t slow_dir = (err > 0) ? 1 : -1;

                line_move(slow_dir * POS_QR_SLOW_RPM, yaw_hold);
            }
        }


        vTaskDelay(pdMS_TO_TICKS(10));
    }
#endif
}


