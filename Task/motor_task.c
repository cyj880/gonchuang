#include "motor_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdlib.h>
#include "Emm_V5.h"
#include "route_rx.h"
#include "pid.h"
#include "odom_task.h"
#include "lunqu_imu.h"
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
  *      不能用 Delay_ms（那是 SysTick 忙等，会打断 FreeRTOS 节拍）；
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

    for (i = 0; i < MOTOR_NUM; i++)
    {
        Emm_V5_Stop_Now(Motor_Addr[i], false);
    }
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
   位置环: pos误差 -> vy(RPM); 航向环: yaw误差 -> w(RPM), BNO085接入前yaw恒0即w=0。
   到位: |pos-目标| ≤ POS_TOL_COUNTS 立即停车(爬行段低速, 停车必落容差内)。
   整定: 先 Kp(0.0005起, 太慢加大/振荡减小), 再 Ki(消除稳态误差), Kd 默认关闭
   (里程计约35ms刷新一次位置, 微分对量化噪声敏感)。 */
#define POS_TOL_COUNTS      5000     /* 到位容差(编码值): 目标±5000, 进入即停车 */
#define POS_DECEL_COUNTS    400000   /* 距目标约662mm开始渐进减速，降低末段制动负担 */
#define POS_QR_SLOW_COUNTS  12000    /* 最后约18mm进入低速平滑收尾 */
#define POS_APPROACH_RPM    12       /* 直行末段速度上限，缩短等待 */
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
#define RDK_RAW_ARM_GRAB_DEG     285     /*对位完成后RM电机预定位角度*/
#define RDK_RAW_LIFT_POS         84000u  /*对位完成后丝杆绝对目标脉冲(下行,3000RPM)*/
#define RDK_RAW_FINISH_ARM_DEG   107     /*第三块装车后RM收起角度*/
#define RDK_RAW_FINISH_LIFT_POS  50000u  /*第三块装车后丝杆收起高度，确认到位再结束/回程*/
#define RDK_RAW_BELT_POS         7040 /*原料区专用到站/收尾位置，不改变圆环站点中心*/
#define RDK_RAW_CLAW_CLOSE_DEG  109.0f   /*原料区统一夹紧角度；路线和独立1A共用*/
#define RDK_RAW_SERVO180_DEG     70.0f   /*原料区统一张开角度：到站、每块入槽及第三块收尾共用*/
#define RDK_RAW_BELT_WAIT_MS     5000u   /*同步带到位确认超时,超时后放行并照常转舵机*/
#define RING_STN_ARM_DEG         190     /*圆环站点(粗加工/暂存)到站预定位: RM电机角度*/
#define RING_STN_BELT_POS        7040 /*两轮粗加工/两轮暂存共用到站位置，独立于中心标定*/
#define RING_STN_SERVO180_DEG    48.0f   /*圆环站点到站预定位: 180°舵机角度(张开)*/
#define GRAB_SERVO_DWELL_MS      400u    /*统一夹取序列: 舵机步进后驻留时间*/
#define GRAB_REACHED_WAIT_MS     2000u   /*统一夹取序列: 等到位回传超时(超时回读兜底)*/
#define GRAB_TRIG_TIMEOUT_MS     30000u  /*等RDK触发帧超时(超时no next, 不死等)*/
#define GRAB_POS_TOL             200     /*统一夹取序列: 丝杆/同步带回读到位容差(脉冲)*/
#define BELT_POS_TOL             200     /*到站同步带回读兜底容差(脉冲)*/
#define GRAB_ARM_POS_TOL_DEG     1.0f    /*统一夹取序列: M2006到位角度容差*/
#define GRAB_ARM_MAX_WAIT_MS     3000u   /*统一夹取序列: M2006最多等待, 超时强制下一步*/

#define POS_OUT_MAX_RPM     160      /* 直线巡航位置PID输出限幅(RPM) */
#define POS_START_RPM       10       /* 更柔和的直线起步速度上限(RPM) */
#define POS_FINAL_RPM       4.0f    /*最后约12mm平滑降到4RPM，避免长时间爬行*/
#define POS_FINISH_MS       150u    /*仅低速正常到位用150ms收尾；故障/超目标仍立即停*/
#define POS_DECEL_SLEW_RPM_S 180.0f /*正常接近目标时限制减速斜率，避免轮速突降抖动；
                                        不得低于空间限速曲线所需斜率(约136RPM/s峰值)，
                                        否则减速带内压不住速度，到位仍高速→立即停车*/
#define POS_START_RAMP_MS   1200u     /* 延长起步平滑爬升，降低打滑 */
#define POS_RUN_TIMEOUT_MS  30000u   /* 位置闭环最长运行时间, 超时自动停车 */
#define POS_PROGRESS_MS     1000u   /* 持续未向目标靠近时停车(反馈不变/方向错误) */
#define POS_PROGRESS_COUNTS 1000.0f /* 每次确认进度需至少靠近约1.5mm */

#define POS_PID_KP          0.0005f  /* 误差(编码值) -> RPM */
#define POS_PID_KI          0.0003f
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
                                        换陀螺仪(轮趣<->BNO085)后若再反, 回来改这里*/
#define TURN_OUT_SIGN       (-1.0f)  /*原地转向输出方向: 实车"左转发成右转270°"已实测, 翻转。
                                        与YAW_OUT_SIGN同值(两者闭环的都是IMU增方向)*/

#define POS_LEG1_RPM        60       /* 45°斜走巡航: 出力轮转速(RPM), 右前=LF+RR 左前=RF+LR */
#define POS_LEG1_START_MS    800     /*斜走按时间缓启动，减少两轮突然出力打滑*/
#define POS_LEG1_START_RPM   8
#define POS_LEG1_APPROACH_RPM 8      /*低速到点，降低停车瞬间的轮胎打滑*/
#define POS_LEG1_APPROACH_COUNTS 40000 /*末段约61mm平滑减速*/

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
    PHASE_STATION_PLACE = 19
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
#define TURN_TOL_DEG        2.0f     /* 到位容差(deg), 进入即停车(留滞后余量) */
#define TURN_CREEP_DEG      15.0f    /* 最后15°固定低速逼近 */
#define TURN_CREEP_RPM      11       /*最后15°：原17RPM的2/3，取最近整数*/
#define TURN_SPEED_SCALE    3.0f     /*前75°：上一版4.5的2/3，降低打滑*/
#define TURN_OUT_MAX_RPM    70       /* 转向PID输出限幅(RPM) */
#define TURN_FAST_DEG       30.0f    /*此角度以内沿用已实测的减速参数*/
#define TURN_FAST_KP        0.25f    /*远段增量: (剩余角度-30°)*0.25RPM*/
#define TURN_FAST_MAX_RPM   85       /*大角度阶段的转向输出限幅*/
#define TURN_MIN_RPM        12       /* PID输出最低有效转速(RPM) */
#define TURN_SETTLE_TICKS   8        /* 到位后连续确认次数(10ms/次) */
#define TURN_TIMEOUT_MS     15000u   /* 转向最长运行时间, 超时自动停车(陀螺仪无效兜底) */
#define TURN_FULL_TIMEOUT_MS 60000u  /* 相对大角度转向(180~360°)的最长运行时间 */
#define TURN_PID_KP         0.75f    /* 误差(deg) -> RPM: 远段快速，接近目标自动减速
                                             有滞后, 增益须保守(参照直线环稳定比例) */
#define TURN_PID_KI         0.0f     /* 关闭积分，避免停下后的反复补偿 */
#define TURN_PID_KD         10.0f    /* 微分先行阻尼，抑制接近目标时的惯性超调
                                             满速25°/s时提供~5RPM反向阻尼, 压住冲过 */
#define TURN_ERR_INT_MAX    45.0f    /* 积分限幅(deg) */
#define TURN_SLEW_RPM       8        /* 软件限斜率: 每拍(10ms)速度变化上限(RPM)。
                                        命令阶跃会激励ZDT内部斜率跟随导致惯性甩尾超调
                                        (开源工程同款电机靠此招稳住90°转弯), 必加 */

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
static uint8_t      turn_settle_ticks = 0; /*到位停车稳定确认计数*/
static TickType_t   posrun_start = 0;  /*本步骤起始时刻*/
static float        yaw_hold = 0.0f;   /*本步航向保持目标: 每步启动时刻的车头朝向(相对零点±180)*/
static float        pos_initial_error = 0.0f;
static float        pos_best_error = 0.0f;
static TickType_t   pos_progress_tick = 0;
static float        pos_speed_last = 0.0f;
static TickType_t   pos_finish_tick = 0;
static uint8_t      pos_finish_active = 0;
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
static uint8_t      debug_pause_active = 0;
static uint8_t      debug_pause_next = 0;   /*1下一点对/2转向后/3操作后/4首声后/5静音后/6末声后/0结束*/
static TickType_t   debug_pause_until = 0;
static uint8_t      debug_qr_scan_active = 0;
static uint8_t      debug_qr_scan_done = 0; /*成功后跳过后续找码；新一轮第一批重置*/
static uint8_t      debug_rdk_center_active = 0;
static uint8_t      debug_rdk_standalone = 0; /*0x14独立定位：到位后恢复读取，不启动夹取/回程*/
static int8_t       debug_rdk_x_sign = RDK_CENTER_X_SIGN;
static int8_t       debug_rdk_y_sign = RDK_CENTER_Y_SIGN;
static uint8_t      belt_servo_done = 0;  /*同步带到位确认后180°舵机已转48°*/
static uint8_t      grab_step_index = 0;  /*统一夹取序列当前步*/
static uint8_t      grab_sub = 0;         /*0=发步进命令 1=等待该步完成*/
static uint8_t      grab_parallel_belt_done = 0;
static uint8_t      grab_pick_index = 0;  /*本批已完成夹取次数(0=第一块)*/
static uint8_t      grab_manual = 0;      /*1=网页/串口手动触发: 完成后直接停车*/
static uint8_t      grab_pick_fixed = 0xFF; /*!=0xFF: 固定用该块序, 不递增序号*/
static uint8_t      grab_trig_auto = 0;   /*1=完成后不等夹取通知直接回程(等RDK触发帧流程)*/
static TickType_t   grab_step_tick = 0;
static TickType_t   grab_trig_tick = 0;

/* 前两块收尾回7040/285°/84000；第三块同步带保持1400、RM107°/丝杆50000，舵机转首个待放物料原槽。 */
enum { GRAB_ST_SERVO180, GRAB_ST_SERVO270, GRAB_ST_ARM, GRAB_ST_LIFT,
       GRAB_ST_BELT, GRAB_ST_BELT_ARM, GRAB_ST_BELT_SERVO270_ARM };
typedef struct { uint8_t type; float deg; int32_t target; } GrabStep;
static const GrabStep grab_steps[] = {
    { GRAB_ST_SERVO180, RDK_RAW_CLAW_CLOSE_DEG, 0 }, /*原料区夹紧*/
    { GRAB_ST_LIFT,     0,      50000 }, /*夹住后抬起到原料区搬运高度*/
    { GRAB_ST_BELT_ARM, 107.0f, BELT_POS_GRAB_PUL },
    { GRAB_ST_LIFT,     0,      78000 },
    { GRAB_ST_SERVO180, RDK_RAW_SERVO180_DEG, 0 }, /*原料区张开*/
    { GRAB_ST_BELT_SERVO270_ARM, 285.0f, RDK_RAW_BELT_POS },
};
#define GRAB_STEP_NUM (sizeof(grab_steps) / sizeof(grab_steps[0]))

/* 270°舵机机械位置减30°：A/B/C槽=0/120/240°，每块入槽后转到下一空槽。 */
static const float grab_pick_servo270[3] = {
    SERVO270_SLOT_B_DEG, SERVO270_SLOT_C_DEG, SERVO270_SLOT_A_DEG
};
static const float grab_pick_servo270_drop[3] = {
    SERVO270_SLOT_A_DEG, SERVO270_SLOT_B_DEG, SERVO270_SLOT_C_DEG
};

/* 圆环站点放料及粗加工取回；与原料区grab_steps独立，保存物料/环号/原槽位。 */
#define PLACE_MOVE_TIMEOUT_MS    10000u
#define PLACE_QUERY_PERIOD_MS    200u
#define PLACE_SETTLE_MS          800u   /* 放料轴全部到位后，保持夹紧等待机构稳定 */
#define PLACE_AXIS_BELT          1u
#define PLACE_AXIS_LIFT          2u
#define PLACE_AXIS_ARM           4u
#define PLACE_AXES_ALL           (PLACE_AXIS_BELT | PLACE_AXIS_LIFT | PLACE_AXIS_ARM)
enum { PLACE_PICK, PLACE_CLOSE, PLACE_PICK_LIFT, PLACE_DROP_BELT, PLACE_DROP, PLACE_SETTLE, PLACE_OPEN,
       PLACE_LIFT_HOME, PLACE_BELT_HOME,
       PLACE_RETRIEVE_LIFT100, PLACE_RETRIEVE_TARGET, PLACE_RETRIEVE_CLOSE,
       PLACE_RETRIEVE_LIFT50, PLACE_RETRIEVE_BELT_ARM, PLACE_RETRIEVE_LIFT78,
       PLACE_RETRIEVE_OPEN, PLACE_RETRIEVE_LIFT_HOME, PLACE_ROUTE_RESUME, PLACE_FINISHED };
typedef struct {
    uint8_t color[3], ring[3], slot[3];
    uint8_t pick, step, started, axes, belt_done, lift_done, manual, retrieve; /* 仅粗加工取回 */
    int32_t belt_target;
    uint32_t lift_target, drop_lift_target; /* 本次放料高度：二轮暂存131000，其余227000 */
    uint32_t encoder_sequence; /* 粗加工取回后等待恢复读取后的新编码器数据 */
    TickType_t tick, query_tick;
} StationPlace;
static StationPlace station_place;
static const struct { int32_t belt; int16_t arm; } place_ring_target[3] = {
    {5410, 431}, {7128, 319}, {8782, 446}
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
static uint8_t debug_rdk_ring_encoder_paused = 0;
static uint8_t debug_rdk_ring_serial = 0; /* UART7 standalone completion returns to raw. */
static uint8_t debug_rdk_stack_mode = 0; /* 当前视觉输入为码垛单中心点mode5（路线或1D调试） */
static TickType_t debug_rdk_arm_start_tick = 0;
static TickType_t manual_arm_start_tick = 0;
static int8_t       debug_qr_scan_dir = 1;
static uint16_t     debug_qr_scan_seq = 0;
static TaskHandle_t qr_report_handle = NULL;  /*到扫码区创建/成功后删除的显示转发任务*/
#define DEBUG_QR_SCAN_COUNTS 300000  /*到站后沿原方向低速找码，最多约458mm*/
static void task_step_done(void);
static void turn_run_start(float start_yaw, float delta, uint8_t accumulate);
static uint8_t debug_leg_load(void);
static void debug_leg_start(void);
static void debug_rdk_center_return_done(void);
static void belt_pos_goto(int32_t target);

uint8_t Motor_NoNext(void)
{
    return no_next_alarm;
}

void Motor_RDK_GrabDone(void)
{
    debug_rdk_grab_done = 1;
}

typedef enum { POS_RUNNING, POS_REACHED, POS_FAILED } PosRunState;

/* 绝对帧直接使用累计目标；相对帧只在启动时加一次当前值。 */
static void pos_run_start(int32_t current)
{
    no_next_alarm = 0;
    pos_initial_error = (float)pos_target - (float)current;
    pos_best_error = fabsf(pos_initial_error);
    pos_progress_tick = posrun_start = xTaskGetTickCount();
    pos_speed_last = 0.0f;
    pos_finish_active = 0;
}

static float pos_speed_limit(float output, float error_abs)
{
    TickType_t elapsed = xTaskGetTickCount() - posrun_start;
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

    if (!online || (now - posrun_start) >= pdMS_TO_TICKS(POS_RUN_TIMEOUT_MS))
        return POS_FAILED;
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
        return POS_FAILED;
    return POS_RUNNING;
}

static void debug_route_release(void)
{
    if (debug_route_active) route_debug_release(debug_route_batch);
    if (debug_route_next_batch) route_debug_release(debug_route_next_batch);
    debug_route_next_batch = 0;
}

/* Ring recognition owns its polling pause independently of turntable return. */
static void debug_rdk_ring_stop(void)
{
    debug_rdk_ring_serial = 0;
    debug_rdk_stack_mode = 0;
    RingAlign_Stop();
    if (RDK_Ring_Active() || RDK_Stack_Active()) RDK_Center_SendIdle();
    if (debug_rdk_ring_encoder_paused)
    {
        odometry_resume_encoder_read();
        debug_rdk_ring_encoder_paused = 0;
    }
}

static void pos_run_abort(void)
{
    uint8_t resume_rdk_pos = debug_rdk_pos_active;
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
        odometry_resume_encoder_read();
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
    odometry_get(&o);
    no_next_alarm = 0;
    diag_start = o.enc_pos[ODOM_POS_AXIS];
    leg_dir = direction;
    leg1_end = distance;
    yaw_hold = heading;
    posrun_start = xTaskGetTickCount();
    PID_Init(&YawPID);
    line_yaw_reset();
    YawPID.Kp = YAW_PID_KP;  YawPID.Ki = YAW_PID_KI;  YawPID.Kd = YAW_PID_KD;
    YawPID.ErrorIntMax = YAW_ERR_INT_MAX;  YawPID.ErrorIntMin = -YAW_ERR_INT_MAX;
    YawPID.OutMax = YAW_OUT_MAX_RPM;  YawPID.OutMin = -YAW_OUT_MAX_RPM;
    phase = PHASE_DIAG;
}

static void debug_point_pause_start(uint8_t next_step, uint32_t duration_ms)
{
    Motor_Stop();
    Buzzer_On();
    debug_pause_active = 1;
    debug_pause_next = next_step;
    debug_pause_until = xTaskGetTickCount() + pdMS_TO_TICKS(duration_ms);
}

static void debug_no_next_start(void)
{
    debug_rdk_ring_stop();
    if (phase == PHASE_RDK_GRAB_TRIG || phase == PHASE_RDK_GRAB_SEQ)
        RDK_GrabArmEnable(0);
    if (phase == PHASE_RDK_ARM_EXTEND || phase == PHASE_RDK_ARM_PRE ||
        phase == PHASE_RDK_ARM_POST || phase == PHASE_RDK_GRAB_SEQ)
        Arm_M2006_Stop();
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
            odometry_resume_encoder_read();
            debug_rdk_resume_start_tick = xTaskGetTickCount();
            debug_rdk_resume_action = 2;
            phase = PHASE_RDK_RESUME;
            debug_point_pause_start(0, 5000);
            return;
        }
    }
    debug_point_pause_start(0, 5000);
}

static uint8_t grab_parallel_belt_arm_done(int32_t belt_target)
{
    TickType_t elapsed = xTaskGetTickCount() - grab_step_tick;

    if (!Arm_M2006_Ready())
    {
        Emm_V5_Stop_Now(BELT_ZDT_ADDR, false);
        debug_no_next_start();
        return 0;
    }
    if (!Arm_M2006_AtTargetWithin(GRAB_ARM_POS_TOL_DEG) &&
        elapsed >= pdMS_TO_TICKS(ARM_M2006_MOVE_TIMEOUT_MS))
    {
        Emm_V5_Stop_Now(BELT_ZDT_ADDR, false);
        debug_no_next_start();
        return 0;
    }
    if (!grab_parallel_belt_done && Belt_ZDT_ReachedGet())
        grab_parallel_belt_done = 1;
    if (!grab_parallel_belt_done &&
        elapsed >= pdMS_TO_TICKS(GRAB_REACHED_WAIT_MS))
    {
        int32_t belt_pos;
        grab_parallel_belt_done = (uint8_t)(Belt_ZDT_QueryPos(&belt_pos) &&
            abs(belt_pos - belt_target) <= GRAB_POS_TOL);
        if (!grab_parallel_belt_done)
        {
            Emm_V5_Stop_Now(BELT_ZDT_ADDR, false);
            debug_no_next_start();
            return 0;
        }
    }
    return (uint8_t)(grab_parallel_belt_done &&
                     Arm_M2006_AtTargetWithin(GRAB_ARM_POS_TOL_DEG));
}

static void debug_rdk_center_return_done(void)
{
    Motor_Stop();
    debug_rdk_pos_active = 0;
    if (debug_leg_load())
    {
        if (debug_route_leg.station) debug_leg_start();
        else debug_point_pause_start(1, 500);
    }
    else
        debug_no_next_start();
}

static void debug_qr_scan_found(void)
{
    debug_qr_scan_active = 0;
    debug_qr_scan_done = 1;
    if (qr_report_handle != NULL)
    {
        vTaskDelete(qr_report_handle);   /*扫码成功, 转发任务删除, 鸣叫后继续路线*/
        qr_report_handle = NULL;
    }
    debug_point_pause_start(4, 500);
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
        debug_point_pause_start(3, 500);
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
        odometry_resume_encoder_read();
        debug_rdk_resume_start_tick = xTaskGetTickCount();
        debug_rdk_resume_action = 0;
        phase = PHASE_RDK_RESUME;
    }
    else
    {
        /* 原料区路线流程: 对位完成先预定位 RM285°+丝杆84000(PHASE_RDK_ARM_POST),
           两者到位后再按本批块序发颜色, 并等RDK触发帧进入夹取序列。 */
        if (!Arm_M2006_SetTargetDeg(RDK_RAW_ARM_GRAB_DEG))
        {
            debug_no_next_start();
            return;
        }
        Lift_ZDT_ReachedClear();
        Emm_V5_Pos_Control(LIFT_ZDT_ADDR, LIFT_POS_DOWN_DIR,
                           LIFT_POS_SPEED_RPM, LIFT_POS_ACC,
                           RDK_RAW_LIFT_POS, true, false);
        grab_step_tick = xTaskGetTickCount();
        phase = PHASE_RDK_ARM_POST;
    }
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
        odometry_resume_encoder_read();
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
    if (odometry_encoder_sequence() == debug_rdk_pos_sequence)
    {
        if ((xTaskGetTickCount() - debug_rdk_resume_start_tick) >=
            pdMS_TO_TICKS(RDK_CENTER_RESUME_MS))
        {
            if (debug_rdk_resume_action == 3)
            {
                debug_rdk_resume_action = 2;
                debug_no_next_start();
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
    odometry_get(&o);
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
        debug_rdk_pos_sequence = odometry_pause_encoder_read(&o);
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
    {RP_QR,RP_CENTER,3,{{0,0,725000,-1},{2,0,90,-1},{0,1,620000,90}}},
    {RP_CENTER,RP_RAW,1,{{0,1,625000,180}}},
    {RP_RU,RP_RAW,1,{{0,1,625000,90}}},
    {RP_RAW,RP_ROUGH,1,{{0,1,1250000,0}}},
    {RP_ROUGH,RP_LD,1,{{0,1,-625000,270}}},
    {RP_LD,RP_STORAGE,1,{{0,1,-625000,0}}},
    {RP_STORAGE,RP_LU,1,{{0,1,-625000,0}}},
    {RP_LU,RP_RAW,1,{{0,1,-625000,90}}},
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
    uint8_t i;
    RoutePoint from, to;
    if (!route_debug_next(debug_route_batch, &debug_route_leg))
    {
        if (!debug_route_next_batch || !route_debug_finished(debug_route_batch) ||
            debug_route_leg.to_x != 400 || debug_route_leg.to_y != 1200 ||
            debug_route_heading != 0) return 0;
        route_debug_release(debug_route_batch);
        debug_route_batch = debug_route_next_batch;
        debug_route_next_batch = 0;
        grab_pick_index = 0;                     /*第二批开始, 夹取序号归零*/
        if (!route_debug_next(debug_route_batch, &debug_route_leg)) return 0;
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
        return (uint8_t)(debug_route_leg.station &&
                        debug_route_leg.action != CMD_ACT_BACK &&
                        debug_station_heading(from) == debug_goal_heading);
    return (uint8_t)(debug_rule_index < DEBUG_RULE_COUNT ||
                     debug_route_leg.action == CMD_ACT_CW ||
                     debug_route_leg.action == CMD_ACT_CCW);
}

static void debug_motion_start(void)
{
    const DebugMotion *motion;
    if (debug_rule_index >= DEBUG_RULE_COUNT)
    {
        debug_no_next_start();
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
            debug_no_next_start();
            return;
        }
        if (relative) value = debug_relative_value(value, calibrated_heading);
        debug_line_start(value, relative, (float)debug_route_heading);
        debug_qr_scan_dir = (pos_initial_error < 0) ? -1 : 1;
    }
}

static uint32_t ring_now_ms(void)
{
    return (uint32_t)((uint64_t)xTaskGetTickCount() * 1000u / configTICK_RATE_HZ);
}

static void debug_rdk_ring_start(void)
{
    debug_rdk_ring_serial = 0;
    debug_rdk_stack_mode = 0;
    Motor_Stop();
    no_next_alarm = 0;
    if (!debug_rdk_ring_encoder_paused)
    {
        odometry_pause_encoder_read(0);
        debug_rdk_ring_encoder_paused = 1;
    }
    RDK_Ring_Start();
    RingAlign_Start(ring_now_ms());
    phase = PHASE_RDK_RING_WAIT;
}

/* 模式5码垛对位：路线完成后接第二轮暂存放料；独立1D只对位停车。 */
static void debug_rdk_stack_start(uint8_t standalone)
{
    debug_rdk_ring_serial = standalone;
    debug_rdk_stack_mode = 1;
    Motor_Stop();
    no_next_alarm = 0;
    if (!debug_rdk_ring_encoder_paused)
    {
        odometry_pause_encoder_read(0);
        debug_rdk_ring_encoder_paused = 1;
    }
    RDK_Stack_Start();
    /* 恢复默认yaw校正方向: 0x15 sub=2的方向覆盖(含0=禁用)是圆环调试遗留,
       若残留0, Update第一拍就把对位打回WAIT_DIR(车永不动, LCD显SET DIR)。 */
    RingAlign_ResetYawSign();
    RingAlign_StartMode(5, 0, ring_now_ms());
    phase = PHASE_RDK_RING_WAIT;
}

/* Both already-aligned and turn-completed station paths enter here once. */
static void debug_station_work_start(void)
{
    RoutePoint point = debug_point_at(debug_route_leg.from_x, debug_route_leg.from_y);
    if (point == RP_QR)
        debug_qr_scan_start();
    else if (point == RP_RAW)
    {
        /* 只在原料区到站节点已对准90°后准备机构；普通行驶到点/途经不启动取料。 */
        Motor_Stop();
        debug_first_raw_reached = 1;
        debug_rdk_standalone = 0;
        Servo270_SetAngle(grab_pick_servo270_drop
            [grab_pick_index < 3u ? grab_pick_index : 2u]);
        belt_pos_goto(RDK_RAW_BELT_POS);
        belt_servo_done = 0; /* 同步带确认后180°舵机才转原料区张开角70°，不驻留。 */
        debug_rdk_arm_start_tick = xTaskGetTickCount();
        if (!Arm_M2006_SetTargetDeg(RDK_RAW_ARM_PRE_DEG))
        {
            debug_no_next_start();
            return;
        }
        phase = PHASE_RDK_ARM_PRE; /* RM及同步带准备完成后再开始转盘视觉。 */
    }
    else if (point == RP_ROUGH || point == RP_STORAGE)
    {
        /* 圆环站点到站预定位: RM电机190°/同步带7040/180°舵机48°, 发完即进校准,
           机构动作与视觉校准并行(校准本身不动机构)。 */
        Arm_M2006_SetTargetDeg(RING_STN_ARM_DEG);
        belt_pos_goto(RING_STN_BELT_POS);
        Servo_SetAngle(RING_STN_SERVO180_DEG);
        if (point == RP_STORAGE && debug_route_batch == 2u)
            debug_rdk_stack_start(0); /* 二轮暂存发模式5，完成后自动接131000高度放料。 */
        else
            debug_rdk_ring_start();
        /* Encoder polling paused; mean-XY, yaw, mean-XY; no saved-position return. */
    }
    else
    {
        debug_point_pause_start(3, 500);
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

    station_place.pick = 0;
    station_place.step = PLACE_PICK;
    station_place.started = 0;
    station_place.manual = manual;
    station_place.retrieve = (uint8_t)!storage;
    station_place.drop_lift_target = (storage && batch == 2u) ? 131000u : 227000u;
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
    if (!debug_route_active ||
        !((point == RP_ROUGH && (debug_route_batch == 1 || debug_route_batch == 2)) ||
          (point == RP_STORAGE && (debug_route_batch == 1 || debug_route_batch == 2))))
    {
        station_place_fail();
        return;
    }
    (void)station_place_begin(debug_route_batch, (uint8_t)(point == RP_STORAGE), 0);
}

/* 一组目标在同一拍下发，中间不等待；axes允许单独抬升/收同步带。 */
static void station_place_move(uint8_t axes, int32_t belt, int16_t arm, uint32_t lift)
{
    station_place.axes = axes;
    station_place.belt_target = belt;
    station_place.lift_target = lift;
    station_place.belt_done = station_place.lift_done = 0;
    station_place.tick = station_place.query_tick = xTaskGetTickCount();
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
    TickType_t now = xTaskGetTickCount();
    TickType_t elapsed = now - station_place.tick;
    uint8_t axes = station_place.axes;

    if (axes & PLACE_AXIS_BELT)
        station_place.belt_done |= Belt_ZDT_ReachedGet();
    if (axes & PLACE_AXIS_LIFT)
        station_place.lift_done |= Lift_ZDT_ReachedGet();
    if (elapsed >= pdMS_TO_TICKS(GRAB_REACHED_WAIT_MS) &&
        (now - station_place.query_tick) >= pdMS_TO_TICKS(PLACE_QUERY_PERIOD_MS))
    {
        int32_t pos;
        station_place.query_tick = now;
        if ((axes & PLACE_AXIS_BELT) && !station_place.belt_done &&
            Belt_ZDT_QueryPos(&pos) &&
            pos >= station_place.belt_target - GRAB_POS_TOL &&
            pos <= station_place.belt_target + GRAB_POS_TOL)
            station_place.belt_done = 1;
        /* 丝杆dir=1，回读是负坐标，不能与正幅值直接比较。 */
        if ((axes & PLACE_AXIS_LIFT) && !station_place.lift_done &&
            Lift_ZDT_QueryPos(&pos) &&
            pos >= -(int32_t)station_place.lift_target - GRAB_POS_TOL &&
            pos <= -(int32_t)station_place.lift_target + GRAB_POS_TOL)
            station_place.lift_done = 1;
    }
    if ((xTaskGetTickCount() - station_place.tick) >= pdMS_TO_TICKS(PLACE_MOVE_TIMEOUT_MS))
    {
        station_place_fail();
        return 0;
    }
    return (uint8_t)((!(axes & PLACE_AXIS_BELT) || station_place.belt_done) &&
                     (!(axes & PLACE_AXIS_LIFT) || station_place.lift_done) &&
                     (!(axes & PLACE_AXIS_ARM) ||
                      Arm_M2006_AtTargetWithin(GRAB_ARM_POS_TOL_DEG)));
}

static void station_place_run(void)
{
    uint8_t done;
    Motor_Stop();
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
        case PLACE_PICK:
            /* 仅首块执行取料定位；后续已在返回阶段全部到位，直接进入夹紧。 */
            Servo270_SetAngle(grab_pick_servo270_drop[station_place.slot[station_place.pick]]);
            station_place_move(PLACE_AXES_ALL, BELT_POS_GRAB_PUL, 107, 86000u);
            break;
        case PLACE_CLOSE: Servo_SetAngle(g_place_claw_close_deg); break;
        case PLACE_PICK_LIFT:
            station_place_move(PLACE_AXIS_LIFT, 0, 0, 60000u);
            break;
        case PLACE_DROP_BELT:
        {
            uint8_t ring = (uint8_t)(station_place.ring[station_place.pick] - 1u);
            station_place_move(PLACE_AXIS_BELT, place_ring_target[ring].belt, 0, 0);
            break;
        }
        case PLACE_DROP:
        {
            uint8_t ring = (uint8_t)(station_place.ring[station_place.pick] - 1u);
            station_place_move(PLACE_AXIS_ARM | PLACE_AXIS_LIFT, 0,
                               place_ring_target[ring].arm, station_place.drop_lift_target);
            /* 随RM和丝杆下放同拍发送下一物料槽位，不等待舵机；运动下发失败则不转槽。 */
            if (station_place.step == PLACE_DROP && station_place.pick < 2u)
                Servo270_SetAngle(grab_pick_servo270_drop[station_place.slot[station_place.pick + 1u]]);
            break;
        }
        case PLACE_SETTLE: break; /* 保持夹紧，不阻塞任务；06及故障仍逐轮检查。 */
        case PLACE_OPEN: Servo_SetAngle(g_place_claw_open_deg); break; /* 直接到完全松开角度，无缓张开。 */
        case PLACE_LIFT_HOME:
            /* 有下一块先回取料高度86000；暂存第三块仍收起到50000后停车。 */
            station_place_move(PLACE_AXIS_LIFT, 0, 0,
                               station_place.pick < 2u ? 86000u : 50000u);
            break;
        case PLACE_BELT_HOME:
            /* 同步带1400与RM107°并行返回：有下一块时到位后直接夹紧，
               第三块(暂存)收起后走PLACE_ROUTE_RESUME续下一段路线(2026-11-07)。 */
            station_place_move(PLACE_AXIS_BELT | PLACE_AXIS_ARM,
                               BELT_POS_GRAB_PUL, 107, 0);
            /* 第三块放完预转A槽(0°): 马上回原料区夹下一轮, 首个空槽先到位;
               与丝杆/带/RM同拍下发, 不驻留不等待, 交给硬件执行。 */
            if (station_place.pick == 2u)
                Servo270_SetAngle(SERVO270_SLOT_A_DEG);
            break;
        case PLACE_RETRIEVE_LIFT100:
            /* 第三块放开后先抬到取回准备高度，同步带保持在上一目标环位置。 */
            station_place_move(PLACE_AXIS_LIFT, 0, 0, 100000u);
            break;
        case PLACE_RETRIEVE_TARGET:
        {
            uint8_t ring = (uint8_t)(station_place.ring[station_place.pick] - 1u);
            /* 转到该物料原来所在的车载槽位，不等待270°舵机反馈。 */
            Servo270_SetAngle(grab_pick_servo270_drop[station_place.slot[station_place.pick]]);
            station_place_move(PLACE_AXES_ALL, place_ring_target[ring].belt,
                               place_ring_target[ring].arm, station_place.drop_lift_target);
            break;
        }
        case PLACE_RETRIEVE_CLOSE:
            Servo_SetAngle(g_place_claw_close_deg);
            break;
        case PLACE_RETRIEVE_LIFT50:
            station_place_move(PLACE_AXIS_LIFT, 0, 0, 50000u);
            break;
        case PLACE_RETRIEVE_BELT_ARM:
            station_place_move(PLACE_AXIS_BELT | PLACE_AXIS_ARM,
                               BELT_POS_GRAB_PUL, 107, 0);
            break;
        case PLACE_RETRIEVE_LIFT78:
            station_place_move(PLACE_AXIS_LIFT, 0, 0, 78000u);
            break;
        case PLACE_RETRIEVE_OPEN:
            Servo_SetAngle(g_place_claw_open_deg);
            break;
        case PLACE_RETRIEVE_LIFT_HOME:
            station_place_move(PLACE_AXIS_LIFT, 0, 0, 50000u);
            break;
        case PLACE_ROUTE_RESUME:
            station_place.encoder_sequence = odometry_encoder_sequence();
            debug_rdk_ring_stop();
            break;
        }
        return;
    }
    if (station_place.step == PLACE_ROUTE_RESUME)
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
            station_place_fail();
            return;
        }
    }
    else if (station_place.step == PLACE_SETTLE)
        done = (uint8_t)((xTaskGetTickCount() - station_place.tick) >=
                         pdMS_TO_TICKS(PLACE_SETTLE_MS));
    else if (station_place.step == PLACE_CLOSE || station_place.step == PLACE_OPEN ||
             station_place.step == PLACE_RETRIEVE_CLOSE ||
             station_place.step == PLACE_RETRIEVE_OPEN)
        done = (uint8_t)((xTaskGetTickCount() - station_place.tick) >=
                         pdMS_TO_TICKS(GRAB_SERVO_DWELL_MS));
    else
        done = station_place_move_done();
    if (!done) return;
    station_place.started = 0;
    if (station_place.step == PLACE_OPEN && station_place.pick == 2u &&
        station_place.retrieve)
    {
        /* 第三块放开后直接进入取回准备，不先回50000/1400。 */
        station_place.pick = 0;
        station_place.step = PLACE_RETRIEVE_LIFT100;
    }
    else if (station_place.step == PLACE_BELT_HOME)
    {
        if (++station_place.pick == 3u)
        {
            if (station_place.manual)
            {
                station_place.step = PLACE_FINISHED;
                phase = PHASE_DONE;
            }
            else
                /* 暂存放完直接按上位机下一段路线继续(2026-11-07)；
                   粗加工第三块不走此分支(PLACE_OPEN后已跳取回)。 */
                station_place.step = PLACE_ROUTE_RESUME;
        }
        else
            /* 丝杆86000、同步带1400、RM107°已分别确认，直接夹紧下一块。 */
            station_place.step = PLACE_CLOSE;
    }
    else if (station_place.step == PLACE_RETRIEVE_LIFT_HOME)
    {
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
            else debug_point_pause_start(1, 500);
        }
        else
            debug_no_next_start();
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
            debug_turn_start((float)debug_goal_heading);
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
                debug_point_pause_start(2, 500);
            else
                debug_no_next_start();
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
            else debug_point_pause_start(1, 500);
        }
        else
            debug_no_next_start();
    }
}

static void debug_route_start(uint8_t selection)
{
    OdomData_t o;
    uint8_t have_leg;
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
        debug_no_next_start();
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
    turn_settle_ticks = 0;
    Motor_Enable(true);
    phase = PHASE_TURN;
}

/* 同步带绝对坐标0~10000：上电位0、抓取1400、中心7050。
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

    /* 等 1s：让 main() 里的上电提示音（Delay_ms 忙等）先跑完再接管 */
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

        RDK_Center_Poll();

        if (cmd_get(&c))
        {
            switch (c.type)
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
                task_step_start(&task_z1[step_idx]);
                break;

            case CMD_DEBUG_RUN:                    /*0x13: 路线已上传后的调试一键启动*/
                cmd_tx_ack(&c);
                if (!PHASE_CAN_START(phase) || debug_route_active || debug_pause_active ||
                    !route_debug_take((uint8_t)c.param))
                    break;                         /*未完整上传/运行中: 只确认收帧, 保持停车*/
                array_mode = 0;
                leg1_next_valid = 0;
                debug_route_start((uint8_t)c.param);
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

            case CMD_LIFT_POS:                   /*0x17: 丝杆绝对位置调试(张大头FD,3000RPM)*/
                if (!PHASE_CAN_START(phase) || debug_route_active || debug_pause_active ||
                    RDK_Ring_Active())
                    break;
                Motor_Stop();                    /*调试期间底盘保持停车，ZDT闭环到位自停*/
                /* Emm_V5_Pos_Control的clk是无符号幅值，方向由dir字节单独携带
                   (0=CW向上,1=CCW向下)，绝对模式目标=±clk，不能发补码。
                   帧值=离开回零位(最高点)的下行行程，下行方向由LIFT_POS_DOWN_DIR选择。
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
                /* 新坐标: 上电位0、抓取1400、中心7050，绝对定位。 */
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

            case CMD_PLACE_SEQ:                  /*0x1C: 独立放三块，粗加工再取回；不做视觉、不消费路线*/
                if (!PHASE_CAN_START(phase) || debug_route_active || debug_pause_active ||
                    RDK_Ring_Active() || c.param < 1u || c.param > 4u)
                    break;
                if (station_place_begin((c.param == 2u || c.param == 4u) ? 2u : 1u,
                                        (uint8_t)(c.param >= 3u), 1))
                    cmd_tx_ack(&c);
                break;

            case CMD_GRAB_SEQ:                   /*0x1A: 网页手动触发统一夹取序列*/
                if (!PHASE_CAN_START(phase) || debug_route_active || debug_pause_active ||
                    RDK_Ring_Active())
                    break;
                Motor_Stop();
                grab_manual = 1;                 /*手动: 序列完成后直接停车结束*/
                grab_pick_fixed = (c.param >= 1u && c.param <= 3u) ?
                                  (uint8_t)(c.param - 1u) : 0xFFu;
                {
                    /* 2026-11-03: 任务码已注入时与路线流程一致——发颜色给RDK,
                       等触发帧AA 55 11 00 11 0D 0A再夹(30s超时no next);
                       未注入/无效时无颜色可发, 退回纯动作调试直接夹。 */
                    uint8_t pick = (grab_pick_fixed != 0xFFu) ? grab_pick_fixed :
                                   (grab_pick_index < 3u ? grab_pick_index : 2u);
                    uint8_t color = UART8_Color(debug_route_batch, pick);
                    if (UART8_Valid() && !g_link.bad && color != 0u &&
                        RDK_Center_SendGrabColor(color))
                    {
                        grab_trig_tick = xTaskGetTickCount();
                        RDK_GrabArmEnable(1);
                        phase = PHASE_RDK_GRAB_TRIG;
                    }
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
                Arm_M2006_Stop();
                Emm_V5_Stop_Now(LIFT_ZDT_ADDR, false);   /*丝杆调试运动同样可被06中止*/
                Emm_V5_Stop_Now(BELT_ZDT_ADDR, false);   /*同步带调试运动同理*/
                if (phase == PHASE_RDK_GRAB_TRIG || phase == PHASE_RDK_GRAB_SEQ)
                    RDK_GrabArmEnable(0);
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
                    odometry_resume_encoder_read();
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

        if (debug_pause_active)
        {
            if ((int32_t)(xTaskGetTickCount() - debug_pause_until) >= 0)
            {
                if (debug_pause_next != 5) Buzzer_Off();
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
                    else debug_no_next_start();
                }
                else if (debug_pause_next == 4)
                {
                    /* 扫码成功：第一声结束后静音500ms。 */
                    debug_pause_next = 5;
                    debug_pause_active = 1;
                    debug_pause_until = xTaskGetTickCount() + pdMS_TO_TICKS(500);
                }
                else if (debug_pause_next == 5)
                {
                    /* 静音结束，第二声500ms。 */
                    Buzzer_On();
                    debug_pause_next = 6;
                    debug_pause_active = 1;
                    debug_pause_until = xTaskGetTickCount() + pdMS_TO_TICKS(500);
                }
                else if (debug_pause_next == 6)
                {
                    debug_pause_next = 0;
                    if (debug_leg_load()) debug_leg_start();
                    else debug_no_next_start();
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

        if (phase == PHASE_RDK_RING_WAIT)
        {
            RDK_RingResult result;
            RingAlignMotion motion;
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
            RingAlign_Update(&result, ring_now_ms(), &motion);
            if (!debug_rdk_ring_serial && RingAlign_Done())
            {
                station_place_start();
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

        /*---- 原料区视觉靠近：不读取里程计，只按RDK实时圆心做速度闭环 ----*/
        if (phase == PHASE_RDK_ARM_POST)
        {
            /* 对位完成预定位: RM电机285°+丝杆84000, 两者到位后发颜色并等触发帧。 */
            uint8_t arm_done, lift_done;

            Motor_Stop();
            if (!Arm_M2006_Ready() ||
                (xTaskGetTickCount() - grab_step_tick) >=
                    pdMS_TO_TICKS(ARM_M2006_MOVE_TIMEOUT_MS))
            {
                Arm_M2006_Stop();
                debug_no_next_start();
            }
            else
            {
                arm_done = (uint8_t)(Arm_M2006_AtTargetWithin(GRAB_ARM_POS_TOL_DEG) ||
                                     (xTaskGetTickCount() - grab_step_tick) >=
                                         pdMS_TO_TICKS(GRAB_ARM_MAX_WAIT_MS));
                lift_done = Lift_ZDT_ReachedGet();
                if (!lift_done &&
                    (xTaskGetTickCount() - grab_step_tick) >=
                        pdMS_TO_TICKS(GRAB_REACHED_WAIT_MS))
                {
                    int32_t lift_pos;

                    /* 2s没等到9F(未设Response/丢帧): 位置回读一次兜底, 不在容差进no next。 */
                    lift_done = (uint8_t)(Lift_ZDT_QueryPos(&lift_pos) &&
                        abs(lift_pos + (int32_t)RDK_RAW_LIFT_POS) <= GRAB_POS_TOL);
                    if (!lift_done)
                        debug_no_next_start();
                }
                if (arm_done && lift_done)
                {
                    /* 预定位完成: 按本批块序发颜色, 开触发通道, 等RDK触发帧。 */
                    uint8_t pick = grab_pick_index;

                    if (pick > 2u) pick = 2u;
                    if (UART8_Valid() && !g_link.bad)
                        (void)RDK_Center_SendGrabColor(UART8_Color(debug_route_batch, pick));
                    grab_trig_tick = xTaskGetTickCount();
                    RDK_GrabArmEnable(1);
                    phase = PHASE_RDK_GRAB_TRIG;
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
            /* 原料区到站: 等夹爪伸到455°，并等同步带7040的到位回传(5s超时回读兜底)——
               确认后180°舵机转70°，不驻留；两者都完成(同步带确认超时放行)才启动
               转盘识别，避免带子带车旋转时视觉采集。 */
            Motor_Stop();
            if (!Arm_M2006_Ready() ||
                (xTaskGetTickCount() - debug_rdk_arm_start_tick) >=
                    pdMS_TO_TICKS(ARM_M2006_MOVE_TIMEOUT_MS))
            {
                Arm_M2006_Stop();
                debug_no_next_start();
            }
            else
            {
                if (!belt_servo_done)
                {
                    uint8_t confirmed = Belt_ZDT_ReachedGet();

                    if (!confirmed &&
                        (xTaskGetTickCount() - debug_rdk_arm_start_tick) >=
                            pdMS_TO_TICKS(RDK_RAW_BELT_WAIT_MS))
                    {
                        int32_t belt_pos;

                        /* 5s超时: 位置回读一次兜底, 容差内=确认; 仍失败也放行不堵路线。 */
                        confirmed = (uint8_t)(Belt_ZDT_QueryPos(&belt_pos) &&
                            abs(belt_pos - (int32_t)RDK_RAW_BELT_POS) <= BELT_POS_TOL);
                        belt_servo_done = 1;
                        Servo_SetAngle(RDK_RAW_SERVO180_DEG);
                    }
                    else if (confirmed)
                    {
                        belt_servo_done = 1;
                        Servo_SetAngle(RDK_RAW_SERVO180_DEG);
                    }
                }
                if (belt_servo_done && Arm_M2006_AtTarget())
                {
                    debug_rdk_center_start(yaw_abs360_to_signed(RDK_CENTER_RAW_YAW),
                                           RDK_CENTER_X_SIGN, RDK_CENTER_Y_SIGN);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (phase == PHASE_RDK_GRAB_TRIG)
        {
            /* 等RDK触发帧 AA 55 11 00 11 0D 0A(恒定7字节): 目标颜色物体静止时
               RDK经USART3持续发送；每块单独开接收，收到首个完整有效帧即进入夹取序列。 */
            Motor_Stop();
            if (RDK_GrabTakeRequest())
            {
                RDK_GrabArmEnable(0);          /*收到即关, 不跨块复用*/
                if (!grab_manual)
                {
                    grab_pick_fixed = 0xFFu;   /*路线: 块序按grab_pick_index*/
                    grab_trig_auto = 1;        /*完成后不等夹取通知, 直接回程*/
                }
                /* grab_manual=1(0x1A手动): 保持手动语义, 连续块间仍等RDK,
                   夹完本批停车结束。 */
                grab_step_index = 0;
                grab_sub = 0;
                phase = PHASE_RDK_GRAB_SEQ;
            }
            else if ((xTaskGetTickCount() - grab_trig_tick) >=
                     pdMS_TO_TICKS(GRAB_TRIG_TIMEOUT_MS))
            {
                /* 超时保护: RDK未发触发帧(未识别到静止物料/链路故障), 不死等。 */
                RDK_GrabArmEnable(0);
                debug_no_next_start();
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (phase == PHASE_RDK_GRAB_SEQ)
        {
            const GrabStep *st;
            uint8_t pk;

            Motor_Stop();
            if (grab_step_index >= GRAB_STEP_NUM)
            {
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
                            uint8_t color = UART8_Color(debug_route_batch, pick);
                            if (UART8_Valid() && !g_link.bad && color != 0u &&
                                RDK_Center_SendGrabColor(color))
                            {
                                grab_trig_tick = xTaskGetTickCount();
                                RDK_GrabArmEnable(1);
                                phase = PHASE_RDK_GRAB_TRIG;
                            }
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
                        (void)RDK_Center_SendGrabColor(
                            UART8_Color(debug_route_batch, grab_pick_index));
                        grab_trig_tick = xTaskGetTickCount();
                        RDK_GrabArmEnable(1);
                        phase = PHASE_RDK_GRAB_TRIG;
                    }
                    else if (grab_trig_auto)
                    {
                        /* 第三块(RDK触发流程): 完成后不等夹取通知, 直接回程走下一段。 */
                        odometry_resume_encoder_read();
                        debug_rdk_resume_start_tick = xTaskGetTickCount();
                        debug_rdk_resume_action = 3;
                        phase = PHASE_RDK_RESUME;
                    }
                    else
                        phase = PHASE_RDK_GRAB_WAIT;   /*RDK触发: 等待夹取完成通知*/
                }
            }
            else
            {
                uint8_t step_done = 0;

                pk = (grab_pick_fixed != 0xFFu) ? grab_pick_fixed :
                     (grab_pick_index < 3u ? grab_pick_index : 2u);
                st = &grab_steps[grab_step_index];
                if (grab_sub == 0)
                {
                    grab_step_tick = xTaskGetTickCount();
                    grab_sub = 1;
                    switch (st->type)
                    {
                    case GRAB_ST_SERVO180: Servo_SetAngle(st->deg); break;
                    case GRAB_ST_SERVO270:
                    {
                        /* 末步270°舵机取角: 固定块序或按本块序号, 120/240/0。 */
                        Servo270_SetAngle(grab_pick_servo270[pk]);
                        break;
                    }
                    case GRAB_ST_ARM:
                        if (!Arm_M2006_SetTargetDeg((int16_t)st->deg))
                            debug_no_next_start();
                        break;
                    case GRAB_ST_BELT_ARM:
                        grab_parallel_belt_done = 0;
                        if (!Arm_M2006_SetTargetDeg((int16_t)st->deg))
                            debug_no_next_start();
                        else
                            belt_pos_goto(st->target);
                        break;
                    case GRAB_ST_BELT_SERVO270_ARM:
                    {
                        uint8_t last = (uint8_t)(pk == 2u);
                        grab_parallel_belt_done = 0;
                        if (!Arm_M2006_SetTargetDeg(last ? RDK_RAW_FINISH_ARM_DEG : (int16_t)st->deg))
                            debug_no_next_start();
                        else
                        {
                            /* 第三块(收尾)同步带保持1400取料位不回7040(2026-11-07)；
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
                            /* 丝杆第三块回50000，前两块仍回预定位84000；回读目标=-clk。 */
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
                        debug_no_next_start();
                    else if (Arm_M2006_AtTargetWithin(GRAB_ARM_POS_TOL_DEG) ||
                             (xTaskGetTickCount() - grab_step_tick) >=
                                 pdMS_TO_TICKS(GRAB_ARM_MAX_WAIT_MS))
                        step_done = 1;
                }
                else if (st->type == GRAB_ST_BELT_ARM)
                {
                    step_done = grab_parallel_belt_arm_done(st->target);
                }
                else if (st->type == GRAB_ST_BELT_SERVO270_ARM)
                {
                    uint32_t lift_target = (pk == 2u) ? RDK_RAW_FINISH_LIFT_POS : RDK_RAW_LIFT_POS;
                    /* 舵机只发目标，不等到位；此阶段等同步带、RM电机和丝杆。
                       同步带回读兜底目标与执行一致: 第三块1400, 前两块7040。 */
                    step_done = grab_parallel_belt_arm_done(
                        (pk == 2u) ? BELT_POS_GRAB_PUL : st->target);
                    if (step_done && !Lift_ZDT_ReachedGet())
                    {
                        if ((xTaskGetTickCount() - grab_step_tick) >=
                            pdMS_TO_TICKS(GRAB_REACHED_WAIT_MS))
                        {
                            /* 2s没等到丝杆9F: 按本块实际目标回读一次兜底，坐标为负。 */
                            int32_t lift_pos;
                            step_done = (uint8_t)(Lift_ZDT_QueryPos(&lift_pos) &&
                                abs(lift_pos + (int32_t)lift_target) <= GRAB_POS_TOL);
                            if (!step_done)
                                debug_no_next_start();
                        }
                        else
                            step_done = 0;  /*丝杆比RM慢属正常, 下拍再查*/
                    }
                }
                else /* GRAB_ST_LIFT / GRAB_ST_BELT: 等到位回传, 超时回读兜底 */
                {
                    if ((st->type == GRAB_ST_LIFT ?
                         Lift_ZDT_ReachedGet() : Belt_ZDT_ReachedGet()))
                        step_done = 1;
                    else if ((xTaskGetTickCount() - grab_step_tick) >=
                             pdMS_TO_TICKS(GRAB_REACHED_WAIT_MS))
                    {
                        /* 2s没等到9F(未设Response/丢帧): 位置回读一次兜底。 */
                        int32_t pos;
                        uint8_t ok = (st->type == GRAB_ST_LIFT ?
                                      Lift_ZDT_QueryPos(&pos) : Belt_ZDT_QueryPos(&pos));
                        if (ok)
                        {
                            /* 回读为带符号坐标: 丝杆dir=1目标=-target(负),
                               同步带目标=st->target(抓取1400/回中心7050)。 */
                            int32_t tgt = (st->type == GRAB_ST_LIFT) ?
                                      -(int32_t)st->target : st->target;
                            if (pos - tgt <= GRAB_POS_TOL && tgt - pos <= GRAB_POS_TOL)
                                step_done = 1;
                        }
                        if (!step_done)
                            debug_no_next_start();
                    }
                }
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
            Motor_Stop();
            if (debug_rdk_grab_done)
            {
                debug_rdk_grab_done = 0;
                odometry_resume_encoder_read();
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
                float start_ratio = (float)(xTaskGetTickCount() - posrun_start) /
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
                pos_run_abort();
            }
        }

        /*---- 原地转向闭环(10ms周期; 直接读BNO085中断数据, 滞后<=10ms) ----*/
        if (phase == PHASE_TURN)
        {
            float rel_now, err, aerr;
            float turn_scale = 1.0f;
            float turn_transition = 0.0f;
            float turn_min_rpm;
            float turn_speed_cap;

            if (turn_accumulate)
            {
                float current_yaw = IMU_GetYaw();
                turn_travel += turn_angdiff(current_yaw, turn_yaw_last);
                turn_yaw_last = current_yaw;
                rel_now = turn_travel;
                err = turn_target - rel_now;          /*270/360°不能折算成最短角差*/
            }
            else
            {
                rel_now = turn_angdiff(IMU_GetYaw(), yaw_start);
                err = turn_angdiff(turn_target - rel_now, 0.0f);
            }
            aerr = fabsf(err);

            /*前75°使用3倍倍率；进入最后15°前从该倍率平滑过渡到低速爬行。*/
            TurnPID.Kp = TURN_PID_KP;
            TurnPID.OutMax = TURN_OUT_MAX_RPM;
            if (aerr > TURN_FAST_DEG)
            {
                TurnPID.Kp += TURN_FAST_KP * (aerr - TURN_FAST_DEG) / aerr;
                TurnPID.OutMax = TURN_FAST_MAX_RPM;
            }
            if (aerr > TURN_CREEP_DEG)
            {
                /* 30°到15°平滑降倍率，避免从巡航直接切到11RPM。 */
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
                turn_w_last = 0;
                pos_run_abort();                     /*所有转向区间均适用超时停车*/
            }
            else if (aerr <= (float)TURN_TOL_DEG)
            {
                turn_w_last = 0;
                if (turn_settle_ticks == 0)
                    Motor_Stop();                     /*只发一次急停，避免重复占用CAN*/
                if (++turn_settle_ticks >= TURN_SETTLE_TICKS)
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
                Motor_Move(0, 0, turn_w_last);
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
                    /*过渡段也渐进降低上限，保证15°边界的目标输出接近11RPM。*/
                    if (w_raw > (int)turn_speed_cap) w_raw = (int)turn_speed_cap;
                    if (w_raw < -(int)turn_speed_cap) w_raw = -(int)turn_speed_cap;
                    if (w_raw > turn_w_last + TURN_SLEW_RPM)      w_raw = turn_w_last + TURN_SLEW_RPM;
                    else if (w_raw < turn_w_last - TURN_SLEW_RPM) w_raw = turn_w_last - TURN_SLEW_RPM;
                    turn_w_last = w_raw;
                    Motor_Move(0, 0, w_raw);
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


