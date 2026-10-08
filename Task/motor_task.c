#include "motor_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include "Emm_V5.h"
#include "route_rx.h"
#include "pid.h"
#include "odom_task.h"
#include "lunqu_imu.h"
#include "uart8.h"
#include "Buzzer.h"

/**
  * 四路 Emm_V5 步进闭环电机控制任务（CAN2，500Kbps）
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
#define POS_QR_SLOW_COUNTS  60000    /* 接近目标约91mm时限制为缓行速度 */
#define POS_QR_SLOW_RPM     10       /* 缓行速度(RPM), 减弱接近目标的速度衰减 */

#define POS_OUT_MAX_RPM     100      /* 位置PID输出限幅(RPM) */
#define POS_START_RPM       30       /* 起步速度上限(RPM) */
#define POS_START_RAMP_MS   300u     /* 起步上限平滑增加至POS_OUT_MAX_RPM */
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
#define YAW_ERR_INT_MAX     30.0f    /* 积分限幅(deg) */
#define YAW_OUT_SIGN        (-1.0f)  /*航向纠正方向: 实车45°斜走实测越纠越偏, 已翻转。
                                        适用于直线/缓行/校准/45°的yaw保持(共用YawPID);
                                        换陀螺仪(轮趣<->BNO085)后若再反, 回来改这里*/
#define TURN_OUT_SIGN       (-1.0f)  /*原地转向输出方向: 实车"左转发成右转270°"已实测, 翻转。
                                        与YAW_OUT_SIGN同值(两者闭环的都是IMU增方向)*/

#define POS_LEG1_RPM        40       /* 45°斜走段: 出力轮转速(RPM), 右前=LF+RR 左前=RF+LR */

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
    PHASE_HOME = 7
} TaskPhase;

/* 新任务可启动判定: 空闲 或 已完成停车。
   单帧=单步指令: 车停着(DONE)就必须立即执行下一帧(实车bug: 停车后发帧只回ACK不走车);
   07整体链同理(链的前置判断在链内部: 各段自动衔接, 无需外部状态)。 */
#define PHASE_CAN_START(p)  ((p) == PHASE_IDLE || (p) == PHASE_DONE)

/* ---- 任务步骤数组(上位机04动作帧驱动, 每改变一个方向=一步) ----
   type: 1=45°斜走(dir=地图方向码05右前/04左前/07右后/06左后)
         2=直线到pos(axis选反馈轮, target正负=计数方向)
         3=缓行+扫到码即停  5=原地转90°(dir=3顺/4逆)
   axis: 0=纵向(Y轴闭环: 里程计1反馈+vy前后通道)  1=横向(X轴闭环: 里程计2反馈+vx横移通道)
   区一坐标系: X总长1450000, 右上角100000, 扫码区650000 —— 数值实测后直接改本表 */
#define STEPT_DIAG  1
#define STEPT_LINE  2
#define STEPT_SLOW  3
#define STEPT_TURN  5

typedef struct
{
    uint8_t type;      /*步骤类型*/
    uint8_t dir;       /*斜走方向码 / 转弯方向(3顺4逆)*/
    uint8_t axis;      /*闭环反馈轴: 0=纵向(里程计1, 默认) 1=横向(里程计2, 预留)*/
    int32_t target;    /*相对步长(编码值): 正=沿当前车头前进, 负=后退; 45°斜走=斜向距离*/
} TaskStep;

/* 图二"第一批"动作序列(黄色物料) —— target=每步相对步长(正前进/负后退), 全部走里程计1 */
static const TaskStep task_z1[] = {
    {STEPT_DIAG, 5, 1,   100000},   /*45度斜走: 纵向计到100000(启停区1-右上角45度, 方向码实测调)*/
    {STEPT_LINE, 0, 0,   650000},   /*切换直线: 继续纵向到650000(扫码区)*/
    {STEPT_LINE, 0, 0,  -550000},   /*后退550000: 扫码区-右上角(650000-100000)*/
    {STEPT_TURN, 3, 0,         0},  /*顺时针转90度(车头变为朝场地左)*/
    {STEPT_LINE, 0, 0,   634000},   /*前进(估算1050mm): 到原料区横向位置——实测改*/
    {STEPT_TURN, 4, 0,         0},  /*逆时针转90度(车头回朝场地下方)*/
    {STEPT_LINE, 0, 0,  1253000},   /*前进(估算2075mm): 到粗加工区——实测改*/
    {STEPT_TURN, 3, 0,         0},  /*顺时针转90度(车头朝场地左)*/
    {STEPT_LINE, 0, 0,   300000},   /*前进(估算500mm): 到左下角/暂存区横向位——实测改*/
    {STEPT_TURN, 3, 0,         0},  /*顺时针转90度*/
    {STEPT_LINE, 0, 0,   544000},   /*前进(估算900mm): 到暂存区纵向位——实测改*/
};
#define TASK_Z1_N  (sizeof(task_z1) / sizeof(task_z1[0]))

/* ---- 各步骤步长=两测量点间距(mm)x604.17, 实测后直接改上表数值 ---- */

/* ---- 原地转向闭环(05帧 sub=2左转/sub=3右转, param=角度deg) ----
   反馈: IMU_GetYaw()直读当前陀螺仪最新值(中断刷新, 滞后<=10ms)。
   若经 OdomTask(35ms RS485轮询节奏)透传, 高速段每盲区多转2~3°, 会来回振荡!
   以触发时刻航向为原点计算相对角, 角差全程归一, 无±180°环绕跳变。 */
#define TURN_TOL_DEG        2.0f     /* 到位容差(deg), 进入即停车(留滞后余量) */
#define TURN_CREEP_DEG      15.0f    /* 爬行带(deg): 带内固定低速逼近 */
#define TURN_CREEP_RPM      12       /* 爬行转速(RPM), 高于电机低速死区 */
#define TURN_OUT_MAX_RPM    70       /* 转向PID输出限幅(RPM) */
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
static PID_t PosXPID;   /* 位置环: 里程计2(X横向) -> vx(RPM) */
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

static float yaw_target_near(float target, float actual)
{
    return actual + turn_angdiff(target, actual);
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
static int32_t      diag_start = 0;    /*斜走步起点计数(相对步长基准)*/
static uint8_t      line_axis = 0;     /*直线段反馈轴: 0=里程计1(vy) 1=里程计2(vx)*/
static uint8_t      slow_axis = 0;     /*缓行段反馈轴*/
static uint8_t      debug_route_active = 0; /*0x13调试路线运行中*/
static uint8_t      debug_route_step = 0;   /*1=到右上角45度, 2=到扫码区*/
static uint8_t      debug_route_branch = 0; /*0=未知停止, 1=回右上角, 2=去中心点复合*/
static volatile uint8_t no_next_alarm = 0;  /*MotorTask写，LCD_Task读*/
static uint8_t      debug_pause_active = 0;
static uint8_t      debug_pause_next = 0;   /*2=扫码, 3=分支直线, 0=结束*/
static TickType_t   debug_pause_until = 0;
static void task_step_done(void);
static void turn_run_start(float start_yaw, float delta, uint8_t accumulate);

uint8_t Motor_NoNext(void)
{
    return no_next_alarm;
}

typedef enum { POS_RUNNING, POS_REACHED, POS_FAILED } PosRunState;

/* 绝对帧直接使用累计目标；相对帧只在启动时加一次当前值。 */
static void pos_run_start(int32_t current)
{
    no_next_alarm = 0;
    pos_initial_error = (float)pos_target - (float)current;
    pos_best_error = fabsf(pos_initial_error);
    pos_progress_tick = posrun_start = xTaskGetTickCount();
}

static float pos_speed_limit(float output, float error_abs)
{
    TickType_t elapsed = xTaskGetTickCount() - posrun_start;
    TickType_t ramp_ticks = pdMS_TO_TICKS(POS_START_RAMP_MS);
    float limit = POS_OUT_MAX_RPM;

    if (elapsed < ramp_ticks)
        limit = POS_START_RPM + (POS_OUT_MAX_RPM - POS_START_RPM) *
                (float)elapsed / (float)ramp_ticks;
    if (error_abs <= POS_QR_SLOW_COUNTS && limit > POS_QR_SLOW_RPM)
        limit = POS_QR_SLOW_RPM;
    if (output > limit) return limit;
    if (output < -limit) return -limit;
    return output;
}

static PosRunState pos_run_check(int32_t current, uint8_t online)
{
    TickType_t now = xTaskGetTickCount();
    float err = (float)pos_target - (float)current;
    float aerr = fabsf(err);

    if (!online || (now - posrun_start) >= pdMS_TO_TICKS(POS_RUN_TIMEOUT_MS))
        return POS_FAILED;
    /* 低速到位或跨过目标即停，避免采样跳过容差带后反复追赶。 */
    if (aerr <= POS_TOL_COUNTS ||
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

static void pos_run_abort(void)
{
    array_mode = 0;
    debug_route_active = 0;
    debug_route_step = 0;
    debug_route_branch = 0;
    if (debug_pause_active) Buzzer_Off();
    debug_pause_active = 0;
    debug_pause_next = 0;
    leg1_next_valid = 0;
    phase = PHASE_IDLE;
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
    no_next_alarm = 1;
    debug_point_pause_start(0, 5000);
}

static void debug_line_start(int32_t value, uint8_t relative, float heading)
{
    OdomData_t o;
    int32_t current;

    odometry_get(&o);
    line_axis = ODOM_POS_AXIS;
    current = o.enc_pos[line_axis];
    pos_target = relative ? current + value : value;
    pos_run_start(current);
    yaw_hold = yaw_abs360_to_signed(heading);
    PID_Init(&PosPID);
    PosPID.Kp = POS_PID_KP;  PosPID.Ki = POS_PID_KI;  PosPID.Kd = POS_PID_KD;
    PosPID.ErrorIntMax = POS_ERR_INT_MAX;  PosPID.ErrorIntMin = -POS_ERR_INT_MAX;
    PosPID.OutMax = POS_OUT_MAX_RPM;  PosPID.OutMin = -POS_OUT_MAX_RPM;
    PID_Init(&YawPID);
    YawPID.Kp = YAW_PID_KP;  YawPID.Ki = YAW_PID_KI;  YawPID.Kd = YAW_PID_KD;
    YawPID.ErrorIntMax = YAW_ERR_INT_MAX;  YawPID.ErrorIntMin = -YAW_ERR_INT_MAX;
    YawPID.OutMax = YAW_OUT_MAX_RPM;  YawPID.OutMin = -YAW_OUT_MAX_RPM;
    phase = PHASE_LINE;
}

static void debug_turn_start(float target_yaw)
{
    float now_yaw = IMU_GetYaw();
    turn_run_start(now_yaw, turn_angdiff(yaw_abs360_to_signed(target_yaw), now_yaw), 0);
}

static void debug_route_start(uint8_t branch)
{
    debug_route_active = 1;
    debug_route_step = 1;
    debug_route_branch = branch;
    diag_run_start(1, 100000, yaw_abs360_to_signed(0.0f)); /*启停区1→右上角45度*/
    Motor_Enable(true);
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

    case STEPT_LINE:                        /*直线PID(axis选反馈轮/输出通道), target=相对步长*/
    {
        OdomData_t o0;
        odometry_get(&o0);
        line_axis  = st->axis;
        pos_target = ((line_axis == 0) ? o0.enc_pos[0] : o0.enc_pos[1]) + st->target;
        pos_run_start(o0.enc_pos[line_axis]);
        yaw_hold    = IMU_GetYaw();        /*本步保持朝向=启动时刻车头*/
        PID_Init(&PosPID);
        PosPID.Kp = POS_PID_KP;  PosPID.Ki = POS_PID_KI;  PosPID.Kd = POS_PID_KD;
        PosPID.ErrorIntMax =  POS_ERR_INT_MAX;  PosPID.ErrorIntMin = -POS_ERR_INT_MAX;
        PosPID.OutMax =  POS_OUT_MAX_RPM;       PosPID.OutMin = -POS_OUT_MAX_RPM;
        PID_Init(&PosXPID);
        PosXPID.Kp = POS_PID_KP;  PosXPID.Ki = POS_PID_KI;  PosXPID.Kd = POS_PID_KD;
        PosXPID.ErrorIntMax =  POS_ERR_INT_MAX;  PosXPID.ErrorIntMin = -POS_ERR_INT_MAX;
        PosXPID.OutMax =  POS_OUT_MAX_RPM;      PosXPID.OutMin = -POS_OUT_MAX_RPM;
        PID_Init(&YawPID);
        YawPID.Kp = YAW_PID_KP;  YawPID.Ki = YAW_PID_KI;  YawPID.Kd = YAW_PID_KD;
        YawPID.ErrorIntMax =  YAW_ERR_INT_MAX;  YawPID.ErrorIntMin = -YAW_ERR_INT_MAX;
        YawPID.OutMax =  YAW_OUT_MAX_RPM;       YawPID.OutMin = -YAW_OUT_MAX_RPM;
        phase = PHASE_LINE;
        break;
    }

    case STEPT_SLOW:                        /*缓行+扫码(axis选反馈轮), target=相对步长*/
    {
        OdomData_t o0;
        odometry_get(&o0);
        slow_axis  = st->axis;
        pos_target = o0.enc_pos[slow_axis] + st->target;
        pos_run_start(o0.enc_pos[slow_axis]);
        yaw_hold    = IMU_GetYaw();        /*本步保持朝向=启动时刻车头*/
        PID_Init(&YawPID);
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
    YawPID.Kp = YAW_PID_KP;  YawPID.Ki = YAW_PID_KI;  YawPID.Kd = YAW_PID_KD;
    YawPID.ErrorIntMax =  YAW_ERR_INT_MAX;  YawPID.ErrorIntMin = -YAW_ERR_INT_MAX;
    YawPID.OutMax =  YAW_OUT_MAX_RPM;       YawPID.OutMin = -YAW_OUT_MAX_RPM;

    for (;;)
    {
        ZbeeCmd c;

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
                if (debug_pause_active) break;
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
                    slow_axis = ODOM_POS_AXIS;
                    pos_target   = (int32_t)c.param;
                    pos_run_start(o.enc_pos[slow_axis]);
                    yaw_hold = IMU_GetYaw();
                    PID_Init(&YawPID);                 /*缓行段yaw保持用*/
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
                    line_axis = ODOM_POS_AXIS;
                    pos_target   = (int32_t)c.param;
                    pos_run_start(o.enc_pos[line_axis]);
                    yaw_hold = IMU_GetYaw();
                    PID_Init(&PosPID);                         /* 清运行状态 */
                    PosPID.Kp = POS_PID_KP;  PosPID.Ki = POS_PID_KI;  PosPID.Kd = POS_PID_KD;
                    PosPID.ErrorIntMax =  POS_ERR_INT_MAX;  PosPID.ErrorIntMin = -POS_ERR_INT_MAX;
                    PosPID.OutMax =  POS_OUT_MAX_RPM;       PosPID.OutMin = -POS_OUT_MAX_RPM;
                    PID_Init(&YawPID);
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
                if (c.sub == 1)
                {
                    array_mode = 0;
                    leg1_next_valid = 0;
                    diag_run_start(1, (int32_t)c.param, (c.flags & 0x02) ?
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
                    line_axis = ODOM_POS_AXIS;
                    current = o.enc_pos[line_axis];
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
                if (c.param != 1 || !PHASE_CAN_START(phase) || !route_debug_take(1))
                    break;                         /*未完整上传/非批次1: 只确认收帧, 保持停车*/
                array_mode = 0;
                leg1_next_valid = 0;
                debug_route_start(route_debug_branch(1));
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
                array_mode = 0;
                no_next_alarm = 0;
                debug_route_active = 0;
                debug_route_step = 0;
                debug_route_branch = 0;
                if (debug_pause_active) Buzzer_Off();
                debug_pause_active = 0;
                debug_pause_next = 0;
                phase = PHASE_IDLE;
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
                Buzzer_Off();
                debug_pause_active = 0;
                if (debug_pause_next == 2)
                {
                    debug_pause_next = 0;
                    debug_route_step = 2;
                    debug_line_start(520000, 1, 0.0f);       /*右上角45度→扫码区*/
                }
                else if (debug_pause_next == 3)
                {
                    debug_pause_next = 0;
                    debug_route_step = 3;
                    if (debug_route_branch == 1)
                        debug_line_start(100000, 0, 0.0f);   /*扫码区→右上角*/
                    else
                        debug_line_start(720000, 0, 0.0f);   /*扫码区→中心复合流程校准位置*/
                }
                else
                {
                    debug_pause_next = 0;
                    debug_route_active = 0;
                    debug_route_step = 0;
                    debug_route_branch = 0;
                    phase = PHASE_DONE;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /*---- 45°斜走段(匀速+航向保持; 本段纵向行程到位 -> 停车或衔接直线) ----
           移动方向由 vx:vy=1:1 轮速比决定, 车头朝向由 yaw 闭环独立锁住 —— 麦轮两通道解耦 */
        if (phase == PHASE_DIAG)
        {
            OdomData_t o;
            odometry_get(&o);

            YawPID.Actual = IMU_GetYaw();                     /*直读最新yaw(不经35ms透传)*/
            YawPID.Target = yaw_target_near(yaw_hold, YawPID.Actual);
            PID_Update(&YawPID);

            if (leg_dir == 1)                       /*右前45°: LF+RR出力*/
                Motor_Move(-POS_LEG1_RPM / 2, POS_LEG1_RPM / 2, (int)(YawPID.Out * YAW_OUT_SIGN));
            else if (leg_dir == 2)                  /*左前45°: RF+LR出力*/
                Motor_Move(POS_LEG1_RPM / 2, POS_LEG1_RPM / 2, (int)(YawPID.Out * YAW_OUT_SIGN));
            else if (leg_dir == 3)                  /*右后45°: RF+LR反转*/
                Motor_Move(-POS_LEG1_RPM / 2, -POS_LEG1_RPM / 2, (int)(YawPID.Out * YAW_OUT_SIGN));
            else                                    /*左后45°: LF+RR反转*/
                Motor_Move(POS_LEG1_RPM / 2, -POS_LEG1_RPM / 2, (int)(YawPID.Out * YAW_OUT_SIGN));

            if (fabsf((float)o.enc_pos[ODOM_POS_AXIS] - (float)diag_start) >= fabsf((float)leg1_end))
            {
                if (!array_mode && leg1_next_valid)
                {
                    /*07旧链: 无缝衔接直线段(里程计累计不清, 目标绝对值直接生效)*/
                    pos_target   = leg1_next;
                    line_axis = ODOM_POS_AXIS;
                    pos_run_start(o.enc_pos[line_axis]);
                    leg1_next_valid = 0;
                    PID_Init(&PosPID);
                    PosPID.Kp = POS_PID_KP;  PosPID.Ki = POS_PID_KI;  PosPID.Kd = POS_PID_KD;
                    PosPID.ErrorIntMax =  POS_ERR_INT_MAX;  PosPID.ErrorIntMin = -POS_ERR_INT_MAX;
                    PosPID.OutMax =  POS_OUT_MAX_RPM;       PosPID.OutMin = -POS_OUT_MAX_RPM;
                    PID_Init(&YawPID);
                    YawPID.Kp = YAW_PID_KP;  YawPID.Ki = YAW_PID_KI;  YawPID.Kd = YAW_PID_KD;
                    YawPID.ErrorIntMax =  YAW_ERR_INT_MAX;  YawPID.ErrorIntMin = -YAW_ERR_INT_MAX;
                    YawPID.OutMax =  YAW_OUT_MAX_RPM;       YawPID.OutMin = -YAW_OUT_MAX_RPM;
                    phase = PHASE_LINE;                /*无缝衔接: 进入直线行驶段*/
                }
                else
                {
                    if (debug_route_active && debug_route_step == 1)
                    {
                        debug_point_pause_start(2, 500);       /*右上角45度*/
                    }
                    else
                    {
                        task_step_done();              /*段末: 推进步骤(一键自动下一步)*/
                    }
                }
            }
            else if ((xTaskGetTickCount() - posrun_start) > pdMS_TO_TICKS(POS_RUN_TIMEOUT_MS))
            {
                debug_route_active = 0;
                debug_route_step = 0;
                phase = PHASE_IDLE;                              /*超时保护*/
                leg1_next_valid = 0;
                Motor_Stop();
            }
        }

        /*---- 原地转向闭环(10ms周期; 直接读BNO085中断数据, 滞后<=10ms) ----*/
        if (phase == PHASE_TURN)
        {
            float rel_now, err, aerr;

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
                    if (debug_route_active && debug_route_step == 4)
                    {
                        Motor_Stop();
                        debug_route_step = 5;
                        debug_line_start(600000, 1, 90.0f);    /*中心复合流程: yaw90下相对前进*/
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
                    if (w_raw > 0 && w_raw < TURN_MIN_RPM) w_raw = TURN_MIN_RPM;
                    if (w_raw < 0 && w_raw > -TURN_MIN_RPM) w_raw = -TURN_MIN_RPM;
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

                YawPID.Actual = IMU_GetYaw();
                YawPID.Target = yaw_target_near(yaw_hold, YawPID.Actual);
                PID_Update(&YawPID);

                vy = (int)pos_speed_limit(PosPID.Out, aerr);
                Motor_Move(0, vy, (int)(YawPID.Out * YAW_OUT_SIGN));
            }
        }

        /*---- 直线行驶段(10ms 周期; axis=0里程计1/vy  1=里程计2/vx) ----
           纯PID直行+航向保持; 接近目标自动限速; 到位±TOL停车推进步骤 */
        if (phase == PHASE_LINE)
        {
            OdomData_t o;
            int32_t cur;
            float err, aerr, out;
            PosRunState state;
            odometry_get(&o);
            cur  = (line_axis == 0) ? o.enc_pos[0] : o.enc_pos[1];
            err  = (float)pos_target - (float)cur;           /*与PID使用同一累计坐标*/
            aerr = fabsf(err);
            state = pos_run_check(cur, o.online[line_axis]);

            if (state == POS_REACHED)
            {
                if (debug_route_active && debug_route_step == 2)
                {
                    if (debug_route_branch == 0)
                    {
                        debug_no_next_start();
                    }
                    else
                        debug_point_pause_start(3, 500);       /*已知分支: 到扫码区短暂停留*/
                }
                else if (debug_route_active && debug_route_step == 3)
                {
                    if (debug_route_branch == 2)
                    {
                        Motor_Stop();
                        debug_route_step = 4;
                        debug_turn_start(90.0f);               /*中心复合流程: 绝对yaw90*/
                    }
                    else
                    {
                        debug_no_next_start();                /*右上角后尚无可执行下一段*/
                    }
                }
                else if (debug_route_active && debug_route_step == 5)
                {
                    debug_no_next_start();                    /*中心点后尚无可执行下一段*/
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
                YawPID.Actual = IMU_GetYaw();          /*直读最新yaw*/
                YawPID.Target = yaw_target_near(yaw_hold, YawPID.Actual);
                PID_Update(&YawPID);

                if (line_axis == 0)
                {
                    PosPID.Target = (float)pos_target;
                    PosPID.Actual = (float)cur;
                    PID_Update(&PosPID);
                    out = PosPID.Out;
                }
                else
                {
                    PosXPID.Target = (float)pos_target;
                    PosXPID.Actual = (float)cur;
                    PID_Update(&PosXPID);
                    out = PosXPID.Out;
                }
                out = pos_speed_limit(out, aerr);
                if (line_axis == 0)
                    Motor_Move(0, (int)out, (int)(YawPID.Out * YAW_OUT_SIGN));
                else
                    Motor_Move((int)out, 0, (int)(YawPID.Out * YAW_OUT_SIGN));
            }
        }

        /*---- 缓行直行+扫码段(10ms 周期; axis=0里程计1/vy  1=里程计2/vx) ----
           低速逼近+二维码判停; 扫到码或到位即完成步骤(码的位置即最终停靠点) */
        if (phase == PHASE_SLOW)
        {
            OdomData_t o;
            int32_t cur;
            float err;
            PosRunState state;
            odometry_get(&o);
            cur  = (slow_axis == 0) ? o.enc_pos[0] : o.enc_pos[1];
            err  = (float)pos_target - (float)cur;
            state = pos_run_check(cur, o.online[slow_axis]);

            if (state == POS_FAILED)
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
                YawPID.Actual = IMU_GetYaw();          /*缓行段仍保持航向*/
                YawPID.Target = yaw_target_near(yaw_hold, YawPID.Actual);
                PID_Update(&YawPID);

                if (slow_axis == 0)
                    Motor_Move(0, slow_dir * POS_QR_SLOW_RPM, (int)(YawPID.Out * YAW_OUT_SIGN));
                else
                    Motor_Move(slow_dir * POS_QR_SLOW_RPM, 0, (int)(YawPID.Out * YAW_OUT_SIGN));
            }
        }


        vTaskDelay(pdMS_TO_TICKS(10));
    }
#endif
}
