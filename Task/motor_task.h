#ifndef __MOTOR_TASK_H
#define __MOTOR_TASK_H

#include "stm32f4xx.h"
#include <stdbool.h>

/**
  * 四路 Emm_V5（张大头 ZDT 闭环步进）电机控制任务
  * 参考例程：gcs-gold-medal-main/yyb_stm32/can_ZDT/yyb_move.c
  *   car_move()            -> 四轮速度直控（一次下发四轮）
  *   car_move_distance_x/y()-> 四轮位置模式 + Emm_V5_Synchronous_motion() 同步起步
  *
  * 硬件：CAN2 —— PB12 = CAN2_RX、PB13 = CAN2_TX，500Kbps
  *       初始化由 Hardware/board.c 的 board_init() 完成（main.c 中调用）
  *
  * 注意：本文件的所有发送都是阻塞式（can_SendCmd 内部等 TX 完成，最长约 20ms/帧），
  *       必须在 FreeRTOS 任务里调用，且任务栈 >= 256 字。
  */

/* ==================== 车体布局（地址分配，装车前先看这段） ====================
   四个电机按"左前 / 右前 / 左后 / 右后"四个角安装，地址分配为：
       左前 = addr 2      右前 = addr 1
       左后 = addr 3      右后 = addr 4
   → **addr 2 与 addr 3 是同一侧（左侧）的两个轮子（一前一后）
      addr 1 与 addr 4 是同一侧（右侧）的两个轮子**
   斜对角关系：addr 2 ↔ addr 4 一条对角线；addr 1 ↔ addr 3 另一条对角线
   （地址改在 motor_task.c 顶部的 Motor_Addr[] / Motor_FwdDir[] 两个数组）
   验证方法见 motor_task.c 顶部注释。
   ========================================================================= */

/* ==================== 可调参数（按实际底盘改这里即可） ==================== */
#define MOTOR_NUM                4       /* 电机数量：四轮底盘 */

#define MOTOR_DEF_VEL            50     /* 默认速度(RPM)         0 ~ 5000 */
#define MOTOR_DEF_ACC            200     /* 默认加速度            0 ~ 255，0 = 直接启动 */
#define MOTOR_MAX_VEL            2000    /* 速度限幅保护(RPM)，防止手滑填出天量速度 */

/* 1 = 四轮指令先缓存进各自电机，最后广播同步帧，四轮同一时刻起步（推荐，底盘不跑偏）
   0 = 逐条立即执行（与例程 car_move() 的 snF=false 一致）                        */
#define MOTOR_SYNC_START         1

/* 45° 斜走的分解系数（0.7071 × 10000 = 7071，用整数避免引入浮点库）
   斜走时 vx = vy = speed × 0.7071，这样合成速度与直行的 speed 一致 */
#define MOTOR_DIAG_COEF          7071

/* 1 = 上电后自动跑一遍四轮自检动作（前进/后退/平移/自转循环）
   0 = 只创建任务，等其它任务调 Motor_Move()，当前=0：MotorTask 轮询执行
       ZigBee(UART7) 收到的脉冲定距命令（串口助手调车 + 里程计标定）        */
#define MOTOR_TASK_AUTO_DEMO     0

/* ==================== 对外接口 ==================== */

/* 四轮速度直控：s1~s4 为带符号转速(RPM)，正负即方向，自动查表定方向位并限幅。
   一次调用把四个电机全下发了；s1~s4 对应例程的 speed[1]~speed[4] */
void Motor_SetSpeed4(int s1, int s2, int s3, int s4);

/* 麦克纳姆轮运动学（与例程 car_move() 逐字一致）：
   vx > 0 左移（左右轴），vy > 0 前进（前后轴），w > 0 逆时针自转，单位 RPM
   注意例程的轴命名：v_x 管左右、v_y 管前后，别按"x=前后"去理解 */
void Motor_Move(int vx, int vy, int w);

/* 45° 斜向平移（麦轮走对角线只需两个轮子出力，另两个停转）：
   dir_x ：+1 含向左分量 / -1 含向右分量
   dir_y ：+1 含向前分量 / -1 含向后分量
   speed ：合成速度(RPM)，量级与直行相同
   例：Motor_MoveDiagonal(1, 1, 150) = 左前方 45° 斜走 */
void Motor_MoveDiagonal(int8_t dir_x, int8_t dir_y, uint16_t speed);

/* 四轮位置模式（相对运动）：pulse[i] 的符号 = 方向，绝对值 = 脉冲数(0 ~ 2^32-1)，
   四轮同步广播起步。用于"走固定距离"这类开环定位 */
void Motor_MovePulses(const int32_t pulse[MOTOR_NUM]);

/* 立即停止四轮 */
void Motor_Stop(void);

/* 使能(true)/失能(false)四轮 */
void Motor_Enable(bool state);

/* FreeRTOS 任务入口 */
void MotorTask(void *pvParameters);
uint8_t Motor_NoNext(void);             /*未知下一段时返回1，供LCD显示no next*/
int32_t Motor_RoutePos(int32_t encoder_pos); /*原始编码值换算为最近路线的累计pos*/

#endif
