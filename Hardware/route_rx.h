#ifndef __ROUTE_RX_H
#define __ROUTE_RX_H

#include "stm32f4xx.h"

/* 智能搬运赛场路线坐标帧接收（与网页版上位机"发送坐标"配套，移植自触摸屏例程）
   帧格式：AA 55 | 01 | BATCH | N | X1L X1H Y1L Y1H | ... | SUM | 0D 0A
   坐标 mm 小端；SUM = 01+BATCH+N+全部坐标字节 累加和低8位
   在 UART7（PE7=RX / PE8=TX）接收中断里逐字节调用 route_rx_byte() 喂字节
   上限 64 点/批，两批分存 g_route[0]/g_route[1] */

#define ROUTE_MAX_PT  64

#define CMD_ROUTE_DOWN  0x01     /* 上位机 -> 车端：路线段下发(直角段) */
#define CMD_ROUTE_UP    0x02     /* 车端 -> 上位机：坐标回传（网页右上回传板显示） */
#define CMD_ROUTE45     0x03     /* 上位机 -> 车端：路线段下发(含45°斜走段, 结构同01) */
#define CMD_ACTION      0x04     /* 上位机 -> 车端：动作指令帧(7字节, 见下) */
#define CMD_PULSE       0x10     /* 上位机 -> 车端：脉冲定距运动（串口助手调试用, 原0x03改号让位网页协议） */
#define CMD_PULSE_ACK   0x11     /* 车端 -> 上位机：脉冲命令应答（内容同收到的10帧, 原0x04改号） */

/* 脉冲运动帧（串口助手 HEX 发送，共10字节，原 0x03 改号 0x10 让位网页协议）：
   AA 55 | 10 | DIR | PL0 PL1 PL2 PL3 | SUM | 0D 0A
   DIR  ：0x00=前进  0x01=后退  0x02=左移  0x03=右移
          0x04=左前45° 0x05=右前45° 0x06=左后45° 0x07=右后45°
          0x08=原地逆时针  0x09=原地顺时针
          （直行/斜走：出力轮各走 PL 脉冲，斜走位移≈直走的0.707倍；
            自转：四个轮子两两反向各走 PL 脉冲，转角∝PL，需实测标定）
   PL0~3：出力轮脉冲数 uint32 小端（低字节在前），1 ~ PULSE_MAX
   SUM  ：10+DIR+PL0~PL3 累加和低8位
   应答帧：AA 55 | 11 | 同载荷 | SUM | 0D 0A（车端收到并执行时回）
   例：前进10000脉冲 = 10000(0x2710) -> 10 27 00 00，
       SUM = 10+00+10+27+00+00 = 47
       发送：AA 55 10 00 10 27 00 00 47 0D 0A                       */
#define PULSE_DIR_FWD    0x00
#define PULSE_DIR_BACK   0x01
#define PULSE_DIR_LEFT   0x02
#define PULSE_DIR_RIGHT  0x03
#define PULSE_DIR_FL     0x04    /* 左前45°: 出力 = RF(addr1) + LR(addr3) */
#define PULSE_DIR_FR     0x05    /* 右前45°: 出力 = LF(addr2) + RR(addr4) */
#define PULSE_DIR_BL     0x06    /* 左后45°: 出力 = LF(addr2) + RR(addr4), 反向 */
#define PULSE_DIR_BR     0x07    /* 右后45°: 出力 = RF(addr1) + LR(addr3), 反向 */
#define PULSE_DIR_CCW    0x08    /* 原地旋转: 1&2号电机同向, 3&4号电机反向 */
#define PULSE_DIR_CW     0x09    /* 原地旋转: 与CCW相反 */
#define PULSE_MAX        1000000u

/* ---- 定位闭环帧（串口助手调试：pos位置环+yaw保持，MotorTask执行） ----
   05 闭环帧: AA 55 | 05 | SUB | T0 T1 T2 T3 | SUM | 0D 0A   (10字节)
            SUB: 0x00=直线定位(T=目标pos,反馈轴=ODOM_POS_AXIS)
                 0x01=右前45°段(T=段末pos,段末自动衔接直线段)
                 0x04=左前45°段(T=段末pos,同上)
                 0x05=缓行扫码段(T=缓行目标pos, 扫到码即停)
                 0x07=回右上角点位段(T=目标pos, 场地右上角=200000)
                 0x02=原地左转(T=角度deg 1~179)  0x03=原地右转(T=角度deg)
   06 中止帧: AA 55 | 06 | SUM | 0D 0A                       (6字节)
   应答帧  : 车端收到后按原命令字回显(载荷同收到的帧)，串口助手可见即确认链路
   例: 直线定位到 700000 = 0x0AAB30 -> 30 AB 0A 00,
       SUM = 05+00+30+AB+0A+00 = E4
       发送: AA 55 05 00 30 AB 0A 00 E4 0D 0A
   例: 左转90°: 90=0x5A, SUM = 05+02+5A = 61
       发送: AA 55 05 02 5A 00 00 00 61 0D 0A                       */
#define CMD_POS_GO       0x05    /* 上位机 -> 车端：开始位置闭环(目标=参数值) */
#define CMD_POS_ABORT    0x06    /* 上位机 -> 车端：中止位置闭环 */
/* ---- 一键任务链帧（比赛现场一条指令跑完整链：45°斜走->直线->缓行扫码停车） ----
   07 帧: AA 55 | 07 | DIR | D0 D1 D2 D3 | L0 L1 L2 L3 | SUM | 0D 0A   (15字节)
   DIR  : 0x01=右前45°(启停区一) 0x04=左前45°(启停区二)
   D0~D3: 45°斜走段末位置 int32 小端(编码值)
   L0~L3: 直线段目标位置 int32 小端(编码值)
   流程: 45°斜走到段末 -> 自动衔接直线 -> 距直线目标<QR_SLOW(≈15cm)转缓行
         -> MaixCam 扫到二维码即停车(未扫到则走到目标兜底停)
   应答帧: 原样回显(命令字07)
   例: 启停区一(右前, 段末1500000, 直线650000):
       1500000=0x16E360->60 E3 16 00; 650000=0x09EB10->10 EB 09 00
       SUM = 07+01+60+E3+16+00+10+EB+09 = 65
       发送: AA 55 07 01 60 E3 16 00 10 EB 09 00 65 0D 0A               */
#define CMD_CHAIN        0x07    /* 上位机 -> 车端：一键任务链(45°+直线+缓行扫码) */
#define CMD_POS_ADV      0x08    /* 上位机 -> 车端：高级直线闭环(位置模式+目标yaw) */
/* 08帧: AA 55 | 08 | SUB | FLAGS | P0 P1 P2 P3 | Y0 Y1 | SUM | 0D 0A
   FLAGS bit0=1:位置为相对当前值的增量, 0:绝对目标值
          bit1=1:使用Y字段作为绝对目标yaw(deg), 0:保持收到命令时的yaw
   Y为小端角度值, 单位度, 按LCD显示坐标填写范围0~360; 当前仅SUB=0直线段。 */
/* ---- 一键任务数组帧(车端按内置任务步骤表从当前步骤连跑至尾) ----
   12 帧: AA 55 12 12 0D 0A (4字节, 无载荷) ---- */
#define CMD_RUNALL       0x12    /* 上位机 -> 车端：一键执行任务步骤数组 */


/* ---- 动作指令帧(上位机 -> 车端, 7字节): AA 55 | 04 | BATCH | CODE | SUM | 0D 0A ----
   BATCH: 1=第一批 2=第二批
   CODE : 1=前进 2=后退 3=顺时针转90度 4=逆时针转90度 (相对当前车头朝向)
   车头初始朝向恒为向下(启停区1/2通用), 航向跨批连续;
   移动方向与车头相差180°必须用后退(禁止两个90°转弯凑), 相差90°用对应转弯指令;
   45°出发斜走按其移动方向归为"向下"(启停区1)/"向上"(启停区2)参与判断。
   车端收到合法动作帧后: 记录动作码 + 整帧原样压入回传FIFO(逐帧原样回传网页) ---- */
#define CMD_ACT_FWD      1       /* 前进 */
#define CMD_ACT_BACK     2       /* 后退 */
#define CMD_ACT_CW       3       /* 顺时针转90度 */
#define CMD_ACT_CCW      4       /* 逆时针转90度 */

typedef struct
{
    volatile uint8_t  type;     /* 命令字: CMD_PULSE / CMD_POS_GO / CMD_POS_ABORT */
    volatile uint8_t  dir;      /* 03帧: 方向码 */
    volatile uint8_t  sub;      /* 05/08帧: 子命令 */
    volatile uint8_t  flags;    /* 08帧: bit0相对位置, bit1指定yaw */
    volatile uint32_t param;    /* 03帧: 脉冲数; 05帧: 目标位置; 07帧: 45°段末位置 */
    volatile uint32_t param2;   /* 07帧: 直线段目标位置(编码值) */
    volatile int16_t  yaw_target; /* 08帧: LCD坐标目标yaw(0~360deg) */
    volatile uint8_t  ready;    /* 1 = 有未取走的新命令 */
    volatile uint8_t  cnt;      /* 累计收到的有效命令数(饱和255) */
} ZbeeCmd;

/* ---- 通用命令槽（CMD 03/05/06，ISR写 / MotorTask取） ---- */
uint8_t cmd_get(ZbeeCmd *c);                     /* 取走一条新命令，1=有 */
void cmd_last(ZbeeCmd *c, uint8_t *cnt);         /* LCD 只读最后命令 */
void cmd_tx_ack(const ZbeeCmd *c);               /* 按原命令字+载荷回应答帧(任务上下文) */

typedef struct
{
    uint8_t batch;                    /* 1=第一批 2=第二批 */
    uint8_t count;                    /* 关键点数量 */
    uint16_t x[ROUTE_MAX_PT];         /* 各点 X 坐标（mm，场地左上原点） */
    uint16_t y[ROUTE_MAX_PT];         /* 各点 Y 坐标（mm） */
    volatile uint8_t ready;           /* 收到完整一帧并校验通过 */
} RouteFrame;

void route_rx_init(void);             /* 初始化UART7发送互斥(main.c调度器前调用) */
void route_rx_byte(uint8_t ch);       /* 中断里喂字节 */
void route_rx_poll(void);             /* 帧中途超时复位(任务上下文周期调用, LCD_Task 10ms) */
uint8_t route_act_get(void);          /* 取走04动作帧动作码(1前进/2后退/3顺/4逆), 0=无新帧 */
void route_echo_flush(void);          /* 回传FIFO排空: 逐字节经UART7原样发回网页(任务上下文) */
uint8_t route_ready(uint8_t batch);   /* batch: 1~2 */
void route_clear_ready(uint8_t batch);
RouteFrame* route_get(uint8_t batch);
void route_tx_uplink(uint8_t batch);  /* 经 UART7(ZigBee) 按命令字 02 帧回传该批坐标到网页(旧接口, 已被逐帧回传取代, 保留备用) */



#endif
