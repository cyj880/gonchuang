#ifndef __UART8_H
#define __UART8_H

#include "stm32f4xx.h"

/**
  * UART8 —— 摄像头链路 + 任务码显示屏转发口（一根串口同时收发）
  *
  *   PE0 = UART8_RX  ← 上位机 MaixCam 的 TX：收二维码内容(任务码) + 码中心横坐标 x
  *   PE1 = UART8_TX  → 任务码显示屏(F103) 的 USART2_RX：转发任务码 + 搬运信息
  *
  *   摄像头只发不收、显示屏只收不发，所以一根口就能把两边接上，不会打架：
  *     MaixCam TX(A30/A16) ──> PE0
  *     PE1 ──> 显示屏 PA3(USART2_RX)
  *     显示屏 PA2 不接（原来它把任务码回传给主控，现在任务码由摄像头直接给主控，用不上了）
  *     双方共地，115200-8-N-1
  *
  * 摄像头帧格式（见 MaixPy projects/app_scan_qrcode/scan_uart.py）：
  *   AA 55 | LEN | 内容(LEN 字节, UTF-8, 二维码原文) | x低 | x高 | SUM
 *   SUM = (LEN + 全部内容字节 + x低 + x高) 累加和低 8 位
 *   画面里没有码时摄像头静默不发；车端用"多久没来新帧"判无码：
 *   超过 CAM_FRESH_MS 没收到有效帧，UART8_CamHasTarget() 返回 0（应停止纠偏）
 *   x = 二维码中心在图像里的横坐标（0~639，图像 640x480）
  *
  * 任务码（二维码原文，例如 "426+213+432+123"）：
  *   组1 = 第一批三件物料的颜色与搬运顺序
  *   组2 = 第一批物料在粗加工区/暂存区的圆环号（两个区同一环号）
  *   组3 = 第二批三件物料的颜色与搬运顺序
  *   组4 = 第二批物料在粗加工区的圆环号
  *   第二批在暂存区只能码垛在同色的第一批物料上
  *   颜色编号：红1 黄2 蓝3 绿4 黑5 浅蓝6；环号 1~6
  *   ★ 从二维码内容里只取数字，集满 12 位即算一帧，分隔符无所谓
  *     （'+'、空格、逗号，甚至完全没有分隔符都能解析）
  *
  * 转发给显示屏的内容（都以 CRLF 结尾，显示屏按行解析）：
  *   任务码行：426+213+432+123
  *   搬运信息行：S,抓取正确次数,放置正确次数,当前载物数量     例：S,3,2,2
  */

#define TASK_LINK_GROUPS   4
#define TASK_LINK_GLEN     3
#define TASK_LINK_DIGITS   (TASK_LINK_GROUPS * TASK_LINK_GLEN)   /* 12 位 */

#define CAM_PAY_MAX        64      /* 摄像头一帧的内容最大字节数 */
#define CAM_IMG_W          640     /* 摄像头图像宽度，用于算偏差 */
#define CAM_X_LOST         0xFFFF  /* x=0xFFFF 表示画面里没有码（兼容保留） */
#define CAM_FRESH_MS       300     /* 摄像头没码时静默,超过该时间没新帧=画面里无码 */

/* 解析出来的任务码 */
typedef struct
{
    uint8_t  digit[TASK_LINK_DIGITS];   /* 12 位数字字符 '1'~'6' */
    uint8_t  color[2][3];               /* color[批][第几件] = 颜色编号 1~6，批 0=第一批 */
    uint8_t  ring[2][3];                /* ring[批][第几件] = 粗加工区圆环号 1~6 */
    uint8_t  valid;                     /* 1 = 已收到过合法任务码 */
    uint8_t  bad;                       /* 1 = 12 位里有 0/7/8/9（不合法） */
    volatile uint8_t changed;           /* 1 = 来了与上次不同的新码，处理后调 UART8_ClearChanged() */
    volatile uint16_t seq;              /* 每完整收到一帧 +1（含重复帧） */
} TaskLinkCode;

/* 本端要上报的搬运信息 */
typedef struct
{
    uint8_t grab;       /* 抓取正确次数 0~6 */
    uint8_t place;      /* 放置正确次数 0~6 */
    uint8_t load;       /* 当前车载（已夹取）物料数量 0~3 */
} TaskLinkStat;

extern TaskLinkCode g_link;
extern TaskLinkStat g_link_stat;

/* ---------------- 串口本身 ---------------- */
void UART8_Init(uint32_t baud);          /* PE0=RX 收摄像头 / PE1=TX 转发给显示屏 */
void UART8_SendByte(uint8_t byte);
void UART8_SendString(const char *str);

/* ---------------- 初始化与接收 ---------------- */
void     UART8_LinkInit(void);           /* 解析器与统计清零，初始化后调一次 */
void     UART8_RxByte(uint8_t ch);       /* ★ UART8 接收中断里逐字节调用 */

/* ---------------- 任务码查询 ---------------- */
uint8_t  UART8_Valid(void);
uint8_t  UART8_NewCode(void);            /* 1 = 有新码（内部不清标志） */
void     UART8_ClearChanged(void);
uint16_t UART8_Seq(void);
void     UART8_GetCode(char *out);       /* 输出 "426+213+432+123"，out 至少 16 字节 */
void     UART8_GetGroup(uint8_t g, char *out);            /* g:0~3，输出 "426" */
uint8_t  UART8_Color(uint8_t batch, uint8_t idx);         /* batch:1~2  idx:0~2 -> 1~6 */
uint8_t  UART8_Ring(uint8_t batch, uint8_t idx);          /* 粗加工区环号 1~6 */
uint8_t  UART8_StoreRing(uint8_t idx);                    /* 第一批暂存区环号（与粗加工区同环号） */
int8_t   UART8_StackRing(uint8_t idx);                    /* 第二批第 idx 件码垛到哪个环；-1 = 没同色底座 */
const char* UART8_ColorName(uint8_t color);               /* "RED"/"YEL"/"BLU"/"GRN"/"BLK"/"LBL" */

/* ---------------- 摄像头坐标（码中心横坐标，做横向纠偏用） ---------------- */
uint16_t UART8_CamX(void);               /* 0~639；CAM_X_LOST(0xFFFF) = 画面里没有码 */
uint8_t  UART8_CamHasTarget(void);       /* 1 = 最近一帧有效且画面里有码 */
int16_t  UART8_CamError(void);           /* x - 320，正数表示码偏右；没目标时返回 0 */
uint16_t UART8_CamFrames(void);          /* 收到并校验通过的帧数 */
uint16_t UART8_CamBadFrames(void);       /* 校验失败/超长的帧数 */

/* ---------------- 转发给任务码显示屏 ---------------- */
void     UART8_SendTaskCode(void);        /* 发一行任务码 "426+213+432+123" */
void     UART8_SendStat(void);            /* 发一行搬运信息 "S,g,p,l" */
void     UART8_SetGrab(uint8_t n);
void     UART8_SetPlace(uint8_t n);
void     UART8_SetLoad(uint8_t n);
void     UART8_GrabOk(void);              /* 抓取正确次数 +1 */
void     UART8_PlaceOk(void);             /* 放置正确次数 +1 */
void     UART8_LoadInc(void);             /* 车上物料 +1 */
void     UART8_LoadDec(void);             /* 车上物料 -1 */
void     UART8_StatClear(void);           /* 三项计数清零并立刻上报 */

/* 可选任务：只在内容变化时转发（任务码变了发任务码行，计数变了发状态行），
   不做周期心跳。不需要自动转发可以不创建该任务，改在需要的地方直接调
   UART8_SendTaskCode() / UART8_SendStat() */
void     UART8_ReportTask(void *pvParameters);

#endif
