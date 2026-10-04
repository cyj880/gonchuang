#include "route_rx.h"
#include "uart7.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* ================= 帧接收状态机（串口中断里逐字节喂入） =================
   路线段帧：AA 55 | 01/03 | BATCH | N | X1L X1H Y1L Y1H | ... | SUM | 0D 0A
            （01=直角段 03=含45°斜走段, 结构相同, 收法相同）
   动作帧：  AA 55 | 04 | BATCH | CODE | SUM | 0D 0A   （CODE: 1前进/2后退/3顺时针转90度/4逆时针转90度）
   脉冲帧： AA 55 | 10 | DIR | PL0 PL1 PL2 PL3 | SUM | 0D 0A   （调试：定距运动, 原03改号）
   定位帧： AA 55 | 05 | SUB | T0 T1 T2 T3 | SUM | 0D 0A   （pos闭环/45°段/原地转向）
   链式帧： AA 55 | 07 | DIR | D0 D1 D2 D3 | L0 L1 L2 L3 | SUM | 0D 0A （一键任务链）
          SUB=0 直线定位(T=目标pos编码值) / SUB=1 右前45°段(T=段末pos) / SUB=4 左前45°段(同)
          SUB=5 缓行扫码(T=缓行目标pos) / SUB=6 二维码校准(T=0)
          SUB=2 原地左转(T=角度deg, 1~179)  / SUB=3 原地右转(T=角度deg)
   中止帧：AA 55 | 06 | SUM | 0D 0A                           （中止定位/转向闭环）
   回传帧：AA 55 | 02 | BATCH | N | X1L X1H Y1L Y1H | ... | SUM | 0D 0A
   应答帧：AA 55 | 11 | DIR | PL0 PL1 PL2 PL3 | SUM | 0D 0A   （对10帧的应答）
   帧头/命令/长度/校验/帧尾五重验证，坏帧自动丢弃重新同步。
   两批数据分存 g_route[0]/g_route[1]，帧尾 0D 0A 校验通过才置 ready 供任务取用。
   01/03/04 网页协议帧校验通过后整帧原样压入回传FIFO，LCD_Task 排空经 UART7 发回网页。
   05/06 帧校验通过后先暂存，帧尾验证通过才提交 g_cmd（防坏帧触发运动）。 */

#define FRAME_HEAD1   0xAA
#define FRAME_HEAD2   0x55
#define CMD_ROUTE     0x01     /* 直角段坐标帧 */
#define CMD_ROUTE45   0x03     /* 含45°斜走段坐标帧(结构同01, 收法相同) */
#define CMD_ACTION    0x04     /* 动作指令帧(码1前进/2后退/3顺/4逆) */
#define CMD_UPLINK    0x02
#define CMD_PULSE     0x10     /* 脉冲定距运动(调试, 原0x03改号让位网页协议) */
#define CMD_POS_GO    0x05
#define CMD_POS_ABORT 0x06
#define FRAME_TAIL1   0x0D
#define FRAME_TAIL2   0x0A

static RouteFrame g_route[2];

/* ---- UART7 帧级发送互斥 ----
   cmd_tx_ack(MotorTask) 与 route_tx_uplink(LCD_Task) 跨任务共用 UART7,
   无锁时两帧字节交错 -> 网页与串口助手都收到坏帧。帧级锁保证一帧完整发完。 */
static SemaphoreHandle_t s_uart7_tx_mutex;
static StaticSemaphore_t s_uart7_tx_mutex_mem;

void route_rx_init(void)
{
    s_uart7_tx_mutex = xSemaphoreCreateMutexStatic(&s_uart7_tx_mutex_mem);
}     /* [0]=第一批 [1]=第二批 */

/* 通用命令槽：ISR 写 / MotorTask 取（uint32 对齐写原子，ready 最后置位）
   03帧: dir=方向码 param=脉冲数; 05帧: sub=0直线/1右前45°/2左转/3右转/4左前45°/5缓行扫码/6二维码校准/7回右上角 */
typedef struct
{
    volatile uint8_t  type;         /* CMD_PULSE / CMD_POS_GO / CMD_POS_ABORT */
    volatile uint8_t  dir;          /* 03帧: 方向码 */
    volatile uint8_t  sub;          /* 05帧: 0=开始 1=中止 */
    volatile uint32_t param;        /* 03帧: 脉冲数; 05帧: 目标位置; 07帧: 45°段末 */
    volatile uint32_t param2;       /* 07帧: 直线段目标位置 */
    volatile uint8_t  ready;        /* 1 = 有未取走的新命令 */
    volatile uint8_t  cnt;          /* 累计收到的有效命令数(饱和255) */
} ZbeeCmdSlot;

static ZbeeCmdSlot g_cmd;

/* ---- 最近动作码(04动作帧): 已原样回传网页, 此变量供车端逻辑/调试取用 ---- */
static volatile uint8_t g_act_code = 0;      /* 1前进/2后退/3顺时针转90度/4逆时针转90度 */
static volatile uint8_t g_act_new  = 0;      /* 1 = 有未取走的新动作帧 */

typedef enum
{
    RX_H1, RX_H2, RX_CMD, RX_BATCH, RX_ACT, RX_CNT, RX_DATA, RX_SUM, RX_T1, RX_T2,
    RX_P_DATA
} RxState;

static RxState rx_state = RX_H1;
static uint8_t rx_cmd, rx_batch, rx_cnt, rx_idx, rx_sum_calc;
static uint8_t rx_buf[4 * ROUTE_MAX_PT];
static uint8_t rx_p_dir, rx_p_bytes[9], rx_p_idx, rx_p_len;
static uint8_t rx_act;                                /* 04帧动作码(1前进/2后退/3顺/4逆) */

/* ---- 整帧捕获缓冲: 帧头到帧尾逐字节暂存, 校验通过后整帧原样入回传FIFO ---- */
static uint8_t  rx_frame_buf[8 + 4 * ROUTE_MAX_PT];
static uint16_t rx_frame_len;
static volatile TickType_t rx_last_tick;              /* 最后喂字节时刻(超时复位用) */

/* ---- 回传FIFO: ISR把校验通过的整帧原样入队, LCD_Task排空经UART7发回网页 ----
   单生产者(UART7中断)/单消费者(LCD_Task)环形队列; 满则整帧丢弃并计数(网页帧计数可发现)。
   回传是暂态的: 车开跑前网页把路线发完、车端逐帧回传确认后, FIFO空即闲置, 不常驻占用。 */
#define ECHO_FIFO_SZ 512u
static uint8_t  echo_buf[ECHO_FIFO_SZ];
static volatile uint16_t echo_head = 0, echo_tail = 0;
static uint16_t echo_drop = 0;
static void echo_push_frame(const uint8_t *p, uint16_t n);   /* 前向声明: ISR在校验通过处(274/280)先调用, 定义在后 */

/* 喂入一个接收字节（在 UART7 中断服务函数中调用） */
void route_rx_byte(uint8_t ch)
{
    /* 整帧捕获: 帧头启动新捕获, 非空闲态字节全部追加(供校验通过后原样回传) */
    if (rx_state == RX_H1)
    {
        if (ch == FRAME_HEAD1)
        {
            rx_frame_len = 0;
            rx_frame_buf[rx_frame_len++] = ch;
        }
    }
    else if (rx_frame_len < (uint16_t)sizeof(rx_frame_buf))
    {
        rx_frame_buf[rx_frame_len++] = ch;
    }
    rx_last_tick = xTaskGetTickCount();          /*喂字节时刻(帧中途超时复位用)*/

    switch (rx_state)
    {
    case RX_H1:
        if (ch == FRAME_HEAD1) rx_state = RX_H2;
        break;

    case RX_H2:
        rx_state = (ch == FRAME_HEAD2) ? RX_CMD : RX_H1;
        break;

    case RX_CMD:
        rx_sum_calc = ch;
        rx_cmd = ch;
        if (ch == CMD_ROUTE || ch == CMD_ROUTE45)  rx_state = RX_BATCH;
        else if (ch == CMD_ACTION)                 rx_state = RX_BATCH;   /*批号后跟1字节动作码*/
        else if (ch == CMD_PULSE)  { rx_p_idx = 0; rx_p_len = 5; rx_state = RX_P_DATA; }
        else if (ch == CMD_POS_GO) { rx_p_idx = 0; rx_p_len = 5; rx_state = RX_P_DATA; }
        else if (ch == CMD_POS_ABORT) rx_state = RX_SUM;   /*无载荷, 累加和=命令字本身*/
        else if (ch == CMD_CHAIN)  { rx_p_idx = 0; rx_p_len = 9; rx_state = RX_P_DATA; }
        else                       rx_state = RX_H1;
        break;

    case RX_BATCH:
        if (ch < 1 || ch > 2) { rx_state = RX_H1; break; }   /* 批号非法丢帧 */
        rx_batch = ch;
        rx_sum_calc += ch;
        rx_state = (rx_cmd == CMD_ACTION) ? RX_ACT : RX_CNT;
        break;

    case RX_ACT:                                 /* 04帧动作码: 1前进/2后退/3顺/4逆 */
        if (ch < 1 || ch > 4) { rx_state = RX_H1; break; }
        rx_act = ch;
        rx_sum_calc += ch;
        rx_state = RX_SUM;
        break;

    case RX_CNT:
        rx_cnt = ch;
        rx_sum_calc += ch;
        if (rx_cnt == 0 || rx_cnt > ROUTE_MAX_PT) { rx_state = RX_H1; break; }
        rx_idx = 0;
        rx_state = RX_DATA;
        break;

    case RX_DATA:
        rx_buf[rx_idx++] = ch;
        rx_sum_calc += ch;
        if (rx_idx >= (uint16_t)(4 * rx_cnt)) rx_state = RX_SUM;
        break;

    /* 03/05帧载荷: 字节0(方向/子命令) + 4字节参数(小端) 共5字节, 字节0合法性即时校验 */
    case RX_P_DATA:
        if (rx_p_idx == 0)
        {
            uint8_t bmax = 0;
            if (rx_cmd == CMD_PULSE)       bmax = PULSE_DIR_CW;   /*方向码 0~9*/
            else if (rx_cmd == CMD_POS_GO) bmax = 7;              /*子命令 0=直线 1=右前45° 2=左转 3=右转 4=左前45° 5=缓行扫码 6=二维码校准 7=回右上角*/
            else if (rx_cmd == CMD_CHAIN)  bmax = 4;              /*方向: 仅1=右前 4=左前合法(见下)*/
            else { rx_state = RX_H1; break; }
            if (rx_cmd == CMD_CHAIN && ch != 1 && ch != 4) { rx_state = RX_H1; break; }
            if (ch > bmax) { rx_state = RX_H1; break; }
            rx_p_dir = ch;                 /*复用: 03存方向, 05存子命令*/
        }
        else
        {
            rx_p_bytes[rx_p_idx - 1] = ch;
        }
        rx_sum_calc += ch;
        if (++rx_p_idx >= rx_p_len) rx_state = RX_SUM;
        break;

    case RX_SUM:
        if ((uint8_t)rx_sum_calc == ch)
        {
            if (rx_cmd == CMD_PULSE || rx_cmd == CMD_POS_GO || rx_cmd == CMD_CHAIN)
            {
                /* 03/05帧：帧尾通过后再提交（见 RX_T2），此处只留状态 */
                rx_state = RX_T1;
            }
            else if (rx_cmd == CMD_ROUTE || rx_cmd == CMD_ROUTE45)
            {
                RouteFrame* f = &g_route[rx_batch - 1];
                uint8_t i;
                for (i = 0; i < rx_cnt; i++)
                {
                    f->x[i] = (uint16_t)rx_buf[4 * i]     | ((uint16_t)rx_buf[4 * i + 1] << 8);
                    f->y[i] = (uint16_t)rx_buf[4 * i + 2] | ((uint16_t)rx_buf[4 * i + 3] << 8);
                }
                f->batch = rx_batch;
                f->count = rx_cnt;
                /*ready 在帧尾校验通过后置位(见RX_T2), 防坏帧尾污染显示*/
                rx_state = RX_T1;
            }
            else if (rx_cmd == CMD_ACTION)
            {
                /*动作码已在RX_ACT校验暂存, 此处只留状态, 帧尾通过后提交(见RX_T2)*/
                rx_state = RX_T1;
            }
            else rx_state = RX_H1;
        }
        else rx_state = RX_H1;
        break;

    case RX_T1:
        rx_state = (ch == FRAME_TAIL1) ? RX_T2 : RX_H1;
        break;

    case RX_T2:
        if (ch == FRAME_TAIL2)
        {
            if (rx_cmd == CMD_PULSE)
            {
                uint32_t pl = (uint32_t)rx_p_bytes[0] |
                              ((uint32_t)rx_p_bytes[1] << 8) |
                              ((uint32_t)rx_p_bytes[2] << 16) |
                              ((uint32_t)rx_p_bytes[3] << 24);
                if (pl >= 1 && pl <= PULSE_MAX)
                {
                    g_cmd.type   = CMD_PULSE;
                    g_cmd.param  = pl;                   /*uint32对齐写, 原子*/
                    g_cmd.dir    = rx_p_dir;
                    g_cmd.sub    = 0;
                    if (g_cmd.cnt < 255) g_cmd.cnt++;
                    g_cmd.ready  = 1;                    /*最后置位, MotorTask取走*/
                }
            }
            else if (rx_cmd == CMD_POS_GO)
            {
                g_cmd.type   = CMD_POS_GO;
                g_cmd.sub    = rx_p_dir;                 /*0=开始 1=中止*/
                g_cmd.param  = (uint32_t)rx_p_bytes[0] |
                               ((uint32_t)rx_p_bytes[1] << 8) |
                               ((uint32_t)rx_p_bytes[2] << 16) |
                               ((uint32_t)rx_p_bytes[3] << 24);
                if (g_cmd.cnt < 255) g_cmd.cnt++;
                g_cmd.ready  = 1;
            }
            else if (rx_cmd == CMD_POS_ABORT)
            {
                g_cmd.type   = CMD_POS_ABORT;
                if (g_cmd.cnt < 255) g_cmd.cnt++;
                g_cmd.ready  = 1;
            }
            else if (rx_cmd == CMD_CHAIN)
            {
                g_cmd.type   = CMD_CHAIN;
                g_cmd.dir    = rx_p_dir;                 /*1=右前45° 4=左前45°*/
                g_cmd.sub    = 0;
                g_cmd.param  = (uint32_t)rx_p_bytes[0] |   /*45°段末位置*/
                               ((uint32_t)rx_p_bytes[1] << 8) |
                               ((uint32_t)rx_p_bytes[2] << 16) |
                               ((uint32_t)rx_p_bytes[3] << 24);
                g_cmd.param2 = (uint32_t)rx_p_bytes[4] |   /*直线段目标位置*/
                               ((uint32_t)rx_p_bytes[5] << 8) |
                               ((uint32_t)rx_p_bytes[6] << 16) |
                               ((uint32_t)rx_p_bytes[7] << 24);
                if (g_cmd.cnt < 255) g_cmd.cnt++;
                g_cmd.ready  = 1;
            }
            else if (rx_cmd == CMD_ROUTE || rx_cmd == CMD_ROUTE45)
            {
                g_route[rx_batch - 1].ready = 1;                 /*帧尾校验通过, 正式提交*/
                echo_push_frame(rx_frame_buf, rx_frame_len);     /*整帧原样入回传FIFO, LCD_Task排空发回网页*/
            }
            else if (rx_cmd == CMD_ACTION)
            {
                g_act_code = rx_act;                             /*最近动作码(1前进/2后退/3顺/4逆, 备用)*/
                g_act_new  = 1;
                echo_push_frame(rx_frame_buf, rx_frame_len);     /*动作帧也原样回传网页*/
            }
        }
        rx_state = RX_H1;
        break;

    default:
        rx_state = RX_H1;
        break;
    }

    if (rx_state == RX_H1) rx_frame_len = 0;   /*回同步/收完: 捕获清零(整帧已在T2入回传FIFO)*/
}

/* ---- 回传FIFO入队（ISR内调用, 只入队不发送）; 满则整帧丢弃并计数 ---- */
static void echo_push_frame(const uint8_t *p, uint16_t n)
{
    uint16_t free = (uint16_t)((echo_tail - echo_head - 1u + ECHO_FIFO_SZ) % ECHO_FIFO_SZ);
    uint16_t k;
    if (n > free) { echo_drop++; return; }
    for (k = 0; k < n; k++)
    {
        echo_buf[echo_head] = p[k];
        echo_head = (uint16_t)((echo_head + 1u) % ECHO_FIFO_SZ);
    }
}

/* ---- 帧中途超时复位 + 回传FIFO排空（任务上下文周期调用, LCD_Task 10ms） ----
   route_rx_poll   : 帧中途 >200ms 无新字节 -> 丢弃半帧回空闲(防字节丢失卡死状态机)
   route_echo_flush: 把回传FIFO里校验通过的整帧逐字节经 UART7 原样发回网页(逐帧原样回传) */
void route_rx_poll(void)
{
    if (rx_state != RX_H1)
    {
        if ((xTaskGetTickCount() - rx_last_tick) > pdMS_TO_TICKS(200))
            rx_state = RX_H1;                    /*帧中途超时: 丢弃半帧重新同步*/
    }
}

void route_echo_flush(void)
{
    if (s_uart7_tx_mutex == 0) return;
    if (xSemaphoreTake(s_uart7_tx_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;  /*回传/应答帧在发*/

    while (echo_tail != echo_head)
    {
        UART7_SendByte(echo_buf[echo_tail]);
        echo_tail = (uint16_t)((echo_tail + 1u) % ECHO_FIFO_SZ);
    }
    xSemaphoreGive(s_uart7_tx_mutex);
}

uint8_t   route_ready(uint8_t batch)         { return g_route[batch - 1].ready; }
void      route_clear_ready(uint8_t batch)   { g_route[batch - 1].ready = 0; }
RouteFrame* route_get(uint8_t batch)         { return &g_route[batch - 1]; }

/* ================= 坐标回传：按命令字 02 帧经 UART7(ZigBee) 发回网页 =================
   帧结构与下发帧同构（AA 55 | 02 | BATCH | N | 坐标小端 | SUM | 0D 0A），
   网页端按 02 帧解析并显示在右上"STM32回传"板。任务上下文调用（阻塞发送约 4ms/帧）。 */
void route_tx_uplink(uint8_t batch)
{
    const RouteFrame* f = &g_route[batch - 1];
    uint8_t sum;
    uint8_t i;
    uint16_t x, y;

    if (batch < 1 || batch > 2) return;
    if (f->count == 0 || f->count > ROUTE_MAX_PT) return;
    if (s_uart7_tx_mutex == 0) return;
    if (xSemaphoreTake(s_uart7_tx_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;  /*对方长帧在发, 放弃本次回传*/

    sum = (uint8_t)(CMD_UPLINK + batch + f->count);

    UART7_SendByte(FRAME_HEAD1);
    UART7_SendByte(FRAME_HEAD2);
    UART7_SendByte(CMD_UPLINK);
    UART7_SendByte(batch);
    UART7_SendByte(f->count);

    for (i = 0; i < f->count; i++)
    {
        x = f->x[i];
        y = f->y[i];
        UART7_SendByte((uint8_t)(x & 0xFF));
        UART7_SendByte((uint8_t)((x >> 8) & 0xFF));
        UART7_SendByte((uint8_t)(y & 0xFF));
        UART7_SendByte((uint8_t)((y >> 8) & 0xFF));
        sum += (uint8_t)(x & 0xFF);
        sum += (uint8_t)((x >> 8) & 0xFF);
        sum += (uint8_t)(y & 0xFF);
        sum += (uint8_t)((y >> 8) & 0xFF);
    }

    UART7_SendByte(sum);
    UART7_SendByte(FRAME_TAIL1);
    UART7_SendByte(FRAME_TAIL2);
    xSemaphoreGive(s_uart7_tx_mutex);
}

/* ================= 命令槽取用与应答（03/05/06 通用） =================
   get：MotorTask 取走新命令（先清 ready 再取值，ISR 新帧会重新置 ready）。
   last：LCD 只读最后一条（无清零）。
   ack：MotorTask 收到命令后按原命令字+载荷回显应答帧（03->04 / 05->05 / 06->06），
        串口助手看到应答帧即确认"链路+解析+执行"全通。任务上下文调用。 */
uint8_t cmd_get(ZbeeCmd *c)
{
    taskENTER_CRITICAL();          /*检查+清除+读出原子化: 防ISR中途写入新命令撕裂*/
    if (!g_cmd.ready)
    {
        taskEXIT_CRITICAL();
        return 0;
    }
    g_cmd.ready = 0;
    c->type  = g_cmd.type;
    c->dir   = g_cmd.dir;
    c->sub   = g_cmd.sub;
    c->param = g_cmd.param;
    taskEXIT_CRITICAL();
    return 1;
}

void cmd_last(ZbeeCmd *c, uint8_t *cnt)
{
    taskENTER_CRITICAL();
    c->type  = g_cmd.type;
    c->dir   = g_cmd.dir;
    c->sub   = g_cmd.sub;
    c->param = g_cmd.param;
    *cnt     = g_cmd.cnt;
    taskEXIT_CRITICAL();
}

void cmd_tx_ack(const ZbeeCmd *c)
{
    uint8_t pl[9];
    uint8_t plen = 0;
    uint8_t sum;
    uint8_t i;
    uint32_t p = c->param;

    switch (c->type)
    {
    case CMD_PULSE:
    case CMD_PULSE_ACK:
    case CMD_POS_GO:
        pl[0] = (c->type == CMD_PULSE) ? c->dir : c->sub;   /*03=方向; 05=子命令*/
        pl[1] = (uint8_t)(p & 0xFF);
        pl[2] = (uint8_t)((p >> 8) & 0xFF);
        pl[3] = (uint8_t)((p >> 16) & 0xFF);
        pl[4] = (uint8_t)((p >> 24) & 0xFF);
        plen = 5;
        break;
    case CMD_CHAIN:
        pl[0] = c->dir;                                     /*1=右前45° 4=左前45°*/
        pl[1] = (uint8_t)(p & 0xFF);
        pl[2] = (uint8_t)((p >> 8) & 0xFF);
        pl[3] = (uint8_t)((p >> 16) & 0xFF);
        pl[4] = (uint8_t)((p >> 24) & 0xFF);
        pl[5] = (uint8_t)(c->param2 & 0xFF);
        pl[6] = (uint8_t)((c->param2 >> 8) & 0xFF);
        pl[7] = (uint8_t)((c->param2 >> 16) & 0xFF);
        pl[8] = (uint8_t)((c->param2 >> 24) & 0xFF);
        plen = 9;
        break;
    case CMD_POS_ABORT:
        plen = 0;                    /*06中止帧无载荷*/
        break;
    default:
        return;
    }

    sum = c->type;
    for (i = 0; i < plen; i++) sum += pl[i];

    if (s_uart7_tx_mutex == 0) return;
    if (xSemaphoreTake(s_uart7_tx_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;  /*回传长帧在发*/

    UART7_SendByte(FRAME_HEAD1);
    UART7_SendByte(FRAME_HEAD2);
    UART7_SendByte(c->type);
    for (i = 0; i < plen; i++) UART7_SendByte(pl[i]);
    UART7_SendByte(sum);
    UART7_SendByte(FRAME_TAIL1);
    UART7_SendByte(FRAME_TAIL2);
    xSemaphoreGive(s_uart7_tx_mutex);
}
