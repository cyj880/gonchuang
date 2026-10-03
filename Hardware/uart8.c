#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uart8.h"
#include <string.h>

/**
  * UART8 —— 摄像头链路 + 任务码显示屏转发口
  *   PE0 = RX ← MaixCam 的 TX：收二维码内容(任务码) + 码中心横坐标 x
  *   PE1 = TX → 任务码显示屏的 USART2_RX：转发任务码 + 搬运信息
  * 接收在 UART8_IRQHandler 里逐字节喂进 UART8_RxByte()。
  *
 * 摄像头帧（AA 55 | LEN | 内容 | x低 | x高 | SUM）解析成两样东西：
 *   1) 内容里的数字 -> 任务码（集满 12 位即发布）
 *   2) x            -> 二维码中心横坐标，供横向纠偏
 * 摄像头扫到码才发、没码时静默；车端用 CAM_FRESH_MS 内有没有新帧判目标在不在。
 * 校验和不对的帧直接丢弃，不会污染任务码。
  * 另外兼容"摄像头直接发纯文本内容"的情况（没有 AA 55 帧头时按 ASCII 解析）。
  */

TaskLinkCode g_link;
TaskLinkStat g_link_stat;

/* ---------------- 摄像头帧解析状态 ---------------- */
#define CAM_ST_H1   0       /* 找 0xAA */
#define CAM_ST_H2   1       /* 找 0x55 */
#define CAM_ST_LEN  2       /* 读内容长度 */
#define CAM_ST_PAY  3       /* 读内容 */
#define CAM_ST_XL   4       /* 读 x 低字节 */
#define CAM_ST_XH   5       /* 读 x 高字节 */
#define CAM_ST_SUM  6       /* 读校验和 */

static uint8_t  cam_st;
static uint8_t  cam_len;                    /* 本帧内容长度 */
static uint8_t  cam_idx;
static uint8_t  cam_buf[CAM_PAY_MAX];       /* 本帧内容 */
static uint16_t cam_x;                      /* 本帧 x */
static uint8_t  cam_sum;                    /* 累加校验 */

static volatile uint16_t g_cam_x      = CAM_X_LOST;
static volatile uint8_t  g_cam_ok     = 0;
static volatile uint16_t g_cam_frames = 0;
static volatile uint16_t g_cam_bad    = 0;
static volatile TickType_t g_cam_last_tick = 0;   /* 最近一帧有效帧的节拍,判"码还在不在"用 */

/* ---------------- 任务码解析 ---------------- */
static uint8_t  rx_dig[TASK_LINK_DIGITS];
static uint8_t  rx_cnt;
static uint8_t  rx_last[TASK_LINK_DIGITS];  /* 上一次已发布的码，用来判重 */
static uint8_t  rx_have_last;

/* ==================================================================
   一、串口本身
   ================================================================== */

void UART8_Init(uint32_t baud)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /*开启时钟*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART8, ENABLE);        //UART8挂APB1
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    /*GPIO：PE0/PE1 复用推挽，映射到 UART8*/
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource0, GPIO_AF_UART8);     //PE0 = UART8_RX（接摄像头 TX）
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource1, GPIO_AF_UART8);     //PE1 = UART8_TX（接显示屏 RX）
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                 //RX上拉，防止悬空乱码
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    /*USART：收发模式（Mode_Rx | Mode_Tx），115200-8-N-1*/
    USART_InitStructure.USART_BaudRate = baud;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART8, &USART_InitStructure);

    /*NVIC：抢占优先级6（数值≥5，落在 configMAX_SYSCALL_INTERRUPT_PRIORITY
      管辖范围内，之后若要在中断里调 FreeRTOS FromISR API 也安全）*/
    NVIC_InitStructure.NVIC_IRQChannel = UART8_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(UART8, USART_IT_RXNE, ENABLE);                //开接收中断
    USART_Cmd(UART8, ENABLE);
}

/* 阻塞发送一个字节（带超时保护，避免异常时死等） */
void UART8_SendByte(uint8_t byte)
{
    uint32_t timeout = 100000;

    USART_SendData(UART8, byte);
    while (USART_GetFlagStatus(UART8, USART_FLAG_TXE) == RESET)
    {
        if (--timeout == 0) break;
    }
}

/* 阻塞发送字符串 */
void UART8_SendString(const char *str)
{
    while (*str)
    {
        UART8_SendByte((uint8_t)*str++);
    }
}

/* UART8 接收中断：逐字节喂入解析器 */
void UART8_IRQHandler(void)
{
    if (USART_GetITStatus(UART8, USART_IT_RXNE) != RESET)
    {
        UART8_RxByte((uint8_t)USART_ReceiveData(UART8));         //读DR同时清RXNE
    }

    if (USART_GetFlagStatus(UART8, USART_FLAG_ORE) != RESET)     //溢出错误清除：读SR再读DR
    {
        USART_ReceiveData(UART8);
    }
}

/* ==================================================================
   二、任务码解析
   ================================================================== */

/* 一帧 12 位数字发布为任务码 */
static void Code_Publish(void)
{
    uint8_t i, same = 1;

    for (i = 0; i < TASK_LINK_DIGITS; i++)
    {
        g_link.digit[i] = rx_dig[i];
        if (!rx_have_last || rx_last[i] != rx_dig[i]) same = 0;
        rx_last[i] = rx_dig[i];
    }
    rx_have_last = 1;

    /* 组1/组3 -> 颜色；组2/组4 -> 环号 */
    for (i = 0; i < 3; i++)
    {
        g_link.color[0][i] = (uint8_t)(g_link.digit[i] - '0');
        g_link.ring[0][i]  = (uint8_t)(g_link.digit[3 + i] - '0');
        g_link.color[1][i] = (uint8_t)(g_link.digit[6 + i] - '0');
        g_link.ring[1][i]  = (uint8_t)(g_link.digit[9 + i] - '0');
    }

    g_link.bad = 0;
    for (i = 0; i < TASK_LINK_DIGITS; i++)
    {
        if (g_link.digit[i] < '1' || g_link.digit[i] > '6') g_link.bad = 1;
    }

    g_link.valid = 1;
    g_link.seq++;
    if (!same) g_link.changed = 1;              /* 只有内容变了才算"新码" */
}

/* 只累计数字：其它可见字符当分隔符忽略，控制字符清空候选 */
static void Code_Feed(uint8_t ch)
{
    if (ch >= '0' && ch <= '9')
    {
        if (rx_cnt < TASK_LINK_DIGITS)
        {
            rx_dig[rx_cnt++] = ch;
            if (rx_cnt >= TASK_LINK_DIGITS)
            {
                Code_Publish();
                rx_cnt = 0;                     /* 允许紧接着又来一帧 */
            }
        }
    }
    else if (ch < 0x20)
    {
        rx_cnt = 0;
    }
}

/* 逐字节解析摄像头数据（中断里调用，不要加阻塞操作） */
void UART8_RxByte(uint8_t ch)
{
    switch (cam_st)
    {
    case CAM_ST_H1:
        if (ch == 0xAA) cam_st = CAM_ST_H2;
        else            Code_Feed(ch);           /* 兼容纯文本内容 */
        break;

    case CAM_ST_H2:
        if (ch == 0x55)
        {
            cam_st = CAM_ST_LEN;
        }
        else
        {
            cam_st = (ch == 0xAA) ? CAM_ST_H2 : CAM_ST_H1;   /* 允许 AA AA 55 这种重叠 */
            Code_Feed(ch);
        }
        break;

    case CAM_ST_LEN:
        cam_len = ch;
        cam_sum = ch;                            /* 校验和从长度字节开始累加 */
        cam_idx = 0;
        if (cam_len > CAM_PAY_MAX)               /* 长度不合理，丢帧重新同步 */
        {
            g_cam_bad++;
            cam_st = CAM_ST_H1;
        }
        else if (cam_len == 0)                   /* ★心跳帧(画面里没有码): 没有内容, 直接去读 x */
        {
            cam_st = CAM_ST_XL;
        }
        else
        {
            cam_st = CAM_ST_PAY;
        }
        break;

    case CAM_ST_PAY:
        /* 内容里又冒出 AA 55：说明上一帧的长度字节被干扰带偏了(比如上电时收到半帧)，
           直接按"这一帧从这里重新开始"处理，下一字节当作长度，立刻恢复同步 */
        if (cam_idx > 0 && cam_buf[cam_idx - 1] == 0xAA && ch == 0x55)
        {
            g_cam_bad++;
            cam_st = CAM_ST_LEN;
            break;
        }
        cam_buf[cam_idx++] = ch;
        cam_sum += ch;
        if (cam_idx >= cam_len) cam_st = CAM_ST_XL;
        break;

    case CAM_ST_XL:
        cam_x = ch;
        cam_sum += ch;
        cam_st = CAM_ST_XH;
        break;

    case CAM_ST_XH:
        cam_x |= (uint16_t)ch << 8;
        cam_sum += ch;
        cam_st = CAM_ST_SUM;
        break;

    case CAM_ST_SUM:
        if (ch == cam_sum)                       /* 校验通过，帧有效 */
        {
            uint8_t i;

            g_cam_frames++;
            /* 一帧内容就是一个完整二维码，先把候选清空，
               避免上次上电/丢帧时残留的半个码和新码拼成错码 */
            rx_cnt = 0;
            for (i = 0; i < cam_len; i++) Code_Feed(cam_buf[i]);   /* 内容里的数字 -> 任务码 */
            g_cam_x  = cam_x;                                      /* 心跳帧 LEN=0,x=0xFFFF(现摄像头已不发,兼容保留) */
            g_cam_ok = (cam_x == CAM_X_LOST) ? 0 : 1;
            g_cam_last_tick = xTaskGetTickCountFromISR();
        }
        else
        {
            g_cam_bad++;
        }
        cam_st = CAM_ST_H1;
        break;

    default:
        cam_st = CAM_ST_H1;
        break;
    }
}

/* 初始化：解析器与统计清零 */
void UART8_LinkInit(void)
{
    uint8_t i;

    cam_st  = CAM_ST_H1;
    cam_len = 0;
    cam_idx = 0;
    cam_x   = CAM_X_LOST;
    cam_sum = 0;

    g_cam_x      = CAM_X_LOST;
    g_cam_ok     = 0;
    g_cam_frames = 0;
    g_cam_bad    = 0;
    g_cam_last_tick = 0;

    rx_cnt = 0;
    rx_have_last = 0;

    g_link.valid   = 0;
    g_link.bad     = 0;
    g_link.changed = 0;
    g_link.seq     = 0;
    for (i = 0; i < TASK_LINK_DIGITS; i++)
    {
        g_link.digit[i] = '0';
        rx_last[i] = 0;
    }
    for (i = 0; i < 3; i++)
    {
        g_link.color[0][i] = 0; g_link.ring[0][i] = 0;
        g_link.color[1][i] = 0; g_link.ring[1][i] = 0;
    }

    g_link_stat.grab  = 0;
    g_link_stat.place = 0;
    g_link_stat.load  = 0;
}

/* ==================================================================
   三、任务码查询
   ================================================================== */
uint8_t  UART8_Valid(void)        { return g_link.valid; }
uint8_t  UART8_NewCode(void)      { return g_link.changed; }
void     UART8_ClearChanged(void) { g_link.changed = 0; }
uint16_t UART8_Seq(void)          { return g_link.seq; }

void UART8_GetCode(char *out)
{
    uint8_t i, k = 0;

    for (i = 0; i < TASK_LINK_DIGITS; i++)
    {
        out[k++] = (char)g_link.digit[i];
        if (i == 2 || i == 5 || i == 8) out[k++] = '+';
    }
    out[k] = '\0';
}

void UART8_GetGroup(uint8_t g, char *out)
{
    uint8_t i;

    if (g >= TASK_LINK_GROUPS)
    {
        out[0] = '\0';
        return;
    }
    for (i = 0; i < TASK_LINK_GLEN; i++) out[i] = (char)g_link.digit[g * TASK_LINK_GLEN + i];
    out[TASK_LINK_GLEN] = '\0';
}

uint8_t UART8_Color(uint8_t batch, uint8_t idx)
{
    if (batch < 1 || batch > 2 || idx > 2) return 0;
    return g_link.color[batch - 1][idx];
}

uint8_t UART8_Ring(uint8_t batch, uint8_t idx)
{
    if (batch < 1 || batch > 2 || idx > 2) return 0;
    return g_link.ring[batch - 1][idx];
}

/* 第一批：粗加工区和暂存区用同一个环号 */
uint8_t UART8_StoreRing(uint8_t idx)
{
    if (idx > 2) return 0;
    return g_link.ring[0][idx];
}

/* 第二批第 idx 件在暂存区要码垛到"同色第一批物料"所在的那个环上；
   第一批里没有同色物料时返回 -1（该件无法码垛计分） */
int8_t UART8_StackRing(uint8_t idx)
{
    uint8_t c, j;

    if (idx > 2) return -1;
    c = g_link.color[1][idx];
    for (j = 0; j < 3; j++)
    {
        if (g_link.color[0][j] == c) return (int8_t)g_link.ring[0][j];
    }
    return -1;
}

const char* UART8_ColorName(uint8_t color)
{
    switch (color)
    {
    case 1: return "RED";       /* 红 */
    case 2: return "YEL";       /* 黄 */
    case 3: return "BLU";       /* 蓝 */
    case 4: return "GRN";       /* 绿 */
    case 5: return "BLK";       /* 黑 */
    case 6: return "LBL";       /* 浅蓝 Light Blue */
    default: return "--";
    }
}

/* ==================================================================
   四、摄像头坐标（横向纠偏）
   ================================================================== */
uint16_t UART8_CamX(void)          { return g_cam_x; }
uint16_t UART8_CamFrames(void)     { return g_cam_frames; }
uint16_t UART8_CamBadFrames(void)  { return g_cam_bad; }

/* 画面里有没有码:摄像头扫到才发、没码时静默,所以用"多久没来新帧"判断——
   超过 CAM_FRESH_MS 没收到有效帧即视为无码,避免拿最后一次的旧 x 继续纠偏 */
uint8_t  UART8_CamHasTarget(void)
{
    return g_cam_ok &&
           (xTaskGetTickCount() - g_cam_last_tick) < pdMS_TO_TICKS(CAM_FRESH_MS);
}

/* 偏差 = x - 图像中心 320：正数表示码偏右，机器人应右移/右转；没目标时返回 0 */
int16_t UART8_CamError(void)
{
    if (!UART8_CamHasTarget() || g_cam_x == CAM_X_LOST) return 0;
    return (int16_t)g_cam_x - (int16_t)(CAM_IMG_W / 2);
}

/* ==================================================================
   五、转发给任务码显示屏
   ================================================================== */

/* 发一个 0~99 的十进制数（不依赖 printf，省 Flash） */
static void UART8_SendU8(uint8_t v)
{
    if (v >= 10) UART8_SendByte((uint8_t)('0' + v / 10));
    UART8_SendByte((uint8_t)('0' + v % 10));
}

/* 发一行任务码给显示屏：426+213+432+123 */
void UART8_SendTaskCode(void)
{
    char buf[16];

    if (!g_link.valid) return;

    UART8_GetCode(buf);
    UART8_SendString(buf);
    UART8_SendString("\r\n");
}

/* 立刻发一行状态给显示屏：S,抓取正确,放置正确,当前载物 */
void UART8_SendStat(void)
{
    UART8_SendByte('S');
    UART8_SendByte(',');
    UART8_SendU8(g_link_stat.grab);
    UART8_SendByte(',');
    UART8_SendU8(g_link_stat.place);
    UART8_SendByte(',');
    UART8_SendU8(g_link_stat.load);
    UART8_SendString("\r\n");
}

void UART8_SetGrab(uint8_t n)  { g_link_stat.grab  = (n > 6) ? 6 : n; }
void UART8_SetPlace(uint8_t n) { g_link_stat.place = (n > 6) ? 6 : n; }
void UART8_SetLoad(uint8_t n)  { g_link_stat.load  = (n > 3) ? 3 : n; }

void UART8_GrabOk(void)
{
    if (g_link_stat.grab < 6) g_link_stat.grab++;
}

void UART8_PlaceOk(void)
{
    if (g_link_stat.place < 6) g_link_stat.place++;
}

void UART8_LoadInc(void)
{
    if (g_link_stat.load < 3) g_link_stat.load++;
}

void UART8_LoadDec(void)
{
    if (g_link_stat.load > 0) g_link_stat.load--;
}

void UART8_StatClear(void)
{
    g_link_stat.grab  = 0;
    g_link_stat.place = 0;
    g_link_stat.load  = 0;
    UART8_SendStat();
}

/* ==================================================================
   六、可选任务：只在"内容真的变了"的时候转发，不做周期心跳
     - 任务码内容变化 -> 发一行任务码
     - 抓取/放置/载物 有变化 -> 发一行状态
   （本任务不碰 g_link.changed，上层逻辑照旧可以用
     UART8_NewCode()/UART8_ClearChanged() 判断新码）
   ================================================================== */
void UART8_ReportTask(void *pvParameters)
{
    char     code_now[16];
    char     code_last[16] = "";
    uint8_t  last_g = 0xFF, last_p = 0xFF, last_l = 0xFF;

    (void)pvParameters;

    for (;;)
    {
        /* 1) 任务码内容变了 -> 转发任务码行 */
        UART8_GetCode(code_now);
        if (g_link.valid && strcmp(code_now, code_last) != 0)
        {
            UART8_SendTaskCode();
            strcpy(code_last, code_now);
        }

        /* 2) 搬运信息有变化 -> 转发状态行 */
        if (g_link_stat.grab  != last_g ||
            g_link_stat.place != last_p ||
            g_link_stat.load  != last_l)
        {
            UART8_SendStat();
            last_g = g_link_stat.grab;
            last_p = g_link_stat.place;
            last_l = g_link_stat.load;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
