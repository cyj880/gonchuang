#include "FreeRTOS.h"
#include "task.h"
#include "LCD.h"
#include "route_rx.h"
#include "uart8.h"
#include "odom_task.h"
#include "lunqu_imu.h"
#include <string.h>
#include <stdio.h>

/*
 * LCD 显示任务（1.8寸 TFT-LCD，ST7735S，128x160）
 *
 * 单页整刷显示（取消三页轮显，PB2 仍可手动切任务码/里程计页）：
 *   收到 UART7（上位机）路线帧后：
 *     1) 整帧原样入回传FIFO，LCD_Task 循环排空逐帧发回网页（右上回传板显示）；
 *     2) LCD 整页一次刷新：B1/B2 行前缀列出全部关键点坐标（最多 19 行，超出提示 more）。
 *   旧 route_tx_uplink(按 02 帧回传) 保留备用, 现回传走 route_echo_flush 逐帧原样回传。
 *
 * 数据来源：坐标 = UART7（上位机）；任务码 = UART8（串口屏）。
 * 引脚（A板）：PB3=SCK  PA7=SDA  PB9=DC  PB0=RES  PA6=CS  PI2=BL
 */

#define LCD_LIST_Y          8       /*坐标列表起始 y（页头占 y=0，19 行到 y=152）*/
#define LCD_ROUTE_ROWS      19      /*一页可显示的坐标行数（128x160，6X8 字体）*/

#define LCD_PAGE_ROUTE      0
#define LCD_PAGE_CODE       1
#define LCD_PAGE_ODOM       2
#define LCD_PAGE_BOOT       LCD_PAGE_ODOM   /*上电默认页: 调试期=里程计页, 比赛改回LCD_PAGE_ROUTE*/

#define LCD_ODOM_PERIOD_MS  100     /*ODOM调试页刷新周期：100ms*/

typedef struct
{
    uint8_t  batch;
    uint8_t  count;
    uint16_t x[ROUTE_MAX_PT];
    uint16_t y[ROUTE_MAX_PT];
} LcdRouteSnap;

static LcdRouteSnap        s_snap[2];   /* [0]=批1 [1]=批2 */
static volatile uint8_t    s_new[2];    /* 有新帧待显示 */
static volatile uint8_t    s_page   = LCD_PAGE_BOOT;
static volatile uint8_t    s_switch = 0;
static int16_t             s_odom_y = -1;    /*ROUTE 页里程计摘要行的 y（-1=无位置）*/
static uint8_t             s_odom_frame = 0; /*ODOM 页骨架是否已画（防 100ms 清屏闪烁）*/

/* 页面手动切换（key_task 的 PB2 回调仍可调用：切任务码/里程计页看调试信息） */
void LCD_TogglePage(void)
{
    s_page   = (uint8_t)((s_page + 1) % 3);     /*三页轮换：ROUTE->CODE->ODOM*/
    s_switch = 1;
}

/* 快照一帧（临界区保护，LCD_ShowRoute 与其他任务均可安全调用） */
void LCD_ShowRoute(const RouteFrame *f)
{
    LcdRouteSnap *s;

    if (f == NULL || f->batch < 1 || f->batch > 2) return;

    s = &s_snap[f->batch - 1];

    taskENTER_CRITICAL();
    s->batch = f->batch;
    s->count = f->count;
    memcpy(s->x, f->x, sizeof(s->x));
    memcpy(s->y, f->y, sizeof(s->y));
    taskEXIT_CRITICAL();

    s_new[f->batch - 1] = 1;
}

/* ================= 收帧处理：快照上屏（回传已改为逐帧原样回传, 见 route_rx.c 的 echo_flush） =================
   旧 route_tx_uplink(按 02 帧回传) 保留在 route_rx.c 备用, 现调用点已移除 */

/* ================= 坐标页：一页整刷列出全部关键点（B1/B2 行前缀区分批次） ================= */

static void lcd_draw_route(void)
{
    uint8_t row = 1, b, i;                       /*row=0 为页头*/

    LCD_SetColor(LCD_CYAN);
    LCD_Printf(0, 0, LCD_6X8, "ROUTE B1:%u B2:%u",
               (unsigned)s_snap[0].count, (unsigned)s_snap[1].count);

    if (s_snap[0].count == 0 && s_snap[1].count == 0)
    {
        LCD_SetColor(LCD_LGRAY);
        LCD_Printf(0, LCD_LIST_Y, LCD_6X8, "wait UART7 coords...");
        LCD_Printf(0, 152, LCD_6X8, "uplink->web ok");
        return;
    }

    for (b = 0; b < 2; b++)
    {
        for (i = 0; i < s_snap[b].count; i++)
        {
            if (row >= LCD_ROUTE_ROWS)           /*一页放不下：底部提示*/
            {
                LCD_SetColor(LCD_LGRAY);
                LCD_Printf(0, 152, LCD_6X8, "...more(PB2:odom)");
                return;
            }
            LCD_SetColor(b == 0 ? LCD_WHITE : LCD_YELLOW);   /*批1白 / 批2黄*/
            LCD_Printf(0, (int16_t)(LCD_LIST_Y + 8 * (row - 1)), LCD_6X8,
                       "B%uP%02u:(%u,%u)",
                       (unsigned)(b + 1), (unsigned)(i + 1),
                       (unsigned)s_snap[b].x[i], (unsigned)s_snap[b].y[i]);
            row++;
        }
    }

    /*底部一行：里程计摘要（坐标不满页时显示，实时刷新该行；完整调试页按 PB2 切换）*/
    if (row <= LCD_ROUTE_ROWS)
    {
        OdomData_t o;
        odometry_get(&o);
        s_odom_y = (int16_t)(LCD_LIST_Y + 8 * (row - 1));
        LCD_SetColor(LCD_GREEN);
    LCD_Printf(0, s_odom_y, LCD_6X8,
               "X%d Y%d y%d   ", (int)o.x_mm, (int)o.y_mm, (int)IMU_GetYaw());
    }
    else s_odom_y = -1;
}

/* ================= 页 1：任务码（串口屏） ================= */

static void lcd_draw_code(void)
{
    char     g0[4], g1[4], g2[4], g3[4];
    uint8_t  y = 0;

    LCD_SetColor(LCD_CYAN);
    LCD_Printf(0, 0, LCD_6X8, "== CODE+CAM (UART8) ==");

    if (!UART8_Valid())
    {
        LCD_SetColor(LCD_LGRAY);
        LCD_Printf(0, (int16_t)(y + 24), LCD_8X16, "no code");
        LCD_Printf(0, (int16_t)(y + 48), LCD_6X8, "wait UART8...");
        LCD_SetColor(LCD_CYAN);
        LCD_Printf(0, (int16_t)(y + 64), LCD_6X8, "frm=%u bad=%u",
                   (unsigned)UART8_CamFrames(), (unsigned)UART8_CamBadFrames());
        return;
    }

    UART8_GetGroup(0, g0);
    UART8_GetGroup(1, g1);
    UART8_GetGroup(2, g2);
    UART8_GetGroup(3, g3);

    /*12 位数字分两行大字显示（坏码标红）*/
    LCD_SetColor(g_link.bad ? LCD_RED : LCD_WHITE);
    LCD_Printf(0, (int16_t)(y + 14), LCD_8X16, "%s + %s", g0, g1);
    LCD_Printf(0, (int16_t)(y + 32), LCD_8X16, "%s + %s", g2, g3);

    /*两批颜色（英文缩写）*/
    LCD_SetColor(LCD_WHITE);
    LCD_Printf(0, (int16_t)(y + 54), LCD_6X8, "B1 %s %s %s",
               UART8_ColorName(UART8_Color(1, 0)),
               UART8_ColorName(UART8_Color(1, 1)),
               UART8_ColorName(UART8_Color(1, 2)));
    LCD_Printf(0, (int16_t)(y + 62), LCD_6X8, "B2 %s %s %s",
               UART8_ColorName(UART8_Color(2, 0)),
               UART8_ColorName(UART8_Color(2, 1)),
               UART8_ColorName(UART8_Color(2, 2)));

    /*环号：第 1 行=第一批(粗加工区=暂存区)，第 2 行=第二批粗加工区*/
    LCD_SetColor(LCD_LGRAY);
    LCD_Printf(0, (int16_t)(y + 74), LCD_6X8, "R1 %d %d %d",
               UART8_Ring(1, 0), UART8_Ring(1, 1), UART8_Ring(1, 2));
    LCD_Printf(0, (int16_t)(y + 82), LCD_6X8, "R2 %d %d %d",
               UART8_Ring(2, 0), UART8_Ring(2, 1), UART8_Ring(2, 2));

    /*第二批在暂存区码垛到几号环，-1 = 第一批没有同色物料（没法码垛）*/
    LCD_SetColor(LCD_YELLOW);
    LCD_Printf(0, (int16_t)(y + 94), LCD_6X8, "stack %d %d %d",
               UART8_StackRing(0), UART8_StackRing(1), UART8_StackRing(2));

    /*本端上报给显示屏的搬运信息*/
    LCD_SetColor(LCD_WHITE);
    LCD_Printf(0, (int16_t)(y + 110), LCD_6X8, "grab %d/6  place %d/6",
               g_link_stat.grab, g_link_stat.place);
    LCD_Printf(0, (int16_t)(y + 118), LCD_6X8, "load %d/3  seq %u",
               g_link_stat.load, (unsigned)g_link.seq);

    /*摄像头链路：码中心横坐标 x / 偏差(正=偏右) / 有效帧数 / 坏帧数*/
    LCD_SetColor(LCD_CYAN);
    if (UART8_CamHasTarget())
        LCD_Printf(0, (int16_t)(y + 132), LCD_6X8, "cam x=%u err=%d",
                   (unsigned)UART8_CamX(), UART8_CamError());
    else
        LCD_Printf(0, (int16_t)(y + 132), LCD_6X8, "cam x=---- no QR");
    LCD_SetColor(LCD_LGRAY);
    LCD_Printf(0, (int16_t)(y + 140), LCD_6X8, "frm=%u bad=%u",
               (unsigned)UART8_CamFrames(), (unsigned)UART8_CamBadFrames());
}

/* ================= 页 2：编码器里程计调试（TIM4 压地测量轮） =================
   切页时 lcd_draw_odom() 画一次骨架；之后 100ms 只由 lcd_update_odom()
   重写数值行（固定宽度+尾空格覆盖旧值），不做全屏清除，避免闪烁。 */

/* ODOM页 cmd 行：最近收到的脉冲命令(方向/脉冲数/累计帧数)，收到新帧即变化 */
static void lcd_odom_cmd_line(void)
{
    ZbeeCmd c;
    uint8_t ccnt;

    cmd_last(&c, &ccnt);

    LCD_SetColor(LCD_YELLOW);
    LCD_Printf(0, 64, LCD_6X8, "c%02X %-9lu #%-2u", c.type, c.param, ccnt);
}

/* ODOM页 CAM 行：MaixCam 的二维码中心横坐标 x + 偏差(x-320，正=码偏右)
   画面里没有码时显示 noQR 和已经收到的有效帧数(帧数在涨说明摄像头链路是通的) */
static void lcd_odom_cam_line(void)
{
    if (UART8_CamHasTarget())
    {
        LCD_SetColor(LCD_CYAN);
        LCD_Printf(0, 98, LCD_6X8, "CAM x=%3u e=%+4d  ",
                   (unsigned)UART8_CamX(), UART8_CamError());
    }
    else
    {
        LCD_SetColor(LCD_LGRAY);
        LCD_Printf(0, 98, LCD_6X8, "CAM noQR f=%-5u ",
                   (unsigned)UART8_CamFrames());
    }
}

static void lcd_draw_odom(void)
{
    OdomData_t o;

    odometry_get(&o);

    LCD_SetColor(LCD_CYAN);
    LCD_Printf(0, 0, LCD_6X8, "== ODOM (RS485) ==");

    /*主显: X/Y双测量轮直读坐标(站1=X 站2=Y, 绝对值无累计误差)*/
    LCD_SetColor(LCD_WHITE);
    LCD_Printf(0, 14, LCD_6X8, "X: %-9d mm", (int)o.x_mm);
    LCD_SetColor(LCD_GREEN);
    LCD_Printf(0, 26, LCD_6X8, "Y: %-9d mm", (int)o.y_mm);

    LCD_SetColor(LCD_WHITE);
    LCD_Printf(0, 42, LCD_6X8, "p1:%-10d p2:%-9d", (int)o.enc_pos[0], (int)o.enc_pos[1]);
    LCD_SetColor(LCD_CYAN);
    LCD_Printf(0, 52, LCD_6X8, "E1:%s E2:%s", o.online[0] ? "ON" : "OFF", o.online[1] ? "ON" : "OFF");

    lcd_odom_cmd_line();

    LCD_SetColor(LCD_WHITE);
    LCD_Printf(0, 76, LCD_6X8, "yaw %-6d", (int)IMU_GetYaw());

    LCD_SetColor(LCD_LGRAY);
    LCD_Printf(0, 110, LCD_6X8, "cnt 0");

    lcd_odom_cam_line();            /*MaixCam 坐标*/
}

/* 100ms 数值行刷新：等宽字体，格式左对齐补空格，覆盖上一次的旧数字 */
static void lcd_update_odom(void)
{
    OdomData_t o;

    odometry_get(&o);

    LCD_SetColor(LCD_WHITE);
    LCD_Printf(0, 14, LCD_6X8, "X: %-9d mm", (int)o.x_mm);
    LCD_SetColor(LCD_GREEN);
    LCD_Printf(0, 26, LCD_6X8, "Y: %-9d mm", (int)o.y_mm);
    LCD_SetColor(LCD_WHITE);
    LCD_Printf(0, 42, LCD_6X8, "p1:%-10d p2:%-9d", (int)o.enc_pos[0], (int)o.enc_pos[1]);
    LCD_SetColor(LCD_CYAN);
    LCD_Printf(0, 52, LCD_6X8, "E1:%s E2:%s", o.online[0] ? "ON" : "OFF", o.online[1] ? "ON" : "OFF");
    lcd_odom_cmd_line();
    LCD_Printf(0, 76, LCD_6X8, "yaw %-6d", (int)IMU_GetYaw());

    lcd_odom_cam_line();            /*MaixCam 坐标*/
}

/* ================= 任务主体 ================= */

void LCD_Task(void *pvParameters)
{
    uint8_t    drawn_page = 0xFF;          /*0xFF=强制首帧重画(上电直接进默认页)*/
    char       code_now[16];
    char       code_last[16] = "";
    uint8_t    stat_last[3]  = {0xFF, 0xFF, 0xFF};
    uint8_t    need_draw;
    TickType_t odom_next;

    (void)pvParameters;

    LCD_SetBackgroundColor(LCD_BLACK);
    LCD_SetColor(LCD_WHITE);

    LCD_Clear();
    LCD_SetColor(LCD_WHITE);
    LCD_Printf(0, 0, LCD_8X16, "Hello");            /*上电待机画面*/
    LCD_SetColor(LCD_LGRAY);
    LCD_Printf(0, 16, LCD_6X8, "Wait UART7/UART8...");

    odom_next = xTaskGetTickCount() + pdMS_TO_TICKS(LCD_ODOM_PERIOD_MS);

    for ( ;; )
    {
        /*—— 协议维护：帧中途超时复位 + 回传FIFO排空(校验通过的帧逐帧原样发回网页) ——*/
        route_rx_poll();
        route_echo_flush();

        /*—— 轮询收帧：快照供 LCD 坐标页显示 ——*/
        if (route_ready(1))
        {
            LCD_ShowRoute(route_get(1));
            route_clear_ready(1);
        }
        if (route_ready(2))
        {
            LCD_ShowRoute(route_get(2));
            route_clear_ready(2);
        }

        need_draw = 0;

        if (s_page != drawn_page || s_switch)       /*切页了：整页重画*/
        {
            s_switch   = 0;
            drawn_page = s_page;
            LCD_ClearArea(0, 0, LCD_W, LCD_H);
            need_draw = 1;
            if (s_page == LCD_PAGE_ROUTE)
            {
                s_new[0] = 0;                       /*切回坐标页：新帧已随重画上屏*/
                s_new[1] = 0;
            }
            if (s_page == LCD_PAGE_ODOM)
            {
                odom_next    = xTaskGetTickCount() + pdMS_TO_TICKS(LCD_ODOM_PERIOD_MS);
                s_odom_frame = 1;                   /*骨架由本次 need_draw 的 lcd_draw_odom 画*/
            }
            if (drawn_page == LCD_PAGE_ODOM)
            {
                s_odom_frame = 0;                   /*从 ODOM 切走：下次进入重画骨架*/
            }
        }
        else if (s_page == LCD_PAGE_ROUTE)
        {
            if (s_new[0] || s_new[1])               /*停留期间来新坐标帧*/
            {
                s_new[0] = 0;
                s_new[1] = 0;
                LCD_ClearArea(0, 0, LCD_W, LCD_H);
                need_draw = 1;
            }
            else if (s_odom_y >= 0 &&               /*停留期间：200ms 刷新底部里程计摘要行*/
                     (int32_t)(xTaskGetTickCount() - odom_next) >= 0)
            {
                OdomData_t o;
                odometry_get(&o);
                odom_next = xTaskGetTickCount() + pdMS_TO_TICKS(200);
                LCD_SetColor(LCD_GREEN);
                LCD_Printf(0, s_odom_y, LCD_6X8,
                           "X%d Y%d y%d   ", (int)o.x_mm, (int)o.y_mm, (int)o.yaw_deg);
            }
        }
        else if (s_page == LCD_PAGE_CODE)           /*CODE 页：码或统计变了才重画*/
        {
            UART8_GetCode(code_now);
            if (strcmp(code_now, code_last) != 0 ||
                g_link_stat.grab  != stat_last[0] ||
                g_link_stat.place != stat_last[1] ||
                g_link_stat.load  != stat_last[2])
            {
                strcpy(code_last, code_now);
                stat_last[0] = g_link_stat.grab;
                stat_last[1] = g_link_stat.place;
                stat_last[2] = g_link_stat.load;
                LCD_ClearArea(0, 0, LCD_W, LCD_H);
                need_draw = 1;
            }
        }
        else                                        /*ODOM 页：骨架画一次，之后只刷数值行*/
        {
            if ((int32_t)(xTaskGetTickCount() - odom_next) >= 0)
            {
                odom_next += pdMS_TO_TICKS(LCD_ODOM_PERIOD_MS);
                if (!s_odom_frame)
                {
                    LCD_ClearArea(0, 0, LCD_W, LCD_H);
                    lcd_draw_odom();
                    s_odom_frame = 1;
                }
                else lcd_update_odom();
            }
        }

        if (need_draw)
        {
            if (s_page == LCD_PAGE_ROUTE)      lcd_draw_route();
            else if (s_page == LCD_PAGE_CODE)  lcd_draw_code();
            else                               lcd_draw_odom();
        }

        vTaskDelay(pdMS_TO_TICKS(10));              /*10ms：兼顾 UART8 回传响应速度*/
    }
}
