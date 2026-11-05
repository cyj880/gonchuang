#include "FreeRTOS.h"
#include "task.h"
#include "LCD.h"
#include "route_rx.h"
#include "uart8.h"
#include "odom_task.h"
#include "motor_task.h"
#include "lunqu_imu.h"
#include "rdk_center.h"
#include "ring_align.h"
#include <string.h>
#include <stdio.h>

/*
 * LCD 显示任务（1.8寸 TFT-LCD，ST7735S，128x160）
 *
 * 只显示 ODOM 里程计主页面（单页常显，已取消多页轮显）：
 *   里程计1的里程、速度、原始值、通信状态、最近命令、yaw、MaixCam 状态。
 * 顺带协议维护：帧中途超时复位 + 回传FIFO排空（校验通过的帧逐帧原样发回网页）。
 *
 * 引脚（A板）：PB3=SCK  PA7=SDA  PB9=DC  PB0=RES  PA6=CS  PI2=BL
 */

#define LCD_ODOM_PERIOD_MS  100     /*ODOM主页面刷新周期：100ms*/

void LCD_TogglePage(void)           /*保留接口(key_task PB2回调), 单页版无操作*/
{
}

/* ================= ODOM 主页面 ================= */

/* cmd 行：最近收到的定位/脉冲命令(命令字/参数/累计帧数)，收到新帧即变化 */
static void lcd_odom_cmd_line(void)
{
    ZbeeCmd c;
    uint8_t ccnt;

    cmd_last(&c, &ccnt);
    LCD_SetColor(LCD_YELLOW);
    if (Motor_NoNext())
        LCD_Printf(0, 64, LCD_6X8, "no next              ");
    else if (c.type == CMD_POS_GO || c.type == CMD_POS_ADV || c.type == CMD_CHAIN)
        LCD_Printf(0, 64, LCD_6X8, "c%02X %-9ld #%-2u", c.type, (long)(int32_t)c.param, ccnt);
    else
        LCD_Printf(0, 64, LCD_6X8, "c%02X %-9lu #%-2u", c.type, (unsigned long)c.param, ccnt);
}

/* CAM 行：MaixCam 链路状态（坐标已按需求去掉，只看有没有扫到码）
   画面里没有码时显示 noQR；帧数在涨说明摄像头链路是通的 */
static void lcd_odom_cam_line(void)
{
    if (UART8_CamHasTarget())
    {
        LCD_SetColor(LCD_CYAN);
        LCD_Printf(0, 98, LCD_6X8, "CAM got code f=%-5u",
                   (unsigned)UART8_CamFrames());
    }
    else
    {
        LCD_SetColor(LCD_LGRAY);
        LCD_Printf(0, 98, LCD_6X8, "CAM noQR     f=%-5u",
                   (unsigned)UART8_CamFrames());
    }
}

static void lcd_odom_center_line(void)
{
    RDK_CenterDiagnostics d;
    RDK_Center_GetDiagnostics(&d);
    LCD_SetColor(RDK_CenterValid() ? LCD_GREEN : LCD_LGRAY);
    if (RDK_Ring_Active())
    {
        RDK_RingResult result;
        char line[22];
        uint8_t i;
        RDK_Ring_GetResult(&result);
        LCD_Printf(0, 76, LCD_6X8, "%-21s", RingAlign_State());
        {
            int16_t before, after;
            if (RingAlign_TestYaw(&before, &after))
            {
                LCD_Printf(0, 64, LCD_6X8, "Y:%+6d > %+6d    ", (int)before, (int)after);
            }
        }
        LCD_SetColor(result.fresh ? LCD_GREEN : LCD_YELLOW);
        if (!result.fresh)
            snprintf(line, sizeof(line), "RING %s", result.sequence ? "STALE" : "WAIT");
        else if (!result.count)
            snprintf(line, sizeof(line), "R:0 NONE Y:--");
        else if (!result.yaw_valid)
            snprintf(line, sizeof(line), "R:%u Y:--", (unsigned)result.count);
        else
        {
            int yaw = result.yaw_cdeg < 0 ? -(int)result.yaw_cdeg : result.yaw_cdeg;
            snprintf(line, sizeof(line), "R:%u Y:%c%d.%02d", (unsigned)result.count,
                     result.yaw_cdeg < 0 ? '-' : '+', yaw / 100, yaw % 100);
        }
        LCD_Printf(0, 86, LCD_6X8, "%-21s", line);
        for (i = 0; i < 3; i++)
        {
            if (i < result.count)
                snprintf(line, sizeof(line), "ID%u:%3u,%3u", (unsigned)result.points[i].id,
                         (unsigned)result.points[i].x, (unsigned)result.points[i].y);
            else
                snprintf(line, sizeof(line), "R%u:--,--", (unsigned)i + 1);
            LCD_Printf(0, 98 + 12 * i, LCD_6X8, "%-21s", line);
        }
    }
    else if (RDK_Stack_Active())
    {
        /* 码垛对位页: 状态机阶段 + 0x14帧内容, 便于判断卡在哪一步 */
        RDK_StackResult st;
        char line[22];
        RDK_Stack_GetResult(&st);
        LCD_Printf(0, 76, LCD_6X8, "%-21s", RingAlign_State());
        LCD_SetColor(st.fresh ? LCD_GREEN : LCD_YELLOW);
        if (!st.fresh)
            snprintf(line, sizeof(line), "STK %s", st.sequence ? "STALE" : "WAIT");
        else if (!st.flags)
            snprintf(line, sizeof(line), "STK FLAGS=0 SEEK"); /*RDK没看到工位*/
        else
        {
            int yaw = st.yaw_cdeg < 0 ? -st.yaw_cdeg : st.yaw_cdeg;
            snprintf(line, sizeof(line), "S:%3u,%3u Y:%c%d.%02d",
                     (unsigned)st.x, (unsigned)st.y,
                     st.yaw_cdeg < 0 ? '-' : '+', yaw / 100, yaw % 100);
        }
        LCD_Printf(0, 86, LCD_6X8, "%-21s", line);
        {
            uint8_t tap[8], i;
            char hex[20];
            RDK_Center_LastBytes(tap);
            for (i = 0; i < 7u; i++)
                snprintf(&hex[i * 2], sizeof(hex) - i * 2, "%02X", tap[i]);
            LCD_Printf(0, 98, LCD_6X8, "RAW %-15s", hex);
        }
    }
    else if (RDK_CenterValid())
        LCD_Printf(0, 122, LCD_6X8, "RDK C:%4d,%4d      ", (int)RDK_CenterX(), (int)RDK_CenterY());
    else
        LCD_Printf(0, 122, LCD_6X8, "RDK C:--,--          ");
    LCD_SetColor(LCD_LGRAY);
    LCD_Printf(0, 134, LCD_6X8, "B:%05lu F:%05lu      ",
               (unsigned long)(d.rx_bytes % 100000u),
               (unsigned long)(d.valid_frames % 100000u));
    LCD_Printf(0, 146, LCD_6X8, "G:%u E:%04lu O:%04lu  ",
               (unsigned)d.enabled, (unsigned long)(d.errors % 10000u),
               (unsigned long)(d.overruns % 10000u));
}

static void lcd_draw_odom(void)
{
    OdomData_t o;

    odometry_get(&o);

    LCD_SetColor(LCD_CYAN);
    LCD_Printf(0, 0, LCD_6X8, "== ODOM (RS485) ==");

    /*主显: 站号1里程和速度，不显示不存在的第二测量轮*/
    LCD_SetColor(LCD_WHITE);
    LCD_Printf(0, 14, LCD_6X8, "d1:%-9d mm", (int)o.dist_mm[0]);
    LCD_SetColor(LCD_GREEN);
    LCD_Printf(0, 26, LCD_6X8, "v1:%-9d mm/s", (int)o.speed_mm_s[0]);

    LCD_SetColor(LCD_WHITE);
    LCD_Printf(0, 42, LCD_6X8, "p1:%-10d", (int)o.enc_pos[0]);
    LCD_SetColor(LCD_CYAN);
    LCD_Printf(0, 52, LCD_6X8, "E1:%-3s", o.online[0] ? "ON" : "OFF");

    lcd_odom_cmd_line();

    LCD_SetColor(LCD_WHITE);
    LCD_Printf(0, 76, LCD_6X8, "yaw %-6d", (int)IMU_GetYawAbs360());   /*0~360, 零点=上电/按PB2*/

    if (!RDK_Ring_Active())
    {
        LCD_Printf(0, 86, LCD_6X8, "                     ");
        lcd_odom_cam_line();
        LCD_SetColor(LCD_WHITE);
        LCD_Printf(0, 110, LCD_6X8, "pos:%-17ld", (long)Motor_RoutePos(o.enc_pos[ODOM_POS_AXIS]));
    }
    lcd_odom_center_line();
}

/* 100ms 数值行刷新：等宽字体，格式左对齐补空格，覆盖上一次的旧数字 */
static void lcd_update_odom(void)
{
    OdomData_t o;

    odometry_get(&o);

    LCD_SetColor(LCD_WHITE);
    LCD_Printf(0, 14, LCD_6X8, "d1:%-9d mm", (int)o.dist_mm[0]);
    LCD_SetColor(LCD_GREEN);
    LCD_Printf(0, 26, LCD_6X8, "v1:%-9d mm/s", (int)o.speed_mm_s[0]);
    LCD_SetColor(LCD_WHITE);
    LCD_Printf(0, 42, LCD_6X8, "p1:%-10d", (int)o.enc_pos[0]);
    LCD_SetColor(LCD_CYAN);
    LCD_Printf(0, 52, LCD_6X8, "E1:%-3s", o.online[0] ? "ON" : "OFF");
    lcd_odom_cmd_line();
    LCD_Printf(0, 76, LCD_6X8, "yaw %-6d", (int)IMU_GetYawAbs360());   /*0~360, 零点=上电/按PB2*/
    if (!RDK_Ring_Active())
    {
        LCD_Printf(0, 86, LCD_6X8, "                     ");
        lcd_odom_cam_line();
        LCD_SetColor(LCD_WHITE);
        LCD_Printf(0, 110, LCD_6X8, "pos:%-17ld", (long)Motor_RoutePos(o.enc_pos[ODOM_POS_AXIS]));
    }
    lcd_odom_center_line();
}

/* ================= 任务主体: 只显示 ODOM 主页面 ================= */

void LCD_Task(void *pvParameters)
{
    uint8_t    odom_frame = 0;                     /*骨架是否已画(防100ms清屏闪烁)*/
    TickType_t odom_next;

    (void)pvParameters;

    LCD_SetBackgroundColor(LCD_BLACK);
    LCD_SetColor(LCD_WHITE);

    LCD_Clear();
    LCD_SetColor(LCD_WHITE);
    LCD_Printf(0, 0, LCD_8X16, "Hello");            /*上电待机画面, 首次刷新即被ODOM页覆盖*/
    LCD_SetColor(LCD_LGRAY);
    LCD_Printf(0, 16, LCD_6X8, "Wait UART7/UART8...");

    odom_next = xTaskGetTickCount() + pdMS_TO_TICKS(LCD_ODOM_PERIOD_MS);

    for ( ;; )
    {
        /*—— 协议维护：帧中途超时复位 + 回传FIFO排空(校验通过的帧逐帧原样发回网页) ——*/
        route_rx_poll();
        route_echo_flush();

        /*—— 清路线帧就绪标志(路线页已取消, 数据仍由上位机回传确认) ——*/
        if (route_ready(1)) route_clear_ready(1);
        if (route_ready(2)) route_clear_ready(2);

        /*—— ODOM 主页面: 骨架画一次, 之后 100ms 只刷数值行 ——*/
        if ((int32_t)(xTaskGetTickCount() - odom_next) >= 0)
        {
            odom_next += pdMS_TO_TICKS(LCD_ODOM_PERIOD_MS);
            if (!odom_frame)
            {
                LCD_ClearArea(0, 0, LCD_W, LCD_H);
                lcd_draw_odom();
                odom_frame = 1;
            }
            else
            {
                lcd_update_odom();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));              /*10ms：兼顾 UART8 回传响应速度*/
    }
}
