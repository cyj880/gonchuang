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
 * 只显示 ODOM 里程计主页面（单页常显，已取消多页轮显）：
 *   X/Y 坐标、双编码器原始值、通信状态、最近命令、yaw、MaixCam 状态。
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
    if (c.type == CMD_POS_GO || c.type == CMD_POS_ADV || c.type == CMD_CHAIN)
        LCD_Printf(0, 64, LCD_6X8, "c%02X %-9ld #%-2u", c.type, (long)(int32_t)c.param, ccnt);
    else
        LCD_Printf(0, 64, LCD_6X8, "c%02X %-9lu #%-2u", c.type, (unsigned long)c.param, ccnt);
}

/* CAM 行：MaixCam 的二维码中心横坐标 x + 偏差(x-320，正=码偏右)
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
    LCD_Printf(0, 76, LCD_6X8, "yaw %-6d", (int)IMU_GetYawAbs360());   /*0~360, 零点=上电/按PB2*/

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
    LCD_Printf(0, 76, LCD_6X8, "yaw %-6d", (int)IMU_GetYawAbs360());   /*0~360, 零点=上电/按PB2*/
    lcd_odom_cam_line();            /*MaixCam 坐标*/
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
