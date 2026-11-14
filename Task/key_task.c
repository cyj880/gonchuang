#include "FreeRTOS.h"
#include "task.h"
#include "key_task.h"
#include "Key.h"
#include "LED.h"
#include "lcd_task.h"
#include "imu.h"
#include "motor_task.h"

/**
  * 按键任务（源自 STM32F407 例程 key_task.c 移植）
  * 每 10ms 扫描一次 A 板用户按键（PB2，按下为高电平），
  * 检测到松开沿时：切换 LCD 页面（路线坐标页 <-> 任务码页），
  * 同时翻转板载红色指示灯做按键反馈。
  */

static KeyHandle_TypeDef hkey1;
static KeyHandle_TypeDef hkey2;   /*一键启动: 外接模块PI7(5V上拉, 按下低)*/

static void KEY1_ClickedCallback(void);
static void KEY2_ClickedCallback(void);

/**
  * 函    数：按键扫描任务函数（由 main.c 中 xTaskCreate 创建）
  * 参    数：pvParameters 未使用
  */
void KeyTask(void *pvParameters)
{
    (void)pvParameters;

    hkey1.GPIO_Port = KEY1_GPIO_Port;
    hkey1.GPIO_Pin = KEY1_Pin;
    hkey1.ClickedCallback = KEY1_ClickedCallback;

    hkey2.GPIO_Port = KEY2_GPIO_Port;
    hkey2.GPIO_Pin = KEY2_Pin;
    hkey2.ClickedCallback = KEY2_ClickedCallback;

    Key_Init(&hkey1);
    Key_Init(&hkey2);

    for (;;)
    {
        Key_Scan(&hkey1);
        Key_Scan(&hkey2);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

//按键松开回调函数：切换 LCD 页面（路线坐标页 / 任务码页），并翻转红色指示灯
static void KEY1_ClickedCallback(void)
{
    IMU_SetZero();          /*PB2: 重新记零点(当前朝向=0°)*/
    LED_Red_Toggle();       /*按键反馈，不需要可删掉这一行*/
}

/* 一键启动(PI7外接模块, 按下低电平, 高→低沿=按下):
   功能=上位机"连续跑两批"按钮(0x13 batch=3)。守卫在 Motor_DebugRunStart 内:
   路线未上传/运行中/已锁定时静默无效——route_debug_take 成功即锁定,
   保证一键启动的一次性语义(06中止+重新上传路线后才可能再次启动)。 */
static void KEY2_ClickedCallback(void)
{
    Motor_DebugRunStart(3u);
    LED_Red_Toggle();       /*按键反馈*/
}
