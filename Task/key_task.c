#include "FreeRTOS.h"
#include "task.h"
#include "key_task.h"
#include "Key.h"
#include "LED.h"
#include "lcd_task.h"

/**
  * 按键任务（源自 STM32F407 例程 key_task.c 移植）
  * 每 10ms 扫描一次 A 板用户按键（PB2，按下为高电平），
  * 检测到松开沿时：切换 LCD 页面（路线坐标页 <-> 任务码页），
  * 同时翻转板载红色指示灯做按键反馈。
  */

static KeyHandle_TypeDef hkey1;

static void KEY1_ClickedCallback(void);

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

    Key_Init(&hkey1);

    for (;;)
    {
        Key_Scan(&hkey1);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

//按键松开回调函数：切换 LCD 页面（路线坐标页 / 任务码页），并翻转红色指示灯
static void KEY1_ClickedCallback(void)
{
    LCD_TogglePage();
    LED_Red_Toggle();       /*按键反馈，不需要可删掉这一行*/
}
