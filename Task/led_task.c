#include "FreeRTOS.h"
#include "task.h"
#include "led_task.h"
#include "LED.h"

/**
  * 函    数：LED 流水灯任务函数（由 main.c 中 xTaskCreate 创建）
  * 参    数：pvParameters 未使用
  */
void LED_FlowTask(void *pvParameters)
{
    uint8_t i;

    (void)pvParameters;
    for (;;)
    {
        for (i = 1; i <= 8; i++)
        {
            LED_On(i);
            vTaskDelay(pdMS_TO_TICKS(500));
            LED_Off(i);
        }
    }
}
