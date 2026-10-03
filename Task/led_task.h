#ifndef __LED_TASK_H
#define __LED_TASK_H

/* LED 流水灯任务函数（在 main.c 中调用 xTaskCreate 创建） */
void LED_FlowTask(void *pvParameters);

#endif
