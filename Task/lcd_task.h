#ifndef __LCD_TASK_H__
#define __LCD_TASK_H__

#include "route_rx.h"

/* LCD 显示任务（在 main.c 中调用 xTaskCreate 创建） */
void LCD_Task(void *pvParameters);

/* PB2 按键回调里调用：在"路线坐标页 / 任务码页 / 里程计调试页"三页间轮换
   （只置标志，实际绘制由 LCD 任务完成，可从任意任务调用） */
void LCD_TogglePage(void);

/* route_task 收到完整坐标帧后调用：把帧快照交给 LCD 任务分页显示
   （内部会拷贝一份，不会阻塞中断，也不会被后续帧撕裂） */
void LCD_ShowRoute(const RouteFrame *f);

#endif
