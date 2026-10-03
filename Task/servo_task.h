#ifndef __SERVO_TASK_H
#define __SERVO_TASK_H

/* 舵机扫描任务函数（在 main.c 中调用 xTaskCreate 创建） */
void Servo_Task(void *pvParameters);

#endif
