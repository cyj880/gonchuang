#ifndef __ODOM_TASK_H
#define __ODOM_TASK_H

#include "stm32f4xx.h"

/* 仅保留站号1里程计；原始pos用于路线定位，数组下标固定为0。
   里程为测量轮累计位移，不代表地图的X/Y坐标。 */
#define ODOM_ENC_NUM        1
#define ODOM_POS_AXIS       0

typedef struct
{
    int32_t  enc_pos[ODOM_ENC_NUM];    /*有符号原始编码值*/
    int32_t  dist_mm[ODOM_ENC_NUM];    /*累计里程(mm)*/
    int32_t  turns[ODOM_ENC_NUM];      /*累计圈数*/
    float    speed_mm_s[ODOM_ENC_NUM]; /*测量轮速度(mm/s)*/
    uint8_t  online[ODOM_ENC_NUM];     /*站号1通信状态*/
    float    yaw_deg;                 /*当前陀螺仪航向*/
} OdomData_t;

void OdomTask(void *pvParameters);
void odometry_get(OdomData_t *out); /*整包快照，临界区保护*/
void odometry_reset(void);         /*里程原点复位，不修改原始编码值*/
uint32_t odometry_pause_encoder_read(OdomData_t *snapshot); /*停读并原子保存位置*/
void odometry_resume_encoder_read(void);
uint32_t odometry_encoder_sequence(void);

#endif /* __ODOM_TASK_H */
