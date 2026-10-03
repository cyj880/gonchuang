#ifndef __ODOM_TASK_H
#define __ODOM_TASK_H

#include "stm32f4xx.h"

/**
  ******************************************************************************
  * @file    odom_task.h
  * @brief   双 RS-485 绝对值编码器里程计(X/Y 双测量轮直读坐标)
  ******************************************************************************
  * 站号约定: 站1 = X轴测量轮, 站2 = Y轴测量轮。
  * !! 新增编码器必须先用厂家上位机把站号改成 2, 两只同站号会总线冲突 !!
  *
  * x/y 为各测量轮"置零起累计位移"(带符号, mm), 绝对值方案无累计误差、不怕漏拍。
  * 安装方向若与场地轴反向, 改 odom_task.c 里 ODOM_X_SIGN / ODOM_Y_SIGN 即可。
  *
  * 读取接口(任意任务/任何时刻均可调用):
  *   odometry_get_x() / odometry_get_y()  -- 单值快捷读
  *   odometry_get_xy(&x, &y)              -- 一次取XY(定位只用坐标时用这个)
  *   odometry_get(&pack)                  -- 整包快照(含各站原始pos/里程/圈数/速度/在线)
  *   odometry_reset()                     -- 原点复位(XY同时清零, 下次读到位置生效)
  ******************************************************************************
  */

#define ODOM_ENC_NUM        2              /*总线上编码器数量: 站1=X, 站2=Y*/
#define ODOM_POS_AXIS       0              /*直线定位(05帧sub=0)反馈轴: 0=站1(X) 1=站2(Y)*/

typedef struct
{
    /* ---- 双轮直读坐标(上电/置零原点起, 带符号, mm) ---- */
    float    x_mm;              /*X轴测量轮(站1)累计位移*/
    float    y_mm;              /*Y轴测量轮(站2)累计位移*/
    /* ---- 各站细节数据(下标 0=站1/X, 1=站2/Y) ---- */
    int32_t  enc_pos[2];        /*各站有符号位置(编码值, 掉电归零型)*/
    int32_t  dist_mm[2];        /*各站带符号里程(mm)*/
    int32_t  turns[2];          /*各站累计圈数(带符号)*/
    float    speed_mm_s[2];     /*各站速度(mm/s)*/
    uint8_t  online[2];         /*各站RS485通信状态 1=正常*/
    float    yaw_deg;           /*BNO085航向(±180°, 上电即刷, 辅助/航向保持用)*/
} OdomData_t;

void OdomTask(void *pvParameters);      /* 轮询+解算任务(main.c 创建) */
void odometry_get(OdomData_t *out);     /* 整包快照(临界区保护) */
float odometry_get_x(void);             /* 快捷: 当前X(mm) */
float odometry_get_y(void);             /* 快捷: 当前Y(mm) */
void  odometry_get_xy(float *x, float *y); /* 快捷: 一次取XY(可传NULL忽略) */
void  odometry_reset(void);             /* 原点复位(XY同时清零) */

#endif /* __ODOM_TASK_H */
