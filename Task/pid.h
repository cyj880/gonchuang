#ifndef __PID_H
#define __PID_H

/**
  * 位置式PID(移植自平衡车例程 User/PID.c/PID.h, 结构与逻辑保持一致):
  *   - 误差 = Target - Actual
  *   - 积分: Ki=0 时积分清零(便于分步调参), 积分带 ErrorIntMax/Min 限幅
  *   - 微分先行: Kd 作用于实际值变化量而非误差变化量(目标突变不振荡)
  *   - 输出偏移 OutOffset: 死区补偿(正输出加/负输出减)
  *   - 输出限幅: OutMax/OutMin
  */

typedef struct {
    float Target;      /*目标值, 由用户设定*/
    float Actual;      /*实际值, 从传感器读取*/
    float Actual1;     /*上次实际值*/
    float Out;         /*输出值, 作用于执行器*/

    float Kp;          /*比例项权重*/
    float Ki;          /*积分项权重*/
    float Kd;          /*微分项权重*/

    float Error0;      /*本次误差*/
    float Error1;      /*上次误差*/
    float ErrorInt;    /*误差积分*/

    float ErrorIntMax; /*误差积分的最大值*/
    float ErrorIntMin; /*误差积分的最小值*/

    float OutMax;      /*输出限幅的最大值*/
    float OutMin;      /*输出限幅的最小值*/

    float OutOffset;   /*输出偏移*/
} PID_t;

void PID_Init(PID_t *p);     /*清零状态(增益需在调用后重新赋值)*/
void PID_Update(PID_t *p);   /*一次PID计算并更新结构体*/

#endif /* __PID_H */
