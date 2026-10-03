#include "pid.h"

/**
  * 函    数：PID初始化(移植自平衡车例程)
  * 参    数：p 指定结构体的地址
  * 返 回 值：无
  * 说    明：把PID运行状态清零，避免之前遗留的参数对本次启动造成影响
  *           注意增益(Kp/Ki/Kd)与限幅也被清零，初始化后需重新赋值
  */
void PID_Init(PID_t *p)
{
    p->Target    = 0;
    p->Actual    = 0;
    p->Actual1   = 0;
    p->Out       = 0;
    p->Error0    = 0;
    p->Error1    = 0;
    p->ErrorInt  = 0;
}

/**
  * 函    数：PID计算及结构体变量值更新
  * 参    数：p 指定结构体的地址
  * 返 回 值：无
  */
void PID_Update(PID_t *p)
{
    /*获取本次误差和上次误差*/
    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;

    /*外环误差积分(累加)*/
    /*Ki为0时积分直接清零: 便于调试(Ki=0时积分不参与, 清零防止后续突跳)*/
    if (p->Ki != 0)
    {
        p->ErrorInt += p->Error0;

        /*误差积分限幅*/
        if (p->ErrorInt > p->ErrorIntMax) { p->ErrorInt = p->ErrorIntMax; }
        if (p->ErrorInt < p->ErrorIntMin) { p->ErrorInt = p->ErrorIntMin; }
    }
    else
    {
        p->ErrorInt = 0;
    }

    /*位置式PID公式(微分先行: 微分作用于实际值变化量, 目标突变不振荡)*/
    p->Out = p->Kp * p->Error0
           + p->Ki * p->ErrorInt
           - p->Kd * (p->Actual - p->Actual1);

    /*输出偏移(死区补偿)*/
    if (p->Out > 0) { p->Out += p->OutOffset; }
    if (p->Out < 0) { p->Out -= p->OutOffset; }

    /*输出限幅*/
    if (p->Out > p->OutMax) { p->Out = p->OutMax; }
    if (p->Out < p->OutMin) { p->Out = p->OutMin; }

    /*本轮计算后变量传递, 下轮计算时Actual1即为上次实际值*/
    p->Actual1 = p->Actual;
}
