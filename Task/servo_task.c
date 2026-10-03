#include "FreeRTOS.h"
#include "task.h"
#include "servo_task.h"
#include "Servo.h"

/**
  * 270°舵机三工位转盘对位测试（180°舵机暂不加缓动，保持上电中位 90° 不动）
  *
  * 缓动曲线（t ∈ [0,1] 归一化时间，s 归一化位置）：
  *   三次多项式 smoothstep（去程用）：
  *     位置 s(t) = 3t² - 2t³，速度 v(t) = 6t(1-t) —— 开口向下的抛物线，
  *     两端速度为 0，中点最快（峰值 = 1.5 倍平均速度）。
  *   五次多项式 smootherstep（255°→15° 回程用）：
  *     位置 s(t) = 6t⁵ - 15t⁴ + 10t³，速度 v(t) = 30t²(1-t)²，
  *     加速度 a(t) = 60t(1-t)(1-2t)，两端速度、加速度均为 0 ——
  *     与前后步骤衔接处速度连续、加速度也连续，无加速度突跳。
  *
  * 对位步骤表（脉宽 500~2500us 线性映射 0~270°）：
  *   步骤0  15°              ≈611us    盘1
  *   步骤1  15°+120°=135°    1500us    盘2（正中点）
  *   步骤2  135°+120°=255°   ≈2389us   盘3
  *   循环   直接回 15°        ≈611us    回到盘1（五次多项式缓动），重复
  *
  * 节奏：每 3s 走一个步骤 —— 缓动 2s 到下一工位 + 静置 1s。
  * 上电默认停在盘1工位 15°（上电脉宽由 Servo270_Init 设为 ≈611us），
  * 静置 1s 等舵机稳定后直接进入循环，从盘1开始走。
  * 注意：回程 240° 与去程 120° 同用 2s，且五次曲线峰值速度 = 1.875 倍平均
  *       （三次为 1.5 倍），回程峰值 ≈225°/s，比去程明显快；嫌快可加大回程
  *       EaseTo 的时长并相应减少静置时间，保持 3s 一步。
  */

/* 单拍周期：20ms 一拍（50Hz，与舵机 PWM 同频） */
#define SERVO270_EASE_STEP_MS   20u

/* 缓动曲线类型 */
typedef enum {
    EASE_CUBIC   = 0,   /* 三次多项式：两端速度0，速度曲线抛物线 */
    EASE_QUINTIC = 1,   /* 五次多项式：两端速度、加速度均为0 */
} Servo_EaseType;

/* 三工位对位角度：盘1 / 盘2 / 盘3（每步 120°） */
static const float servo270_step_angle[3] = {15.0f, 135.0f, 255.0f};

/**
  * 函    数：缓动曲线
  * 参    数：t     归一化时间 0~1
  *           curve 曲线类型
  * 返 回 值：归一化位置 0~1
  */
static float Servo_EaseCurve(float t, Servo_EaseType curve)
{
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    if (curve == EASE_QUINTIC)
        return t * t * t * (6.0f * t * t - 15.0f * t + 10.0f);  /* 6t⁵-15t⁴+10t³ */

    return t * t * (3.0f - 2.0f * t);                           /* 3t²-2t³ */
}

/**
  * 函    数：270°舵机缓动到目标角度
  * 参    数：start       起始角度
  *           target      目标角度（0~270）
  *           duration_ms 走完全程用时（ms）
  *           curve       缓动曲线类型（EASE_CUBIC / EASE_QUINTIC）
  * 说    明：阻塞式，按所选缓动曲线插值，每 20ms 更新一拍
  */
static void Servo270_EaseTo(float start, float target, uint32_t duration_ms,
                            Servo_EaseType curve)
{
    uint32_t elapsed;

    for (elapsed = 0; elapsed < duration_ms; elapsed += SERVO270_EASE_STEP_MS)
    {
        float t = (float)elapsed / (float)duration_ms;
        Servo270_SetAngle(start + (target - start) * Servo_EaseCurve(t, curve));
        vTaskDelay(pdMS_TO_TICKS(SERVO270_EASE_STEP_MS));
    }
    Servo270_SetAngle(target);      /* 补一拍，保证精确停在目标角 */
}

/**
  * 函    数：270°舵机转盘对位任务函数（由 main.c 中 xTaskCreate 创建）
  * 参    数：pvParameters 未使用
  */
void Servo_Task(void *pvParameters)
{
    uint8_t step = 0;   /* 上电默认在盘1（15°，见 Servo270_Init 上电脉宽） */

    (void)pvParameters;
    vTaskDelay(pdMS_TO_TICKS(1000));    /* 上电静置 1s 等舵机稳定，再走第一步 */

    for (;;)
    {
        /* 0→1→2→0 循环：255° 直接回 15° 的回程用五次多项式（速度、加速度连续），其余三次 */
        uint8_t next = (uint8_t)((step + 1) % 3);
        Servo_EaseType curve = (next == 0) ? EASE_QUINTIC : EASE_CUBIC;
        Servo270_EaseTo(servo270_step_angle[step], servo270_step_angle[next], 2000, curve);
        step = next;
        vTaskDelay(pdMS_TO_TICKS(1000));    /* 缓动2s + 静置1s = 每3s一步 */
    }
}
