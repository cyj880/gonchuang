#include "ring_align.h"
#include <math.h>
#include <string.h>

#define RING_TIMEOUT_MS 60000u
#define RING_TEST_MS 500u
#define RING_TOL_PX 3
#define RING_TARGET_X_PX 320
#define RING_TARGET_Y_PX 240 /* modes 0/4 center average Y; mode 3 is yaw only */
#define RING_TOL_YAW_CDEG 50 /* +/-0.5 deg */

typedef enum
{
    ALIGN_OFF, ALIGN_WAIT_DIR, ALIGN_WAIT_DATA, ALIGN_WAIT_ID, ALIGN_BACK,
    ALIGN_FORWARD, ALIGN_YAW, ALIGN_CENTER, ALIGN_SETTLE, ALIGN_DONE,
    ALIGN_TIMEOUT, ALIGN_TEST, ALIGN_TEST_RESULT
} AlignState;

static AlignState state;
static uint8_t align_mode; /* 0 station: XY then yaw then XY, 3 yaw only, 4 XY, 5 stack(单中心点) */
static int8_t yaw_sign = -1; /* 摄像头圆环Yaw校正方向：默认正Yaw用负旋转，两批粗加工及第一批暂存共用；串口可覆盖。 */
static uint32_t start_ms, step_ms, last_sequence, test_ms, test_sequence;
static uint8_t settled, test_after_valid;
static int8_t test_sign;
static int16_t test_before, test_after;
static float speed_x, speed_y, speed_w;
static RingAlignMotion last_motion;

static void reset_motion(void)
{
    speed_x = speed_y = speed_w = 0.0f;
    memset(&last_motion, 0, sizeof(last_motion));
}


static uint8_t full_result(const RDK_RingResult *r)
{
    return (uint8_t)(r->fresh && r->age_ms <= 300 && r->identified && r->count == 3 &&
        r->yaw_valid && r->points[0].id == 1 && r->points[1].id == 2 && r->points[2].id == 3);
}

static float slew(float current, float target, float step)
{
    if (target > current + step) return current + step;
    if (target < current - step) return current - step;
    return target;
}

static int rpm_round(float speed)
{
    return speed < 0 ? -(int)(-speed + 0.5f) : (int)(speed + 0.5f);
}

/* 2026-11-05: 等待/终态不再突停——速度按斜坡衰减到0后输出, 消除轮速
   突跳引起的原地打滑(全向轮换向/启停突变最容易滑)。 */
static RingAlignMotion decay_and_output(float dt)
{
    RingAlignMotion out;

    speed_x = slew(speed_x, 0.0f, 100.0f * dt);
    speed_y = slew(speed_y, 0.0f, 100.0f * dt);
    speed_w = slew(speed_w, 0.0f, 20.0f * dt);
    out.vx = rpm_round(speed_x);
    out.vy = rpm_round(speed_y);
    out.w = rpm_round(speed_w);
    last_motion = out;
    return out;
}


static float center_speed(float error)
{
    /* 2026-10-24 overall slow-down: 4..20 RPM, gain 0.5 RPM/px. */
    float speed = fabsf(error) * 0.5f;
    if (speed < 12.0f) speed = 12.0f;
    if (speed > 60.0f) speed = 60.0f;
    speed *= (2.0f / 3.0f) * 0.5f;
    /* Fine approach inside 2x tolerance: taper below the 4 RPM floor so the
       tight +/-3 px window is not overshot at crawl speed. */
    if (fabsf(error) <= 2.0f * RING_TOL_PX)
        speed = fabsf(error) * (2.0f / 3.0f);
    return error < 0 ? -speed : speed;
}

void RingAlign_Start(uint32_t now_ms)
{
    align_mode = 0;
    start_ms = step_ms = now_ms;
    last_sequence = 0;
    settled = 0;
    test_after_valid = 0;
    state = yaw_sign ? ALIGN_WAIT_DATA : ALIGN_WAIT_DIR;
    reset_motion();
}

void RingAlign_StartMode(uint8_t mode, int8_t sign, uint32_t now_ms)
{
    RingAlign_Start(now_ms);
    align_mode = mode;
    if (mode == 3) yaw_sign = sign;
    state = ALIGN_WAIT_DATA;
}

void RingAlign_ResetYawSign(void)
{
    yaw_sign = -1;
}

void RingAlign_Stop(void)
{
    state = ALIGN_OFF;
    settled = 0;
    reset_motion();
}

uint8_t RingAlign_Command(uint8_t op, int8_t value, const RDK_RingResult *r, uint32_t now_ms)
{
    if (op == 2 && (value == -1 || value == 0 || value == 1))
    {
        yaw_sign = value;
        if (state != ALIGN_OFF) RingAlign_Start(now_ms);
        return 1;
    }
    if (op == 1 && (value == -1 || value == 1) && state != ALIGN_OFF &&
        state != ALIGN_TEST && state != ALIGN_TIMEOUT &&
        !last_motion.vx && !last_motion.vy && !last_motion.w && full_result(r))
    {
        reset_motion();
        test_before = r->yaw_cdeg;
        test_after_valid = 0;
        test_ms = now_ms;
        test_sequence = r->sequence;
        test_sign = value;
        state = ALIGN_TEST;
        settled = 0;
        return 1;
    }
    return 0;
}

void RingAlign_Update(const RDK_RingResult *r, uint32_t now_ms, RingAlignMotion *out)
{
    float dt = (float)(uint32_t)(now_ms - step_ms) / 1000.0f;
    float vx = 0, vy = 0, w = 0;
    AlignState next;
    uint8_t mask = 0, i, fresh_sample;
    int sum_x, sum_y;
    *out = (RingAlignMotion){0, 0, 0};
    step_ms = now_ms;
    if (dt > 0.05f) dt = 0.05f;
    if (state == ALIGN_OFF || state == ALIGN_DONE || state == ALIGN_TIMEOUT)
    { *out = decay_and_output(dt); return; }
    if (now_ms - start_ms >= RING_TIMEOUT_MS)
    { state = ALIGN_TIMEOUT; settled = 0; *out = decay_and_output(dt); return; }

    if (state == ALIGN_TEST)
    {
        if (now_ms - test_ms >= RING_TEST_MS || !full_result(r))
        {
            state = ALIGN_TEST_RESULT;
            test_sequence = r->sequence; /* after must be a new post-stop observation */
            reset_motion();
        }
        else
        {
            out->w = 2 * test_sign;
            last_motion = *out;
        }
        return;
    }
    if (state == ALIGN_TEST_RESULT)
    {
        if (!test_after_valid && full_result(r) && r->sequence != test_sequence)
        {
            test_after = r->yaw_cdeg;
            test_after_valid = 1;
        }
        reset_motion();
        return; /* explicit direction confirmation/restart required */
    }
    if (align_mode != 4 && !yaw_sign)
    { state = ALIGN_WAIT_DIR; settled = 0; *out = decay_and_output(dt); return; }
    if (!r->fresh || r->age_ms > 300 || !r->count)
    { state = ALIGN_WAIT_DATA; settled = 0; *out = decay_and_output(dt); return; }
    if (!r->identified || r->count > 3)
    { state = ALIGN_WAIT_ID; settled = 0; *out = decay_and_output(dt); return; }
    for (i = 0; i < r->count; i++)
    {
        uint8_t id = r->points[i].id;
        if (!id || id > 3 || (mask & (1u << id)) ||
            (i && id <= r->points[i - 1].id))
        { state = ALIGN_WAIT_ID; settled = 0; *out = decay_and_output(dt); return; }
        mask |= 1u << id;
    }
    fresh_sample = (uint8_t)(r->sequence != last_sequence);
    last_sequence = r->sequence;
    if (align_mode == 3 && !full_result(r))
    { state = ALIGN_WAIT_ID; settled = 0; *out = decay_and_output(dt); return; }
    if (align_mode == 5)
    {
        /* 码垛(0x14帧单中心点转单点RingResult, points[0].id=1):
           2026-11-05与mode0同款往复校准——XY优先, XY在容差内才修yaw;
           yaw转动会改变圆心像素位置, 可能再次触发XY, 如此往复。
           两者同时在容差内连续5个新结果 -> DONE; 符号映射与OP4相同
           (图像Y->vx, 图像X->vy)。FLAGS=0探照帧 -> 停车等待。 */
        int xy_bad, yaw_bad;
        if (!r->yaw_valid)
        { state = ALIGN_WAIT_DATA; settled = 0; *out = decay_and_output(dt); return; }
        sum_x = (int)r->points[0].x - RING_TARGET_X_PX;
        sum_y = (int)r->points[0].y - RING_TARGET_Y_PX;
        xy_bad = (sum_x < -RING_TOL_PX || sum_x > RING_TOL_PX ||
                  sum_y < -RING_TOL_PX || sum_y > RING_TOL_PX);
        yaw_bad = (r->yaw_cdeg < -RING_TOL_YAW_CDEG ||
                   r->yaw_cdeg > RING_TOL_YAW_CDEG);
        if (xy_bad)
        {
            next = ALIGN_CENTER;
            if (sum_x < -RING_TOL_PX || sum_x > RING_TOL_PX)
                vy = center_speed(-(float)sum_x);   /*图像X -> vy(同OP4)*/
            if (sum_y < -RING_TOL_PX || sum_y > RING_TOL_PX)
                vx = center_speed((float)sum_y);    /*图像Y -> vx*/
            settled = 0;
        }
        else if (yaw_bad)
        {
            next = ALIGN_YAW;
            w = fabsf((float)r->yaw_cdeg) / 50.0f;
            if (w < 2) w = 2;
            if (w > 6) w = 6;
            w *= (r->yaw_cdeg < 0 ? -1 : 1) * yaw_sign;
            settled = 0;
        }
        else
        {
            if (fresh_sample && ++settled >= 5) state = ALIGN_DONE;
            else state = ALIGN_SETTLE;
            *out = decay_and_output(dt);
            return;
        }
    }
    else if (mask == (1u << 1) || mask == ((1u << 1) | (1u << 2)))
    { next = ALIGN_BACK; vy = -3; settled = 0; }
    else if (mask == (1u << 3) || mask == ((1u << 2) | (1u << 3)))
    { next = ALIGN_FORWARD; vy = 3; settled = 0; }
    else if (mask == 14 && (align_mode != 3 || full_result(r)))
    {
        sum_x = (int)r->points[0].x + r->points[1].x + r->points[2].x - 3 * RING_TARGET_X_PX;
        sum_y = (int)r->points[0].y + r->points[1].y + r->points[2].y - 3 * RING_TARGET_Y_PX;
        if (align_mode == 0)
        {
            /* 2026-11-05: XY与yaw往复校准——XY优先, XY在容差内才修yaw;
               yaw转动会改变圆心像素位置, 可能再次触发XY, 如此往复。
               两者同时在容差内且连续5个新结果确认 -> DONE。 */
            int xy_bad = (sum_x < -3 * RING_TOL_PX || sum_x > 3 * RING_TOL_PX ||
                          sum_y < -3 * RING_TOL_PX || sum_y > 3 * RING_TOL_PX);
            int yaw_bad = (r->yaw_cdeg < -RING_TOL_YAW_CDEG ||
                           r->yaw_cdeg > RING_TOL_YAW_CDEG);

            if (xy_bad)
            {
                next = ALIGN_CENTER;
                if (sum_x < -3 * RING_TOL_PX || sum_x > 3 * RING_TOL_PX)
                    vy = center_speed(-(float)sum_x / 3.0f);
                /* 与OP4共用安装方向：图像Y控制车体vx，图像X控制vy。 */
                if (sum_y < -3 * RING_TOL_PX || sum_y > 3 * RING_TOL_PX)
                    vx = center_speed((float)sum_y / 3.0f);
                settled = 0;
            }
            else if (yaw_bad)
            {
                next = ALIGN_YAW;
                w = fabsf((float)r->yaw_cdeg) / 50.0f;
                if (w < 2) w = 2;
                if (w > 6) w = 6;
                w *= (r->yaw_cdeg < 0 ? -1 : 1) * yaw_sign;
                settled = 0;
            }
            else
            {
                if (fresh_sample && ++settled >= 5) state = ALIGN_DONE;
                else state = ALIGN_SETTLE;
                *out = decay_and_output(dt);
                return;
            }
        }
        else if (align_mode != 4 && (r->yaw_cdeg < -RING_TOL_YAW_CDEG || r->yaw_cdeg > RING_TOL_YAW_CDEG))
        {
            next = ALIGN_YAW;
            w = fabsf((float)r->yaw_cdeg) / 50.0f;
            if (w < 2) w = 2;
            if (w > 6) w = 6;
            w *= (r->yaw_cdeg < 0 ? -1 : 1) * yaw_sign;
            settled = 0;
        }
        else if (align_mode != 3 &&
                 (sum_x < -3 * RING_TOL_PX || sum_x > 3 * RING_TOL_PX ||
                  (align_mode == 4 &&
                   (sum_y < -3 * RING_TOL_PX || sum_y > 3 * RING_TOL_PX))))
        {
            next = ALIGN_CENTER;
            if (sum_x < -3 * RING_TOL_PX || sum_x > 3 * RING_TOL_PX)
                vy = center_speed(-(float)sum_x / 3.0f);
            /* image Y -> vx(+1), image X -> vy(-1): turntable-measured cross mapping */
            if (align_mode == 4 &&
                (sum_y < -3 * RING_TOL_PX || sum_y > 3 * RING_TOL_PX))
                vx = center_speed((float)sum_y / 3.0f);
            settled = 0;
        }
        else
        {
            if (fresh_sample && ++settled >= 5) state = ALIGN_DONE;
            else state = ALIGN_SETTLE;
            *out = decay_and_output(dt);
            return;
        }
    }
    else
    { state = ALIGN_WAIT_ID; settled = 0; *out = decay_and_output(dt); return; }

    /* 2026-11-05: 状态切换不再硬停一拍——新旧目标由底部斜坡平滑过渡
       (含穿零), CENTER<->YAW往复切换不再产生轮速突跳。 */
    if (state != next)
    {
        state = next;
    }
    /* 2026-11-05: 取消反向/归零瞬断清零——速度一律经斜坡连续过渡
       (含穿零反向), 配合100 RPM/s斜坡消除平移启停打滑。 */
    speed_x = slew(speed_x, vx, 100.0f * dt);
    speed_y = slew(speed_y, vy, 100.0f * dt);
    speed_w = slew(speed_w, w, 20.0f * dt);
    out->vx = rpm_round(speed_x);
    out->vy = rpm_round(speed_y);
    out->w = rpm_round(speed_w);
    last_motion = *out;
}

uint8_t RingAlign_Done(void)
{
    return (uint8_t)(state == ALIGN_DONE);
}

const char *RingAlign_State(void)
{
    static const char *names[] = {"OFF", "SET DIR", "WAIT DATA", "WAIT ID", "BACK",
        "FORWARD", "YAW", "CENTER", "SETTLE", "DONE", "TIMEOUT", "TEST", "TEST STOP"};
    if (state == ALIGN_DONE && align_mode == 3) return "YAW DONE";
    if (state == ALIGN_DONE && align_mode == 4) return "XY DONE";
    return names[state];
}

uint8_t RingAlign_TestYaw(int16_t *before, int16_t *after)
{
    *before = test_before;
    *after = test_after;
    return test_after_valid;
}
