#include "FreeRTOS.h"
#include "task.h"
#include "Encoder_RS485.h"
#include "lunqu_imu.h"
#include "odom_task.h"

/* 站号1单里程计：读取编码器内部累计的有符号多圈位置。
   标定周长 = 131072 * 实测距离(mm) / 原始编码增量。
   每次轮询站号1后延时15ms，不再访问不存在的站号2。 */
#define ODOM_COUNTS_PER_REV  131072.0f
#define ODOM_WHEEL_CIRC_MM   200.0f
#define ODOM_MM_PER_COUNT    (ODOM_WHEEL_CIRC_MM / ODOM_COUNTS_PER_REV)

typedef struct
{
    int32_t  last;        /*上次位置(有符号)*/
    int32_t  cnt_acc;     /*累计编码值(带符号), 用于圈数*/
    uint8_t  have;        /*收到过有效位置*/
    uint8_t  zero_pend;   /*置零请求挂起*/
    uint8_t  miss;
} OdomCh;

static OdomCh        odom_ch;
static OdomData_t    s_odom;
static volatile uint8_t s_reset_req = 0;

/* 编码器轮询任务，yaw更新不依赖编码器在线状态。 */
void OdomTask(void *pvParameters)
{
    TickType_t last_tick = 0;
    (void)pvParameters;

    for (;;)
    {
        /*编码器掉线时仍刷新yaw*/
        taskENTER_CRITICAL();
        s_odom.yaw_deg = IMU_GetYaw();   /*按陀螺仪开关取当前源(轮趣/BNO085)*/
        taskEXIT_CRITICAL();

        /*原点复位请求在下次有效读数时生效*/
        if (s_reset_req)
        {
            taskENTER_CRITICAL();
            odom_ch.zero_pend = 1;
            odom_ch.cnt_acc = 0;
            s_odom.dist_mm[ODOM_POS_AXIS] = 0;
            s_odom.turns[ODOM_POS_AXIS] = 0;
            s_odom.speed_mm_s[ODOM_POS_AXIS] = 0.0f;
            taskEXIT_CRITICAL();
            s_reset_req = 0;
        }

        /*仅访问站号1*/
        {
            uint32_t raw;
            if (EncRS485_ReadPos(ENC_RS485_DEFAULT_STATION, &raw))   /*阻塞约25ms*/
            {
                OdomCh *c = &odom_ch;
                TickType_t now = xTaskGetTickCount();
                float dt_s = (float)(now - last_tick) / 1000.0f;

                int32_t pos = EncRS485_ToSigned(raw);   /*31bit回绕 -> 有符号*/
                int32_t d_cnt = pos - c->last;          /*有符号差值: 反转自然为负*/
                float d_mm = (float)d_cnt * ODOM_MM_PER_COUNT;

                if (!c->have) { d_cnt = 0; d_mm = 0.0f; c->have = 1; }
                if (c->zero_pend) { d_cnt = 0; d_mm = 0.0f; c->zero_pend = 0; }

                c->last = pos;
                c->miss = 0;

                taskENTER_CRITICAL();
                s_odom.enc_pos[ODOM_POS_AXIS]  = pos;
                c->cnt_acc          += d_cnt;
                s_odom.dist_mm[ODOM_POS_AXIS] += (int32_t)d_mm;
                s_odom.turns[ODOM_POS_AXIS]    = c->cnt_acc / (int32_t)ODOM_COUNTS_PER_REV;
                s_odom.online[ODOM_POS_AXIS]   = 1;
                if (dt_s > 0.001f) s_odom.speed_mm_s[ODOM_POS_AXIS] = d_mm / dt_s;
                taskEXIT_CRITICAL();

                last_tick = now;
            }
            else
            {
                if (odom_ch.miss < 255) odom_ch.miss++;
                if (odom_ch.miss >= 5)
                {
                    taskENTER_CRITICAL();
                    s_odom.online[ODOM_POS_AXIS] = 0;
                    taskEXIT_CRITICAL();
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(15));
    }
}

/**
  * 函    数：整包快照读取(站号1)
  * 参    数：out 输出指针(可为NULL忽略)
  */
void odometry_get(OdomData_t *out)
{
    if (out == 0) return;

    taskENTER_CRITICAL();
    *out = s_odom;
    taskEXIT_CRITICAL();
}

/*里程原点复位，不清除用于路线定位的原始编码值*/
void odometry_reset(void)
{
    s_reset_req = 1;
}
