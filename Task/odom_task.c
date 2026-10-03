#include "FreeRTOS.h"
#include "task.h"
#include "Encoder_RS485.h"
#include "lunqu_imu.h"
#include "odom_task.h"

/**
  ******************************************************************************
  * @file    odom_task.c
  * @brief   双 RS-485 绝对值编码器里程计(X/Y 双测量轮直读坐标)
  ******************************************************************************
  * 站号约定: 站1 = X轴测量轮, 站2 = Y轴测量轮(装反时对调站号表或改SIGN宏)。
  * 每站轮询"虚拟多圈位置"(32位, 掉电归零), 该位置由编码器内部固件连续累计
  * —— 无累计误差、不怕漏拍, 读到的就是绝对真相:
  *   x_mm = 站1 置零起累计位移;  y_mm = 站2 置零起累计位移。
  * 轮询周期: 2站 x ~25ms + 10ms ≈ 60ms(绝对值方案不怕漏拍, 对里程计足够)。
  *
  * 标定: 车沿某轴走已知距离 D(脉冲命令或卷尺),
  *   ODOM_WHEEL_CIRC_MM = 131072 x D / (该轴走完后的位置增量)
  ******************************************************************************
  */

static const uint8_t odom_stations[ODOM_ENC_NUM] = {1, 2};   /*站1=X轴 站2=Y轴*/

#define ODOM_COUNTS_PER_REV  131072.0f   /* 17bit: 一圈131072编码值 */
#define ODOM_WHEEL_CIRC_MM   200.0f      /* 压地轮周长(mm), 标定后修改! 两轮同规格 */
#define ODOM_MM_PER_COUNT    (ODOM_WHEEL_CIRC_MM / ODOM_COUNTS_PER_REV)
#define ODOM_X_SIGN          (+1.0f)     /*X测量轮安装反了改-1*/
#define ODOM_Y_SIGN          (+1.0f)     /*Y测量轮安装反了改-1*/

typedef struct
{
    int32_t  last;        /*上次位置(有符号)*/
    int32_t  cnt_acc;     /*累计编码值(带符号), 用于圈数*/
    uint8_t  have;        /*收到过有效位置*/
    uint8_t  zero_pend;   /*置零请求挂起*/
    uint8_t  miss;
    uint8_t  online;
} OdomCh;

static OdomCh        odom_ch[ODOM_ENC_NUM];
static OdomData_t    s_odom;
static volatile uint8_t s_reset_req = 0;

/**
  * 函    数：双编码器里程计任务(轮询 + 坐标解算)
  * 说    明：每站读为阻塞式(~25ms@9600), 两站共约60ms一周期;
  *           yaw 在循环顶部无条件透传(上电即刷, 不依赖RS485)。
  */
void OdomTask(void *pvParameters)
{
    uint8_t  idx;
    TickType_t last_tick[ODOM_ENC_NUM] = {0};
    (void)pvParameters;

    for (;;)
    {
        /*yaw 无条件透传(每轮一次): BNO085上电即输出, 编码器掉线也不影响yaw显示*/
        taskENTER_CRITICAL();
        s_odom.yaw_deg = IMU_GetYaw();   /*按陀螺仪开关取当前源(轮趣/BNO085)*/
        taskEXIT_CRITICAL();

        /*原点复位请求: 各通道置零挂起, 下次读到位置时生效(绝对值方案的优雅之处)*/
        if (s_reset_req)
        {
            uint8_t i;
            taskENTER_CRITICAL();
            for (i = 0; i < ODOM_ENC_NUM; i++)
            {
                odom_ch[i].zero_pend = 1;
            }
            s_odom.x_mm = 0.0f;
            s_odom.y_mm = 0.0f;
            taskEXIT_CRITICAL();
            s_reset_req = 0;
        }

        for (idx = 0; idx < ODOM_ENC_NUM; idx++)
        {
            uint32_t raw;
            if (EncRS485_ReadPos(odom_stations[idx], &raw))   /*阻塞约25ms*/
            {
                OdomCh *c = &odom_ch[idx];
                TickType_t now = xTaskGetTickCount();
                float dt_s = (float)(now - last_tick[idx]) / 1000.0f;

                int32_t pos = EncRS485_ToSigned(raw);   /*31bit回绕 -> 有符号*/
                int32_t d_cnt = pos - c->last;          /*有符号差值: 反转自然为负*/
                float d_mm = (float)d_cnt * ODOM_MM_PER_COUNT;

                if (!c->have) { d_mm = 0.0f; c->have = 1; }
                if (c->zero_pend) { d_mm = 0.0f; c->zero_pend = 0; }

                c->last = pos;
                c->miss = 0;
                c->online = 1;

                taskENTER_CRITICAL();
                s_odom.enc_pos[idx]  = pos;
                c->cnt_acc          += d_cnt;
                s_odom.dist_mm[idx] += (int32_t)d_mm;
                s_odom.turns[idx]    = c->cnt_acc / (int32_t)ODOM_COUNTS_PER_REV;
                s_odom.online[idx]   = 1;
                if (dt_s > 0.001f) s_odom.speed_mm_s[idx] = d_mm / dt_s;
                if (idx == 0) s_odom.x_mm = (float)s_odom.dist_mm[0] * ODOM_X_SIGN;
                else          s_odom.y_mm = (float)s_odom.dist_mm[1] * ODOM_Y_SIGN;
                taskEXIT_CRITICAL();

                last_tick[idx] = now;
            }
            else
            {
                if (odom_ch[idx].miss < 255) odom_ch[idx].miss++;
                if (odom_ch[idx].miss >= 5)
                {
                    odom_ch[idx].online = 0;
                    taskENTER_CRITICAL();
                    s_odom.online[idx] = 0;
                    taskEXIT_CRITICAL();
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
  * 函    数：整包快照读取(含双站原始数据)
  * 参    数：out 输出指针(可为NULL忽略)
  */
void odometry_get(OdomData_t *out)
{
    if (out == 0) return;

    taskENTER_CRITICAL();
    *out = s_odom;
    taskEXIT_CRITICAL();
}

/**
  * 函    数：快捷读取X坐标(mm) —— 定位只需单轴时直接调用
  */
float odometry_get_x(void)
{
    float v;
    taskENTER_CRITICAL();
    v = s_odom.x_mm;
    taskEXIT_CRITICAL();
    return v;
}

/**
  * 函    数：快捷读取Y坐标(mm) —— 定位只需单轴时直接调用
  */
float odometry_get_y(void)
{
    float v;
    taskENTER_CRITICAL();
    v = s_odom.y_mm;
    taskEXIT_CRITICAL();
    return v;
}

/**
  * 函    数：一次读取XY坐标(mm) —— 纯定位场景的推荐接口
  * 参    数：x/y 输出指针, 不需要的轴可传NULL
  */
void odometry_get_xy(float *x, float *y)
{
    taskENTER_CRITICAL();
    if (x != 0) *x = s_odom.x_mm;
    if (y != 0) *y = s_odom.y_mm;
    taskEXIT_CRITICAL();
}

/**
  * 函    数：原点复位(XY同时清零; 绝对值方案无需清编码器本身)
  * 说    明：只置请求标志, 各通道在下一次读到位置时挂起置零并生效
  */
void odometry_reset(void)
{
    s_reset_req = 1;
}
