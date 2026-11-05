#include "rdk_center.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define RDK_HEAD       0x66u
#define RDK_CMD_CENTER 0x01u
#define RDK_TAIL       0x77u
#define RDK_MODE_CMD   0x10u
#define RDK_MODE_RAW   0x00u
#define RDK_MODE_RING  0x01u
#define RDK_MODE_ALL_MATERIALS 0x02u /* 全物料识别 */
#define RDK_MODE_TURNTABLE 0x04u
#define RDK_MODE_STACK  0x05u        /* 码垛识别 */
#define RDK_STACK_CMD   0x14u        /* RDK→STM32: 码垛单中心点(LEN固定7) */
#define RDK_RING_RESULT 0x12u
#define RDK_RING_IDENTIFIED 0x13u
#define RDK_GRAB_CMD    0x11u       /* RDK→STM32: 夹取触发(目标颜色物体静止时持续发送) */
#define RDK_PARTIAL_TIMEOUT_MS 50u

static volatile uint8_t  rdk_enabled;
static volatile uint8_t  rdk_valid;
static volatile int16_t rdk_x;
static volatile int16_t rdk_y;
static volatile TickType_t rdk_last_tick;
static volatile uint32_t rdk_sequence;
static volatile uint32_t rdk_rx_bytes;
static volatile uint32_t rdk_tap_bytes;
static volatile uint32_t rdk_tap_wr;
static volatile uint8_t rdk_tap[8];   /*最近8字节hex采样, 诊断RDK到底发了什么*/
static volatile uint32_t rdk_valid_frames;
static volatile uint32_t rdk_errors;
static volatile uint32_t rdk_overruns;
static volatile uint8_t rdk_ring_enabled;
static volatile uint8_t rdk_stack_enabled;
static volatile uint8_t rdk_grab_arm_enabled; /* 1=允许接收统一夹取触发帧 */
static volatile uint8_t rdk_grab_request;     /* 收到触发帧, 由MotorTask取走 */
static RDK_RingResult ring_result;
static RDK_StackResult stack_result; /*同ring_result: 临界区/ISR内访问, 非volatile*/
static TickType_t ring_last_tick;
static TickType_t stack_last_tick;
static uint8_t rx_buf[32];
static uint8_t rx_index;
static TickType_t rx_last_tick;

static void rdk_tx_byte(uint8_t byte)
{
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET) {}
    USART_SendData(USART3, byte);
}

/* STM32 -> RDK: AA 55 10 01 MODE CHECK 0D 0A. */
static void rdk_send_mode(uint8_t mode)
{
    rdk_tx_byte(0xAAu);
    rdk_tx_byte(0x55u);
    rdk_tx_byte(RDK_MODE_CMD);
    rdk_tx_byte(0x01u);
    rdk_tx_byte(mode);
    rdk_tx_byte((uint8_t)(RDK_MODE_CMD + 0x01u + mode));
    rdk_tx_byte(0x0Du);
    rdk_tx_byte(0x0Au);
}

void RDK_Center_Init(uint32_t baud)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &gpio);

    USART_StructInit(&usart);
    usart.USART_BaudRate = baud;
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART3, &usart);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART3, ENABLE);

    nvic.NVIC_IRQChannel = USART3_IRQn;
    /* FromISR API要求优先级数值>=5；与UART7/UART8保持一致。 */
    nvic.NVIC_IRQChannelPreemptionPriority = 6;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    RDK_Center_Clear();
    RDK_Center_Enable(0);
}

/* 码垛0x14帧: AA 55 14 07 FLAGS U(2) V(2) YAW(2) CK 0D 0A。FLAGS只允许bit0;
   FLAGS=0(探照帧)要求x/y/yaw全0; FLAGS=1校验坐标0~639/0~479、yaw±180°。 */
static uint8_t rdk_accept_stack(const uint8_t *p, TickType_t now)
{
    RDK_StackResult next = {0};

    next.flags = p[0];
    if (next.flags & ~1u) return 0;
    next.x = (uint16_t)p[1] | ((uint16_t)p[2] << 8);
    next.y = (uint16_t)p[3] | ((uint16_t)p[4] << 8);
    next.yaw_cdeg = (int16_t)((uint16_t)p[5] | ((uint16_t)p[6] << 8));
    if (next.flags & 1u)
    {
        if (next.x >= 640 || next.y >= 480 ||
            next.yaw_cdeg < -18000 || next.yaw_cdeg > 18000) return 0;
    }
    else if (next.x || next.y || next.yaw_cdeg) return 0;
    next.sequence = stack_result.sequence + 1u;
    next.received_ms = (uint32_t)((uint64_t)now * 1000u / configTICK_RATE_HZ);
    next.fresh = 1;
    stack_result = next;
    stack_last_tick = now;
    return 1;
}

/* Called only by USART3 ISR; publish snapshots after complete validation. */
static uint8_t rdk_accept_ring(const uint8_t *payload, uint8_t length, uint8_t command, TickType_t now)
{
    RDK_RingResult next = {0};
    uint8_t i, seen = 0;
    uint8_t stride = command == RDK_RING_IDENTIFIED ? 5u : 4u;
    if (length < 4 || payload[0] > 3 || length != 4u + stride * payload[0]) return 0;
    next.identified = (uint8_t)(command == RDK_RING_IDENTIFIED);
    next.count = payload[0];
    next.yaw_valid = payload[1] & 1u;
    next.yaw_cdeg = (int16_t)((uint16_t)payload[2] | ((uint16_t)payload[3] << 8));
    if ((payload[1] & ~1u) || next.yaw_cdeg < -9000 || next.yaw_cdeg > 9000 ||
        (next.count < 2 && next.yaw_valid) || (!next.yaw_valid && next.yaw_cdeg != 0)) return 0;
    for (i = 0; i < next.count; i++)
    {
        uint8_t offset = 4u + i * stride;
        if (next.identified)
        {
            uint8_t id = payload[offset++];
            if (id > 3 || (id && (seen & (1u << id)))) return 0;
            next.points[i].id = id;
            if (id) seen |= 1u << id;
        }
        next.points[i].x = (uint16_t)payload[offset] | ((uint16_t)payload[offset + 1] << 8);
        next.points[i].y = (uint16_t)payload[offset + 2] | ((uint16_t)payload[offset + 3] << 8);
        if (next.points[i].x >= 640 || next.points[i].y >= 480) return 0;
        if (i && (next.points[i].x < next.points[i - 1].x ||
            (next.points[i].x == next.points[i - 1].x &&
             next.points[i].y <= next.points[i - 1].y))) return 0;
    }
    next.sequence = ring_result.sequence + 1u;
    next.received_ms = (uint32_t)((uint64_t)now * 1000u / configTICK_RATE_HZ);
    next.fresh = 1;
    ring_result = next;
    ring_last_tick = now;
    return 1;
}

static void rdk_rx_drop(uint8_t count)
{
    rx_index -= count;
    memmove(rx_buf, rx_buf + count, rx_index);
}

static void rdk_rx_byte(uint8_t byte, TickType_t now)
{
    if (!rdk_enabled && !rdk_ring_enabled &&
        !rdk_stack_enabled && !rdk_grab_arm_enabled)
    { rx_index = 0; return; }
    if (rx_index && now - rx_last_tick > pdMS_TO_TICKS(RDK_PARTIAL_TIMEOUT_MS))
    {
        rx_index = 0;
        rdk_errors++;
    }
    rx_last_tick = now;
    rx_buf[rx_index++] = byte;
    while (rx_index)
    {
        if (rdk_enabled && rx_buf[0] == RDK_HEAD)
        {
            int16_t x, y;
            if (rx_index < 2) return;
            if (rx_buf[1] != RDK_CMD_CENTER) { rdk_rx_drop(1); continue; }
            if (rx_index < 7) return;
            x = (int16_t)(((uint16_t)rx_buf[2] << 8) | rx_buf[3]);
            y = (int16_t)(((uint16_t)rx_buf[4] << 8) | rx_buf[5]);
            if (rx_buf[6] != RDK_TAIL || x < 0 || x >= 640 || y < 0 || y >= 480)
            {
                rdk_errors++;
                rdk_rx_drop(1);
                continue;
            }
            rdk_x = x;
            rdk_y = y;
            rdk_valid = 1;
            rdk_last_tick = now;
            rdk_sequence++;
            rdk_valid_frames++;
            rdk_rx_drop(7);
        }
        else if (rdk_grab_arm_enabled && rx_buf[0] == 0xAAu)
        {
            /* 统一夹取触发帧: AA 55 11 00 11 0D 0A, 恒定7字节(2026-11-03)。
               每次对位完成后才打开接收, 收到一帧即取走并关闭, 不会跨次复用。 */
            if (rx_index < 3) return;
            if (rx_buf[1] != 0x55u || rx_buf[2] != RDK_GRAB_CMD)
            { rdk_rx_drop(1); continue; }
            if (rx_index < 7) return;
            if (rx_buf[3] != 0x00u ||
                rx_buf[4] != (uint8_t)(RDK_GRAB_CMD + rx_buf[3]) ||
                rx_buf[5] != 0x0Du || rx_buf[6] != 0x0Au)
            {
                rdk_errors++;
                rdk_rx_drop(1);
                continue;
            }
            rdk_grab_request = 1;
            rdk_valid_frames++;
            rdk_rx_drop(7);
        }
        else if ((rdk_ring_enabled || rdk_stack_enabled) && rx_buf[0] == 0xAAu)
        {
            uint8_t length, total, i, check;
            if (rx_index < 2) return;
            if (rx_buf[1] != 0x55u) { rdk_rx_drop(1); continue; }
            if (rx_index < 4) return;
            if (rdk_stack_enabled && rx_buf[2] == RDK_STACK_CMD)
            {
                total = 7u + 7u;   /* CMD+LEN+7载荷+CK+帧尾3 = 14 */
                if (rx_index < total) return;
                check = (uint8_t)(RDK_STACK_CMD + 7u);
                for (i = 0; i < 7; i++) check += rx_buf[4 + i];
                if (rx_buf[11] != check || rx_buf[12] != 0x0Du || rx_buf[13] != 0x0Au ||
                    !rdk_accept_stack(rx_buf + 4, now))
                {
                    rdk_errors++;
                    rdk_rx_drop(1);
                    continue;
                }
                rdk_valid_frames++;
                rdk_rx_drop(total);
                continue;
            }
            length = rx_buf[3];
            if ((rx_buf[2] != RDK_RING_RESULT && rx_buf[2] != RDK_RING_IDENTIFIED) ||
                length < 4 || length > (rx_buf[2] == RDK_RING_IDENTIFIED ? 19u : 16u) ||
                (length - 4u) % (rx_buf[2] == RDK_RING_IDENTIFIED ? 5u : 4u))
            {
                rdk_errors++;
                rdk_rx_drop(1);
                continue;
            }
            total = length + 7u;
            if (rx_index < total) return;
            check = rx_buf[2] + length;
            for (i = 0; i < length; i++) check += rx_buf[4 + i];
            if (rx_buf[4 + length] != check || rx_buf[total - 2] != 0x0Du ||
                rx_buf[total - 1] != 0x0Au || !rdk_accept_ring(rx_buf + 4, length, rx_buf[2], now))
            {
                rdk_errors++;
                rdk_rx_drop(1);
                continue;
            }
            rdk_valid_frames++;
            rdk_rx_drop(total);
        }
        else rdk_rx_drop(1);
    }
}

void RDK_Center_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        uint8_t byte = (uint8_t)USART_ReceiveData(USART3);
        rdk_tap[rdk_tap_wr & 7u] = byte;
        rdk_tap_wr++;
        rdk_rx_bytes++;
        rdk_rx_byte(byte, xTaskGetTickCountFromISR());
    }
    if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)
    {
        (void)USART3->SR;
        (void)USART3->DR;
        rx_index = 0;
        rdk_overruns++;
    }
}

void RDK_Center_Enable(uint8_t enable)
{
    taskENTER_CRITICAL();
    rdk_enabled = enable ? 1 : 0;
    rdk_ring_enabled = 0;
    rdk_stack_enabled = 0;
    memset(&ring_result, 0, sizeof(ring_result));
    memset(&stack_result, 0, sizeof(stack_result));
    rx_index = 0;
    taskEXIT_CRITICAL();
}

void RDK_Ring_Start(void)
{
    RDK_Center_Enable(0);
    RDK_Center_Clear();
    taskENTER_CRITICAL();
    rdk_ring_enabled = 1;
    taskEXIT_CRITICAL();
    rdk_send_mode(RDK_MODE_RING);
}

uint8_t RDK_Ring_Active(void) { return rdk_ring_enabled; }

void RDK_Ring_GetResult(RDK_RingResult *out)
{
    if (!out) return;
    taskENTER_CRITICAL();
    *out = ring_result;
    out->age_ms = out->sequence ?
        (uint32_t)((uint64_t)(xTaskGetTickCount() - ring_last_tick) * 1000u / configTICK_RATE_HZ) :
        0xFFFFFFFFu;
    out->fresh = (uint8_t)(rdk_ring_enabled && out->sequence && out->age_ms <= 300u);
    taskEXIT_CRITICAL();
    if (!out->fresh)
    {
        out->count = out->yaw_valid = 0;
        out->yaw_cdeg = 0;
        memset(out->points, 0, sizeof(out->points));
    }
}

void RDK_Stack_Start(void)
{
    RDK_Center_Enable(0);
    RDK_Center_Clear();
    taskENTER_CRITICAL();
    rdk_stack_enabled = 1;
    taskEXIT_CRITICAL();
    rdk_send_mode(RDK_MODE_STACK);
}

uint8_t RDK_Stack_Active(void) { return rdk_stack_enabled; }

void RDK_Stack_GetResult(RDK_StackResult *out)
{
    if (!out) return;
    taskENTER_CRITICAL();
    *out = stack_result;
    out->age_ms = out->sequence ?
        (uint32_t)((uint64_t)(xTaskGetTickCount() - stack_last_tick) * 1000u / configTICK_RATE_HZ) :
        0xFFFFFFFFu;
    out->fresh = (uint8_t)(rdk_stack_enabled && out->sequence && out->age_ms <= 300u);
    taskEXIT_CRITICAL();
    if (!out->fresh)
    {
        out->flags = 0;
        out->x = 0;
        out->y = 0;
        out->yaw_cdeg = 0;
    }
}

void RDK_Center_Clear(void)
{
    rdk_valid = 0;
    rdk_x = 0;
    rdk_y = 0;
    rdk_last_tick = 0;
    rdk_sequence = 0;
}

void RDK_Center_SendTurntable(void)
{
    rdk_send_mode(RDK_MODE_TURNTABLE);
}

/* AA 55 10 01 02 13 0D 0A: 全物料识别模式(RDK端模式02)。 */
void RDK_Center_SendAllMaterials(void)
{
    rdk_send_mode(RDK_MODE_ALL_MATERIALS);
}

/* 打开/关闭统一夹取触发帧的接收通道(平时全关, 字节直接丢弃)。 */
void RDK_GrabArmEnable(uint8_t enable)
{
    taskENTER_CRITICAL();
    rdk_grab_arm_enabled = enable ? 1u : 0u;
    rdk_grab_request = 0;
    rx_index = 0;
    taskEXIT_CRITICAL();
}

/* 取走触发请求: 1=收到过触发帧(取走即清零)。 */
uint8_t RDK_GrabTakeRequest(void)
{
    uint8_t req;

    taskENTER_CRITICAL();
    req = rdk_grab_request;
    rdk_grab_request = 0;
    taskEXIT_CRITICAL();
    return req;
}

/* AA 55 10 01 05 16 0D 0A: 码垛识别模式(RDK端模式05)。 */
void RDK_Center_SendStackDetect(void)
{
    rdk_send_mode(RDK_MODE_STACK);
}

/* AA 55 10 01 00 11 0D 0A: 退出识别，恢复raw相机画面。 */
void RDK_Center_SendIdle(void)
{
    RDK_Center_Enable(0);
    RDK_Center_Clear();
    rdk_send_mode(RDK_MODE_RAW);
}

/* AA 55 10 02 03 COLOR CHECK 0D 0A: 原料区对位完成后下发第一件夹取物料颜色
   (与RDK端物料追踪指令对齐)。COLOR: 红1黄2蓝3绿4黑5浅蓝6;
   CHECK=(0x10+0x02+0x03+COLOR)&0xFF，如红色=0x16。颜色非法不发。 */
uint8_t RDK_Center_SendGrabColor(uint8_t color)
{
    if (color < 1u || color > 6u) return 0;
    rdk_tx_byte(0xAAu);
    rdk_tx_byte(0x55u);
    rdk_tx_byte(RDK_MODE_CMD);
    rdk_tx_byte(0x02u);
    rdk_tx_byte(0x03u);
    rdk_tx_byte(color);
    rdk_tx_byte((uint8_t)(RDK_MODE_CMD + 0x02u + 0x03u + color));
    rdk_tx_byte(0x0Du);
    rdk_tx_byte(0x0Au);
    return 1;
}

void RDK_Center_Poll(void)
{
    taskENTER_CRITICAL();
    if (rx_index && xTaskGetTickCount() - rx_last_tick > pdMS_TO_TICKS(RDK_PARTIAL_TIMEOUT_MS))
    {
        rx_index = 0;
        rdk_errors++;
    }
    taskEXIT_CRITICAL();
}

uint8_t RDK_CenterValid(void) { return rdk_valid; }
int16_t RDK_CenterX(void) { return rdk_x; }
int16_t RDK_CenterY(void) { return rdk_y; }
uint32_t RDK_CenterAgeMs(void)
{
    TickType_t now;
    TickType_t last = rdk_last_tick;
    if (!rdk_valid) return 0xFFFFFFFFu;
    now = xTaskGetTickCount();
    return (uint32_t)((uint64_t)(now - last) * 1000u / configTICK_RATE_HZ);
}
uint32_t RDK_CenterSequence(void) { return rdk_sequence; }

/* 最近8个收到的原始字节(诊断: 判断RDK发的到底是不是AA 55帧流) */
void RDK_Center_LastBytes(uint8_t out[8])
{
    uint8_t i, wr;
    taskENTER_CRITICAL();
    wr = (uint8_t)(rdk_tap_wr & 7u);
    for (i = 0; i < 8u; i++)
        out[i] = rdk_tap[(wr + i) & 7u];   /*旧->新*/
    taskEXIT_CRITICAL();
}

void RDK_Center_GetDiagnostics(RDK_CenterDiagnostics *out)
{
    if (out == 0) return;
    taskENTER_CRITICAL();
    out->rx_bytes = rdk_rx_bytes;
    out->valid_frames = rdk_valid_frames;
    out->errors = rdk_errors;
    out->overruns = rdk_overruns;
    out->enabled = (uint8_t)(rdk_enabled || rdk_ring_enabled);
    taskEXIT_CRITICAL();
}

void USART3_IRQHandler(void)
{
    RDK_Center_IRQHandler();
}
