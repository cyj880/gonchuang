#ifndef __RDK_CENTER_H
#define __RDK_CENTER_H

#include "stm32f4xx.h"

typedef struct
{
    uint32_t rx_bytes;       /*所有收到的字节，包括接收门关闭时*/
    uint32_t valid_frames;   /*接收门打开且坐标有效的累计帧数*/
    uint32_t errors;         /*帧尾错误或坐标越界*/
    uint32_t overruns;       /*USART硬件接收溢出*/
    uint8_t enabled;
} RDK_CenterDiagnostics;

typedef struct
{
    uint16_t x, y;          /* absolute image pixels, X right / Y down */
    uint8_t id;            /* fixed board 1/2/3; 0 unknown or legacy */
} RDK_RingPoint;

typedef struct
{
    RDK_RingPoint points[3];
    int16_t yaw_cdeg;       /* 0.01 degrees, image line slopes down => positive */
    uint8_t count;
    uint8_t yaw_valid;
    uint8_t fresh;          /* received a valid frame within 300 ms, including count=0 */
    uint8_t identified;     /* 0x13 frame; legacy 0x12 is display-only */
    uint32_t sequence;
    uint32_t received_ms;
    uint32_t age_ms;
} RDK_RingResult;

typedef struct
{
    uint8_t flags;          /* bit0=1看到码垛工位; 0=探照帧(未看到,x/y/yaw全0) */
    uint16_t x, y;          /* 中环圆心像素: x 0~639, y 0~479 */
    int16_t yaw_cdeg;       /* 工位角度x100, 有符号 */
    uint8_t fresh;
    uint32_t sequence;
    uint32_t received_ms;
    uint32_t age_ms;
} RDK_StackResult;

/* RDK圆心链路: USART3, PD8=TX / PD9=RX, 115200-8-N-1. */
void RDK_Center_Init(uint32_t baud);
void RDK_Center_IRQHandler(void);
void RDK_Center_Enable(uint8_t enable);
void RDK_Center_Clear(void);
void RDK_Center_SendTurntable(void); /*AA55模式4：转盘识别，首个圆心前500ms重发*/
void RDK_Center_SendIdle(void);      /*AA55模式0：退出识别，显示相机画面*/
void RDK_Center_SendAllMaterials(void); /*AA55模式2：全物料识别（尚未接入流程）*/
void RDK_Center_SendStackDetect(void);  /*AA55模式5：码垛识别（第二轮暂存/1D通过RDK_Stack_Start启用接收）*/
enum { RDK_GRAB_STATE_READY = 5 };
enum { RDK_GRAB_REGION_NONE, RDK_GRAB_REGION_NEAR, RDK_GRAB_REGION_LEFT, RDK_GRAB_REGION_RIGHT };
typedef struct
{
    uint8_t state, color, region; /* 新版0x11 LEN=3；1中下/2左上/3右上 */
} RDK_GrabReport;
void RDK_GrabArmEnable(uint8_t enable, uint8_t expected_color); /* enable=1时颜色须1~6；关闭传0,0 */
uint8_t RDK_GrabTakeReport(RDK_GrabReport *out); /* 读清；首个匹配READY不被后续帧覆盖 */
void RDK_GrabPeek(uint8_t *ack_seen, RDK_GrabReport *out); /* 只读；无报告state=0xFF */
uint8_t RDK_GrabActive(void);               /*夹取接收通道是否打开*/
uint8_t RDK_Center_SendGrabColor(uint8_t color); /*AA55 10 02 03 COLOR：第一件夹取颜色1~6，非法返回0*/
void RDK_Center_Poll(void);
uint8_t RDK_VisionWaitingFirst(void); /*模式1/4/5首个有效回包前每500ms重发，退出取消。*/
uint8_t RDK_CenterValid(void);
int16_t RDK_CenterX(void);
int16_t RDK_CenterY(void);
uint32_t RDK_CenterAgeMs(void);
uint32_t RDK_CenterSequence(void);
void RDK_Center_GetDiagnostics(RDK_CenterDiagnostics *out);
void RDK_Center_LastBytes(uint8_t out[8]); /* 最近8原始字节hex(诊断) */
void RDK_Ring_Start(void); /* clear old results, enable ring RX, send AA55 mode1 */
void RDK_Stack_Start(void);   /* clear old results, enable stack RX, send AA55 mode5 */
uint8_t RDK_Stack_Active(void);
void RDK_Stack_GetResult(RDK_StackResult *out); /* atomic snapshot; stale cleared */
uint8_t RDK_Ring_Active(void);
void RDK_Ring_GetResult(RDK_RingResult *out); /* atomic snapshot; stale data is cleared */

#endif
