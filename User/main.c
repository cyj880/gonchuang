#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "LED.h"
#include "Servo.h"
#include "Buzzer.h"
#include "Delay.h"
#include "led_task.h"
#include "servo_task.h"
#include "key_task.h"
#include "uart7.h"
#include "uart8.h"
#include "motor_task.h"
#include "board.h"
#include "bno08x_uart_rvc.h"
#include "LCD.h"
#include "lcd_task.h"
#include "Encoder_RS485.h"
#include "odom_task.h"
#include "lunqu_imu.h"


/* ---- 陀螺仪选择开关: 两陀螺仪物理共用USART6/PG9, 同一时刻只插一个; 改后重新上电, 只能一个为1 ---- */
uint8_t Bno085_RunFlag = 0;    /*1=BNO085 RVC@115200*/
uint8_t lunqu_RunFlag  = 1;    /*1=轮趣FDI FDILink@921600*/
/**
  * STM32F427IIH（大疆 RoboMaster A 板）+ FreeRTOS V11.1.0
  * 任务函数放在 Task/ 文件夹（led_task.c、servo_task.c、motor_task.c、lcd_task.c），
  * 任务创建与外设初始化直接在 main.c 中完成：
  *   1. LED 流水灯 —— PG1~PG8（低电平点亮），每颗亮 500ms
  *   2. 舵机扫描   —— PA1（TIM5_CH2）0↔180° 往复，每 100ms 走 10°
  *   3. 坐标链路   —— 接收走 UART7（PE7=RX），发送走 UART8（PE1=TX），115200-8N1，
  *   *      接收坐标帧（AA 55 01 BATCH N ... SUM 0D 0A），解析成功后打印回上位机
  *   6. 摄像头 + 任务码显示屏 —— 一根 UART8（PE0=RX / PE1=TX）同时接两边：
  *          PE0 ← MaixCam 的 TX：收二维码内容(任务码) + 码中心横坐标 x
  *                （帧格式 AA 55 | LEN | 内容 | x低 | x高 | SUM，x=0xFFFF 表示画面里没码）
  *          PE1 → 任务码显示屏(F103) 的 USART2_RX：转发任务码 + 搬运信息
  *                （任务码行 "426+213+432+123"；搬运信息行 "S,抓取正确,放置正确,当前载物"）
  *          摄像头只发、显示屏只收，所以一根口就够，不会打架；
  *          显示屏原来的回传线 PA2 不用接（任务码现在由摄像头直接给主控）
  *          接口都在 Hardware/uart8.c：
  *            任务码 UART8_GetCode()/UART8_Color()/UART8_Ring()/UART8_StackRing()/UART8_NewCode()
  *            坐标   UART8_CamX()/UART8_CamHasTarget()/UART8_CamError()  ← 横向纠偏用
  *            上报   UART8_GrabOk()/UART8_PlaceOk()/UART8_LoadInc()/UART8_SetLoad()
  *   4. 四轮电机   —— CAN2（PB12=RX/PB13=TX，500Kbps）驱动 4 路 Emm_V5 闭环步进，
  *          一次调用下发四轮速度并同步起步（Task/motor_task.c）
 *   5. TFT-LCD    —— 1.8寸 ST7735S（硬件SPI1+DMA2_Stream3，5.25MHz）：PB3=SCK / PA7=SDA /
 *          PB9=DC / PB0=RES / PA6=CS / PI2=BL（背光），驱动见 Hardware/LCD.c
 *          三页显示：页面0 = UART7 路线坐标，页面1 = UART8 收回来的任务码 + 上报计数，
 *          页面2 = 编码器里程计调试页；A 板 PB2 按键切换页面（KeyTask 回调里调 LCD_TogglePage()）
 *          ★ main.c 必须调用 LCD_Init()（曾因该调用丢失导致"背光常亮+整屏纯白"）
 *   6. 编码器里程计 —— 欧艾迪 RS485 绝对值编码器（OID-R3806D-17S1S，17bit单圈，Modbus-RTU）：
 *          编码器 白(485A)/绿(485B) -> RS485收发模块 -> USART2（PD5=TX / PD6=RX，蓝牙口）；
 *          模块 DE+RE 短接 -> PA4（方向控制：高=发送，低=接收）。9600 轮询读"虚拟多圈位置"
 *          （32位，掉电归零，无累计误差），Task/odom_task.c 解算里程；
 *          站号1当前唯一，第二只改站号2后把 ODOM_ENC_NUM 改成2即可；标定见 odom_task.c 顶部
  *
  * 注意：SysTick / PendSV / SVC 三个异常已由 FreeRTOS 接管
  * （见 FreeRTOSConfig.h 尾部宏重定义与 stm32f4xx_it.c 说明），
  * 不要再在 it.c 中定义这三个同名函数。
  */

int main(void)
{
    /* ★中断优先级分组必须最先设置（4位抢占）！
       LCD 的 DMA 中断、CAN/串口中断都依赖它：若在分组设置前调用 NVIC_Init，
       SPL 按复位默认分组(PRIGROUP=0)计算会得到优先级 0（最高紧急级），
       该中断一触发就违反 FreeRTOS 的 configMAX_SYSCALL 限制 → configASSERT 卡死 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* 时钟已由 startup 文件中的 SystemInit() 配置为 168MHz（HSE 12MHz） */
    LED_Init();
    LED_Green_On();         /* 板载绿色指示灯 PF14 常亮 */
    LED_Red_On();           /* 板载红色指示灯 PE11 常亮 */
    Servo_Init();			/* 180°舵机: PA1(TIM5_CH2) */
    Servo270_Init();        /* 270°舵机: PA0(TIM5_CH1, 蓝排针丝印TIM2_CH1), 须在Servo_Init()后 */
	LCD_Init();
    board_init();           /* CAN2 初始化：PB12=RX / PB13=TX，500Kbps，供 4 路 Emm_V5 电机使用 */
    UART7_ZigBee_Init(115200); /* 上位机坐标串口：PE7=RX收坐标帧 / PE8=TX回传网页(命令字02帧)，收发双向 */   //ZigBee链路：接收电脑发送的坐标，并回传给网页端显示
    UART8_Init(115200);     /* 摄像头(PE0) + 任务码显示屏(PE1)，见 Hardware/uart8.c */
    UART8_LinkInit();       /* 摄像头帧解析 + 任务码解析 + 上报变量初始化 */
    if (lunqu_RunFlag)        Lunqu_Init(921600);   /*轮趣FDI惯导: PG9=RX, FDILink@921600*/
    else if (Bno085_RunFlag)  BNO08X_Init(115200);  /*BNO085: PG9=RX, RVC@115200*/
    route_rx_init();        /* UART7发送互斥锁: ACK帧(电机任务)与02回传帧(LCD任务)防交错 */
    Encoder_RS485_Init();   /* RS485绝对值编码器: USART2(PD5=TX/PD6=RX) + PA4方向脚, Modbus轮询站号1 */

    /* 上电提示音：蜂鸣器（PH6, TIM12_CH1, 2700Hz）鸣叫1s后关闭。
       此处调度器尚未启动，SysTick 空闲，可安全使用阻塞式 Delay_ms */
    Delay_Init(168);        /* SYSCLK=168MHz，SysTick HCLK/8 计时 */
	Buzzer_Init();

//    xTaskCreate(LED_FlowTask, "LED_Flow", 256, NULL, 2, NULL);
    xTaskCreate(Servo_Task, "Servo", 256, NULL, 2, NULL); /* 270°转盘对位:15/135/255°三工位循环,每3s一步;去程三次缓动,255→15回程五次缓动(加速度连续) */
    /* RouteTask 已并入 LCD_Task（收帧回传 + 坐标/任务码两页轮显），不再单独创建 */
    xTaskCreate(KeyTask, "Key", 256, NULL, 2, NULL);   /* PB2按键：松开翻转红色LED */
    xTaskCreate(MotorTask, "Motor", 512, NULL, 2, NULL); /* 四轮 Emm_V5 电机（CAN2） */
    xTaskCreate(OdomTask, "Odom", 1024, NULL, 3, NULL);  /* RS485编码器里程计(双轮X/Y), 优先级2防忙等饿死 */
    xTaskCreate(LCD_Task, "LCD", 1024, NULL, 1, NULL);    /* TFT-LCD 显示（SPI1+DMA） */
    xTaskCreate(UART8_ReportTask, "TLink", 256, NULL, 2, NULL); /* 摄像头链路：任务码一变就转发给显示屏，
                                                                  搬运信息一变也转发（不做周期心跳）
                                                                  不需要自动转发可以不创建这个任务，
                                                                  改在需要的地方直接调 UART8_SendTaskCode()/UART8_SendStat() */

    vTaskStartScheduler();

    /* 正常情况下不会执行到这里；若堆内存不足导致调度器启动失败则停在此处 */
    for (;;)
    {
    }
}


/* configSUPPORT_STATIC_ALLOCATION=1 时内核需要的内存回调 */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize )
{
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

/* configCHECK_FOR_STACK_OVERFLOW=2 时内核需要的栈溢出钩子：关中断并停机 */
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    (void) xTask;
    (void) pcTaskName;
    taskDISABLE_INTERRUPTS();
    for( ;; );
}
