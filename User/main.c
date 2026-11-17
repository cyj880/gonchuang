#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "LED.h"
#include "Light.h"
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
#include "LCD.h"
#include "lcd_task.h"
#include "Encoder_RS485.h"
#include "odom_task.h"
#include "imu.h"
#include "rdk_center.h"
#include "arm_m2006.h"
#include "lift_zdt.h"
#include "belt_zdt.h"

/* ---- 放料夹爪角度调节（180°舵机，单位：度，范围0~180；修改后重新编译烧录）----
   两轮粗加工、两轮暂存及网页1C放料/取回共用；原料区1A/1E抓取角度独立。
   当前角度越大夹得越紧：夹紧力度不足可适当增大，松不开可适当减小松开角度。
   放料到位后先稳定等待800ms，直接到松开角度并等400ms。 */
#define PLACE_CLAW_CLOSE_DEG   87.5f  /* 非原料区夹紧角：原79.5°加8°；原料区参数独立 */
#define PLACE_CLAW_OPEN_DEG    30.0f  /* 粗加工/暂存放料及取回张开角：36°→30°；原料区参数独立 */

/* 导出给motor_task.c使用，只需修改上方两个#define。 */
const float g_place_claw_close_deg = PLACE_CLAW_CLOSE_DEG;
const float g_place_claw_open_deg  = PLACE_CLAW_OPEN_DEG;

/* 陀螺仪选择：0=HWT101（当前选择，115200，使用上位机已配置的100Hz）；
   1=轮趣FDILink（921600，保留原模块输出设置）。修改后重新编译并上电生效。
   两款共用USART6/PG9，实际只接所选一款；HWT101仅接TX到PG9，共地，无需模块RX线。
   上电锁存此选择，运行中修改变量不会切换，避免行驶中改变航向基准。 */
uint8_t IMU_Select = IMU_SOURCE_HWT101;

/* 丝杆(张大头ID6)上电自动回零开关：0=关闭(默认)，跳过回零直接开机；
   1=开启，开机阻塞回零，失败时不启动任务。修改后重新编译烧录，下次上电生效。
   跳过回零不会自动设置零点，执行绝对定位前需先确认或校准丝杆零点。 */
uint8_t Lift_AutoHome_RunFlag = 1;

/* 上电默认任务码：156+123+516+231。接口接收12位数字，不含加号；后续扫码或1B注入可覆盖。 */
static const uint8_t startup_task_digits[] = "156123516231";

/* Only used before the scheduler starts (or if it fails to start). Do not
 * continue with a missing motor/odometry/LCD task and an unchanged boot screen. */
static void startup_failed(const char *reason, const char *name)
{
    LCD_Clear();
    LCD_SetColor(LCD_RED);
    LCD_Printf(0, 0, LCD_8X16, "%s", reason);
    LCD_Printf(0, 24, LCD_8X16, "%s", name);
    LCD_Printf(0, 48, LCD_6X8, "Heap free: %lu", (unsigned long)xPortGetFreeHeapSize());
    LED_Green_Off();
    for (;;)
    {
        LED_Red_Toggle();
        Delay_ms(250);
    }
}

static void startup_create_task(TaskFunction_t entry, const char *name,
                                configSTACK_DEPTH_TYPE stack_words, UBaseType_t priority)
{
    if (xTaskCreate(entry, name, stack_words, NULL, priority, NULL) != pdPASS)
        startup_failed("Task create FAIL", name);
}
/**
  * STM32F427IIH（大疆 RoboMaster A 板）+ FreeRTOS V11.1.0
  * 任务函数放在 Task/ 文件夹（led_task.c、servo_task.c、 motor_task.c、lcd_task.c），
  * 任务创建与外设初始化直接在 main.c 中完成：
  *   1. LED 流水灯 —— PG1~PG8（低电平点亮），每颗亮 500ms
  *   2. 舵机扫描   —— PA1（TIM5_CH2）0↔180° 往复，每 100ms 走 10°
  *   3. 坐标链路   —— 接收走 UART7（PE7=RX），发送走 UART8（PE1=TX），115200-8N1，
  *   *      接收坐标帧（AA 55 01 BATCH N ... SUM 0D 0A），解析成功后打印回上位机
 *   6. 摄像头 + 任务码显示屏 —— 一根 UART8（PE0=RX / PE1=TX）同时接两边：
 *          PE0 ← MaixCam 的 TX：收二维码内容(任务码)（帧格式 AA 55 | LEN | 内容 | SUM，
 *                只发内容不回传坐标；没码静默，CAM_FRESH_MS 内没新帧=无码）
 *          PE1 → 任务码显示屏(F103) 的 USART2_RX：转发任务码 + 搬运信息
 *                （任务码行 "426+213+432+123"；搬运信息行 "S,抓取正确,放置正确,当前载物"）
 *          摄像头只发、显示屏只收，所以一根口就够，不会打架；
 *          显示屏原来的回传线 PA2 不用接（任务码现在由摄像头直接给主控）
 *          接口都在 Hardware/uart8.c：
 *            任务码 UART8_GetCode()/UART8_Color()/UART8_Ring()/UART8_StackRing()/UART8_NewCode()
 *            链路   UART8_CamHasTarget()/UART8_CamFrames()
 *            上报   UART8_GrabOk()/UART8_PlaceOk()/UART8_LoadInc()/UART8_SetLoad()
  *   4. 四轮电机   —— CAN2（PB12=RX/PB13=TX，1Mbps）驱动 4 路 Emm_V5 闭环步进，
  *          一次调用下发四轮速度并同步起步（Task/motor_task.c）
 *   5. TFT-LCD    —— 1.8寸 ST7735S（硬件SPI1+DMA2_Stream3，5.25MHz）：PB3=SCK / PA7=SDA /
 *          PB9=DC / PB0=RES / PA6=CS / PI2=BL（背光），驱动见 Hardware/LCD.c
 *          三页显示：页面0 = UART7 路线坐标，页面1 = UART8 收回来的任务码 + 上报计数，
 *          页面2 = 编码器里程计调试页；A 板 PB2 按键切换页面（KeyTask 回调里调 LCD_TogglePage()）
 *          ★ main.c 必须调用 LCD_Init()（曾因该调用丢失导致"背光常亮+整屏纯白"）
 *   6. 编码器里程计 —— 欧艾迪 RS485 绝对值编码器（OID-R3806D-17S1S，17bit单圈，Modbus-RTU）：
 *          编码器 白(485A)/绿(485B) -> RS485收发模块 -> USART2（PD5=TX / PD6=RX，蓝牙口）；
 *          模块 DE+RE 短接 -> PA4（方向控制：高=发送，低=接收）。115200 轮询读"虚拟多圈位置"
 *          （32位，掉电归零，无累计误差），Task/odom_task.c 解算里程；
 *          仅保留站号1里程计，不再轮询站号2；标定见 odom_task.c 顶部
  *
  * 注意：SysTick / PendSV / SVC 三个异常已由 FreeRTOS 接管
  * （见 FreeRTOSConfig.h 尾部宏重定义与 stm32f4xx_it.c 说明），
  * 不要再在 it.c 中定义这三个同名函数。
  */

/* Keil Watch: 0=success; other values are LiftHomeResult_t errors. */
volatile LiftHomeResult_t g_lift_home_result = LIFT_HOME_NOT_READY;

int main(void)
{
    /* ★中断优先级分组必须最先设置（4位抢占）！
       LCD 的 DMA 中断、CAN/串口中断都依赖它：若在分组设置前调用 NVIC_Init，
       SPL 按复位默认分组(PRIGROUP=0)计算会得到优先级 0（最高紧急级），
       该中断一触发就违反 FreeRTOS 的 configMAX_SYSCALL 限制 → configASSERT 卡死 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* 时钟已由 startup 文件中的 SystemInit() 配置为 168MHz（HSE 12MHz） */
    Light_Init();          /* PI6灯光PWM，上电默认60%占空比 */
    LED_Init();
    LED_Green_On();         /* 板载绿色指示灯 PF14 常亮 */
    LED_Red_On();           /* 板载红色指示灯 PE11 常亮 */
    Servo_Init();			/* 180°舵机: PA1(TIM5_CH2) */
    Servo270_Init();        /* 270°舵机: PA0(TIM5_CH1, 蓝排针丝印TIM2_CH1), 须在Servo_Init()后 */
	LCD_Init();
    board_init();           /* CAN2 初始化：PB12=RX / PB13=TX，1Mbps，供 4 路 Emm_V5 电机使用 */
    Arm_M2006_Init();       /* 夹爪伸出电机：CAN1 PD0/PD1，C610 ID1，首次反馈记为0度 */
    UART7_ZigBee_Init(115200); /* 上位机坐标串口：PE7=RX收坐标帧 / PE8=TX回传网页(命令字02帧)，收发双向 */   //ZigBee链路：接收电脑发送的坐标，并回传给网页端显示
    UART8_LinkInit();       /* 摄像头帧解析 + 任务码解析 + 上报变量初始化 */
    (void)UART8_InjectTask(startup_task_digits); /* 开机即可使用默认物料/环号计划 */
    UART8_Init(115200);     /* 默认任务码发布后再启用接收，后续扫码可覆盖 */
    IMU_Init(IMU_Select);  /* 只初始化所选陀螺仪；共享USART6/PG9，选择在上电时锁存。 */
    route_rx_init();        /* UART7发送互斥锁: ACK帧(电机任务)与02回传帧(LCD任务)防交错 */
    Encoder_RS485_Init();   /* RS485绝对值编码器: USART2(PD5=TX/PD6=RX) + PA4方向脚, Modbus轮询站号1 */                           
    RDK_Center_Init(115200); /* RDK圆心链路: USART3(PD8=TX/PD9=RX) */

    /* 上电提示音：蜂鸣器（PH6, TIM12_CH1, 2700Hz）鸣叫1s后关闭。
       此处调度器尚未启动，Delay_ms使用DWT；运行期调用则让出RTOS调度。 */
    Delay_Init(168);        /* DWT独立计时，不修改FreeRTOS使用的SysTick */
	Buzzer_Init();                              

    /* 丝杆地址6：Lift_AutoHome_RunFlag=1 时 CW向上碰撞回零，确认当前位置清零后才创建任务；
       =0 时跳过回零，直接创建任务开机。 */
    if (Lift_AutoHome_RunFlag)
    {
        LCD_Clear();
        LCD_ShowString(0, 0, "Lift CW homing", LCD_8X16);
        LCD_ShowString(0, 20, "CAN2 ID:6", LCD_8X16);
        g_lift_home_result = Lift_ZDT_HomeBeforeScheduler(LIFT_ZDT_ADDR);
        if (g_lift_home_result != LIFT_HOME_OK)
        {
            LCD_SetColor(LCD_RED);
            LCD_ShowString(0, 40, "Home failed:", LCD_8X16);
            LCD_ShowNum(104, 40, (uint32_t)g_lift_home_result, 1, LCD_8X16);
            LCD_ShowString(0, 60, "CMD:", LCD_8X16);
            LCD_ShowHexNum(40, 60, g_lift_home_diag.command, 2, LCD_8X16);
            LCD_ShowString(0, 80, "RET:", LCD_8X16);
            LCD_ShowHexNum(40, 80, g_lift_home_diag.reply, 2, LCD_8X16);
            LCD_ShowString(0, 100, "LEN:", LCD_8X16);
            LCD_ShowNum(40, 100, g_lift_home_diag.length, 1, LCD_8X16);
            LCD_ShowString(0, 120, "ORG:", LCD_8X16);
            LCD_ShowHexNum(40, 120, g_lift_home_diag.origin_flags, 2, LCD_8X16);
            LCD_ShowString(0, 140, "MOT:", LCD_8X16);
            LCD_ShowHexNum(40, 140, g_lift_home_diag.motor_flags, 2, LCD_8X16);
            LED_Green_Off();
            /* 回零失败时已尝试退出回零并急停，禁止启动调度器。 */
            for (;;)
            {
                LED_Red_Toggle();
                Delay_ms(250);
            }
        }
        LCD_ShowString(0, 40, "Lift zero OK", LCD_8X16);
    }
    else
    {
        LCD_Clear();
        LCD_ShowString(0, 0, "Lift home off", LCD_8X16);
    }

    /* 同步带(地址5)不自动回零: 零点由上电时机构所在位置决定, 手动校准。 */


//    xTaskCreate(LED_FlowTask, "LED_Flow", 256, NULL, 2, NULL);
    /* 270°舵机上电由 Servo270_Init() 直接输出60°(≈944us)并保持，不再创建轮转任务；
       路线到原料区时由 MotorTask 转到0°工位；需恢复三工位循环时取消下行注释即可 */
//    xTaskCreate(Servo_Task, "Servo", 256, NULL, 2, NULL);  /*优先级3(高于RS485忙等): 20ms拍不被调度抖动拉长*/ /* 270°转盘对位:30/150/270°三工位循环,每3s一步;去程三次缓动,270→30回程五次缓动(加速度连续) */
    /* RouteTask 已并入 LCD_Task（收帧回传 + 坐标/任务码两页轮显），不再单独创建 */
    LCD_ShowString(0, 60, "Creating tasks...", LCD_6X8);
    startup_create_task(KeyTask, "Key", 256, 2);       /* PB2按键 */
    startup_create_task(MotorTask, "Motor", 2048, 2);  /* 四轮电机 */
    startup_create_task(Arm_M2006_Task, "Arm2006", 256, 3); /* 100Hz RM控制 */
    startup_create_task(OdomTask, "Odom", 2048, 3);    /* 连续RS485里程计 */
    startup_create_task(LCD_Task, "LCD", 1024, 1);     /* TFT显示 */
//    UART8_ReportTask 不在此创建：由 MotorTask 在到扫码区开始找码时动态创建
//    (qr_report_handle)，之后全程保留1s轮询转发任务码/状态行——开关见
//    motor_task.h 的 MOTOR_ROUTE_QR_SCAN_ENABLE（当前=1，到扫码区自动创建）。

    LCD_ShowString(0, 72, "Starting RTOS...", LCD_6X8);
    vTaskStartScheduler();

    /* 正常情况下不会执行到这里；若堆内存不足导致调度器启动失败则停在此处 */
    startup_failed("RTOS start FAIL", "Scheduler returned");
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
