#include "stm32f4xx.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "LCD.h"
#include "LCD_Data.h"

/*
 * 1.8寸TFT-LCD（ST7735S，128*160）驱动 —— 已移植到 RoboMaster A 型开发板（STM32F427IIH6）
 *
 * A 板接线（走 A 板 1.17 节 J17 排针 + 1.16 节 8路PWM排针）：
 *   模块引脚    F427引脚    说明
 *   SCL/SCK ->  PB3     SPI1_SCK（AF5；复位后默认是JTAG的JTDO，F4无SWJ配置位，
 *                       直接把AFR配成AF5即可，SWD调试口PA13/PA14不受影响）
 *   SDA     ->  PA7     SPI1_MOSI（AF5）
 *   DC      ->  PB9     数据/命令选择，普通推挽输出（A板 J17 排针的 DC 位）
 *   RST/RES ->  PB0     复位，普通推挽输出（A板 J17 排针的 RST 位）
 *   CS      ->  PA6     片选，普通推挽输出（A板 J17 排针的按键AD位，此处改当片选）
 *   BL/BLK  ->  PI2     背光控制，普通推挽输出（A板 1.16 节 8路PWM排针的 TIM8_CH4，
 *                       以后想调光可把 PI2 配成 TIM8_CH4 的 PWM 输出）
 *   VCC  ->  3.3V      GND -> GND
 * 说明：LCD 只写不读，MISO 不接；PA6 虽与 SPI1_MISO 同位置，但未配成复用，不会干扰 SPI。
 *
 * 与 F407 版本的差异：仅"引脚宏定义"与"LCD_Init 里的 GPIO/复用配置"两处，
 * 其余（ST7735S初始化序列、DMA搬运、互斥锁、绘图/字符函数）与原例程完全一致。
 *
 * 发送方式：批量像素（填充、字符、图像）通过DMA2数据流3（SPI1_TX）搬运，
 *           配合FreeRTOS任务通知阻塞等待，DMA期间不占用CPU，不拖慢其他任务；
 *           少量数据（命令、区域设置、单点）仍用轮询发送
 * 线程安全：所有上层函数内部使用递归互斥锁，可从多个任务并发调用（自动排队）；
 *           LCD_Init在调度器启动前调用时自动退化为忙等方式，无需特殊处理
 */

/*前景色与背景色（全局变量，供所有上层函数使用）*/
static uint16_t LCD_Color = LCD_WHITE;					//当前前景色，默认白色
static uint16_t LCD_BackgroundColor = LCD_BLACK;		//当前背景色，默认黑色

/*引脚操作宏定义********************/

/*复位脚：手册 1.17 节写的是 OLED_RST(PB10)，而用户板上该排针丝印是 PB0 —— 两种板子都兼容，
  所以同时驱动 PB0 和 PB10。若确认板上 PB10 另有用途，把 LCD_RES_ALSO_PB10 改成 0*/
#define LCD_RES_ALSO_PB10   1

#if LCD_RES_ALSO_PB10
#define LCD_RES_Clr()  do { GPIO_ResetBits(GPIOB, GPIO_Pin_0);  GPIO_ResetBits(GPIOB, GPIO_Pin_10); } while (0)
#define LCD_RES_Set()  do { GPIO_SetBits(GPIOB, GPIO_Pin_0);    GPIO_SetBits(GPIOB, GPIO_Pin_10);   } while (0)
#else
#define LCD_RES_Clr()  GPIO_ResetBits(GPIOB, GPIO_Pin_0)		//复位      PB0
#define LCD_RES_Set()  GPIO_SetBits(GPIOB, GPIO_Pin_0)
#endif

#define LCD_CS_Clr()   GPIO_ResetBits(GPIOA, GPIO_Pin_6)		//片选      PA6
#define LCD_CS_Set()   GPIO_SetBits(GPIOA, GPIO_Pin_6)

#define LCD_DC_Clr()   GPIO_ResetBits(GPIOB, GPIO_Pin_9)		//数据/命令 PB9
#define LCD_DC_Set()   GPIO_SetBits(GPIOB, GPIO_Pin_9)

#define LCD_BLK_Clr()  GPIO_ResetBits(GPIOI, GPIO_Pin_2)		//背光      PI2
#define LCD_BLK_Set()  GPIO_SetBits(GPIOI, GPIO_Pin_2)

/*SCK 引脚选择：
  0 = PB3（J17 排针 SCLK 位；注意 PB3 复位后是 JTDO/TRACESWO——SWO/跟踪开着时会被调试器占用！）
  1 = PA5（J34 排针 DAC_OUT2 位，无 JTAG 历史）
  判定方法：把模块 SCL 线从 J17 的 PB3 位置改插到 J34 的 PA5 位置，这里改成 1 再烧。
  若 PA5 能显示而 PB3 不能 → 问题就是 PB3 被调试器/跟踪占用（关 Trace 或改用 PA5）*/
#define LCD_SCK_USE_PA5            0

#if LCD_SCK_USE_PA5
#define LCD_SCK_PORT                GPIOA
#define LCD_SCK_PIN                 GPIO_Pin_5
#define LCD_SCK_SOURCE              GPIO_PinSource5
#else
#define LCD_SCK_PORT                GPIOB
#define LCD_SCK_PIN                 GPIO_Pin_3
#define LCD_SCK_SOURCE              GPIO_PinSource3
#endif

/*软件SPI用到的两根线（仅 LCD_USE_SOFT_SPI=1 时使用）*/
#define LCD_SCK_Clr()  GPIO_ResetBits(LCD_SCK_PORT, LCD_SCK_PIN)
#define LCD_SCK_Set()  GPIO_SetBits(LCD_SCK_PORT, LCD_SCK_PIN)
#define LCD_MOSI_Clr() GPIO_ResetBits(GPIOA, GPIO_Pin_7)		//MOSI/SDA  PA7
#define LCD_MOSI_Set() GPIO_SetBits(GPIOA, GPIO_Pin_7)

/*********************引脚操作宏定义*/

/*DMA发送配置*********************/

/*SPI1_TX对应DMA2数据流3、通道3*/
#define LCD_DMA_STREAM              DMA2_Stream3
#define LCD_DMA_CHANNEL             DMA_Channel_3
#define LCD_DMA_ALL_FLAGS           (DMA_FLAG_FEIF3 | DMA_FLAG_DMEIF3 | DMA_FLAG_TEIF3 | DMA_FLAG_HTIF3 | DMA_FLAG_TCIF3)
#define LCD_DMA_ALL_IT_FLAGS        (DMA_IT_FEIF3 | DMA_IT_DMEIF3 | DMA_IT_TEIF3 | DMA_IT_HTIF3 | DMA_IT_TCIF3)
#define LCD_DMA_IT_SOURCES          (DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE)
#define LCD_DMA_TIMEOUT_MS          100U		/*单次DMA等待超时，仅用于总线异常时自恢复*/
#define LCD_DMA_IRQ_PRIORITY        6U			/*中断优先级数值需大于等于configMAX_SYSCALL（5），才能调用FromISR API*/
#define LCD_DMA_CHUNK_PIXELS        640U		/*DMA发送暂存缓冲的像素数（640像素=1280字节）*/
#define LCD_USE_DMA                 1			/*1使用DMA搬运像素，0退化为纯轮询发送（调试用）*/

/*软件SPI（GPIO 位操作）开关 —— 白屏排查用：
  0 = 正常走硬件 SPI1（默认）
  1 = SCK/MOSI 当普通 IO 位操作，完全绕开 SPI1 外设 / 引脚复用 / DMA
  判定：改成 1 后能出画面 → 问题出在硬件 SPI 这条路（复用/DMA/外设）；
        改成 1 后仍白屏 → 问题在接线或屏本身。*/
#define LCD_USE_SOFT_SPI            0

/*软件SPI半周期延时（空指令条数）：168MHz 下 8 条约 50ns，对应约 5MHz 时钟上限*/
#define LCD_SOFT_SPI_NOP()          { __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); }

/*SPI时钟分频（APB2 = 84MHz）：
  _4=21MHz（F407 原例程值）、_8=10.5MHz、_16=5.25MHz、_32=2.625MHz、_64=1.3125MHz
  ★ A 板必须降速：手册 1.17 节 J17 排针的 5 根信号线上每根都串了一颗 0201 磁珠（约1uH）
    并对地挂了 47pF 电容，LC 谐振点约 23.8MHz —— 21MHz 正好落在谐振区，波形被削顶/振铃，
    ST7735S 收不到有效命令，现象是"背光亮、整屏白"。默认取 _16(5.25MHz)，远低于谐振点。
    若仍然白屏，依次试 _32、_64（屏幕刷新会变慢，但本任务只做局部刷新，无影响）*/
#define LCD_SPI_PRESCALER           SPI_BaudRatePrescaler_16

#define LCD_DMA_RESULT_PENDING      0U
#define LCD_DMA_RESULT_COMPLETE     1U
#define LCD_DMA_RESULT_ERROR        2U

static TaskHandle_t volatile LCD_DMA_WaitingTask = NULL;		/*等待DMA完成的任务句柄*/
static volatile uint8_t LCD_DMA_Result = LCD_DMA_RESULT_PENDING;
static SemaphoreHandle_t LCD_Mutex = NULL;					/*递归互斥锁，保证多任务调用安全*/
static uint8_t LCD_DMATxBuffer[LCD_DMA_CHUNK_PIXELS * 2];	/*DMA发送暂存缓冲（字节序：每个像素高字节在前）*/

/*********************DMA发送配置*/

/**
  * 函    数：加锁（内部调用）
  * 说    明：调度器未运行时（如main里调用LCD_Init）没有任务上下文，直接跳过；
  *           递归互斥锁允许同一任务内上层函数嵌套调用（如Printf->ShowString->ShowChar）
  */
static void LCD_Lock(void)
{
	if (LCD_Mutex != NULL && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
	{
		xSemaphoreTakeRecursive(LCD_Mutex, portMAX_DELAY);
	}
}

/**
  * 函    数：解锁（内部调用）
  */
static void LCD_Unlock(void)
{
	if (LCD_Mutex != NULL && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
	{
		xSemaphoreGiveRecursive(LCD_Mutex);
	}
}

/**
  * 函    数：LCD粗略延时（毫秒），仅用于初始化阶段的阻塞延时
  * 参    数：ms 延时的毫秒数（168MHz主频下近似值，偏长一些不影响初始化）
  * 返 回 值：无
  */
static void LCD_DelayMs(uint32_t ms)
{
	volatile uint32_t i;
	while (ms --)
	{
		for (i = 0; i < 168000U; i ++);
	}
}

/**
  * 函    数：LCD底层SPI发送一个字节（轮询方式，用于命令等少量数据）
  * 参    数：Byte 要发送的字节
  * 返 回 值：无
  */
static void LCD_SPI_WriteByte(uint8_t Byte)
{
#if LCD_USE_SOFT_SPI
	uint8_t i;
	for (i = 0; i < 8; i ++)										//MSB先行，SCK空闲低、上升沿采样（模式0）
	{
		if (Byte & 0x80) { LCD_MOSI_Set(); } else { LCD_MOSI_Clr(); }
		Byte <<= 1;
		LCD_SOFT_SPI_NOP();
		LCD_SCK_Set();												//上升沿
		LCD_SOFT_SPI_NOP();
		LCD_SCK_Clr();
		LCD_SOFT_SPI_NOP();
	}
#else
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) != SET);	//等待发送数据寄存器空
	SPI_I2S_SendData(SPI1, Byte);									//写入数据到发送数据寄存器，开始产生时序
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) != SET);	//等待接收数据寄存器非空（全双工模式下接收完成即发送完成）
	(void)SPI_I2S_ReceiveData(SPI1);								//读取接收寄存器，清除标志位
#endif
}

/** 等待SPI完全空闲（发送寄存器空且移位完成） */
static void LCD_SPI_WaitIdle(void)
{
#if !LCD_USE_SOFT_SPI
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) != SET);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);
#endif
}

/** 等待最后一个字节完全发送后结束当前SPI事务 */
static void LCD_SPI_End(void)
{
	LCD_SPI_WaitIdle();
	LCD_CS_Set();
}

/** 在已经拉低CS的事务内发送一个RGB565颜色值 */
static void LCD_SPI_WriteData16(uint16_t Data)
{
	LCD_SPI_WriteByte((uint8_t)(Data >> 8));
	LCD_SPI_WriteByte((uint8_t)Data);
}

/**
  * 函    数：LCD写入一个字节数据
  * 参    数：Data 要写入的数据
  * 返 回 值：无
  */
static void LCD_WR_DATA8(uint8_t Data)
{
	LCD_CS_Clr();
	LCD_DC_Set();
	LCD_SPI_WriteByte(Data);
	LCD_SPI_End();
}

/**
  * 函    数：LCD写入命令
  * 参    数：Reg 要写入的命令
  * 返 回 值：无
  */
static void LCD_WR_REG(uint8_t Reg)
{
	LCD_CS_Clr();
	LCD_DC_Clr();						//DC拉低，写命令
	LCD_SPI_WriteByte(Reg);
	LCD_SPI_End();
	LCD_DC_Set();						//DC拉高，恢复写数据
}

/** 在一次片选事务中发送命令及两个16位参数 */
static void LCD_WR_REG_DATA16X2(uint8_t Reg, uint16_t Data1, uint16_t Data2)
{
	LCD_CS_Clr();
	LCD_DC_Clr();
	LCD_SPI_WriteByte(Reg);
	LCD_DC_Set();
	LCD_SPI_WriteData16(Data1);
	LCD_SPI_WriteData16(Data2);
	LCD_SPI_End();
}

/**
  * 函    数：设置显示区域（起始和结束地址）
  * 参    数：x1,y1 区域的起始坐标；x2,y2 区域的结束坐标
  * 返回值：无
  */
static void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	if (USE_HORIZONTAL == 0)
	{
		LCD_WR_REG_DATA16X2(0x2A, x1 + 2, x2 + 2);	//列地址设置（与原版例程一致：竖屏0 X+2 Y+1）
		LCD_WR_REG_DATA16X2(0x2B, y1 + 1, y2 + 1);	//行地址设置
	}
	else if (USE_HORIZONTAL == 1)
	{
		LCD_WR_REG_DATA16X2(0x2A, x1 + 2, x2 + 2);	//列地址设置
		LCD_WR_REG_DATA16X2(0x2B, y1 + 1, y2 + 1);	//行地址设置
	}
	else if (USE_HORIZONTAL == 2)
	{
		LCD_WR_REG_DATA16X2(0x2A, x1 + 1, x2 + 1);	//列地址设置
		LCD_WR_REG_DATA16X2(0x2B, y1 + 2, y2 + 2);	//行地址设置
	}
	else
	{
		LCD_WR_REG_DATA16X2(0x2A, x1 + 1, x2 + 1);	//列地址设置
		LCD_WR_REG_DATA16X2(0x2B, y1 + 2, y2 + 2);	//行地址设置
	}

	/*发送显存写命令后保持CS为低，调用者连续发送全部像素并调用LCD_SPI_End结束*/
	LCD_CS_Clr();
	LCD_DC_Clr();
	LCD_SPI_WriteByte(0x2C);
	LCD_DC_Set();
}

#if (LCD_USE_DMA && !LCD_USE_SOFT_SPI)
/** DMA异常时复位SPI和DMA，避免程序永久阻塞 */
static void LCD_DMA_Abort(void)
{
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, DISABLE);
	DMA_Cmd(LCD_DMA_STREAM, DISABLE);
	DMA_ClearFlag(LCD_DMA_STREAM, LCD_DMA_ALL_FLAGS);
	SPI_Cmd(SPI1, DISABLE);					//关闭SPI丢弃未发送完的数据
	(void)SPI_I2S_ReceiveData(SPI1);		//清空接收寄存器
	SPI_Cmd(SPI1, ENABLE);
}

/**
  * 函    数：启动DMA并等待发送完成（数据流已配置好）
  * 说    明：调度器运行时通过任务通知阻塞等待，只在DMA完成/错误中断时唤醒，不消耗CPU；
  *           LCD初始化发生在调度器启动前，此时使用一次性忙等待
  * 返 回 值：1成功，0超时或错误
  */
static uint8_t LCD_DMA_StartAndWait(void)
{
	if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
	{
		TickType_t TimeoutTicks = pdMS_TO_TICKS(LCD_DMA_TIMEOUT_MS);
		uint8_t Result;

		if (TimeoutTicks == 0U)
		{
			TimeoutTicks = 1U;
		}

		/*清掉该任务以前遗留的通知，再公布句柄，防止DMA过快完成造成丢通知*/
		(void)ulTaskNotifyTake(pdTRUE, 0U);
		LCD_DMA_Result = LCD_DMA_RESULT_PENDING;
		LCD_DMA_WaitingTask = xTaskGetCurrentTaskHandle();
		DMA_ClearITPendingBit(LCD_DMA_STREAM, LCD_DMA_ALL_IT_FLAGS);
		DMA_ITConfig(LCD_DMA_STREAM, LCD_DMA_IT_SOURCES, ENABLE);

		/*先使能数据流，再打开SPI的DMA请求，顺序颠倒会丢失请求*/
		DMA_Cmd(LCD_DMA_STREAM, ENABLE);
		SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);

		/*阻塞态不消耗CPU；超时仅用于总线或DMA异常时自恢复*/
		if (ulTaskNotifyTake(pdTRUE, TimeoutTicks) == 0U)
		{
			taskENTER_CRITICAL();
			DMA_ITConfig(LCD_DMA_STREAM, LCD_DMA_IT_SOURCES, DISABLE);
			LCD_DMA_WaitingTask = NULL;
			DMA_ClearITPendingBit(LCD_DMA_STREAM, LCD_DMA_ALL_IT_FLAGS);
			taskEXIT_CRITICAL();
			return 0;
		}

		taskENTER_CRITICAL();
		LCD_DMA_WaitingTask = NULL;
		Result = LCD_DMA_Result;
		taskEXIT_CRITICAL();

		return (Result == LCD_DMA_RESULT_COMPLETE) ? 1U : 0U;
	}
	else
	{
		uint32_t Timeout = 1000000U;

		/*调度器未启动时不能调用FromISR通知API，故关闭DMA中断改为查询标志*/
		DMA_ITConfig(LCD_DMA_STREAM, LCD_DMA_IT_SOURCES, DISABLE);
		DMA_ClearITPendingBit(LCD_DMA_STREAM, LCD_DMA_ALL_IT_FLAGS);

		DMA_Cmd(LCD_DMA_STREAM, ENABLE);
		SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);

		while (DMA_GetFlagStatus(LCD_DMA_STREAM, DMA_FLAG_TCIF3) == RESET)
		{
			if (DMA_GetFlagStatus(LCD_DMA_STREAM, DMA_FLAG_FEIF3) != RESET ||
				DMA_GetFlagStatus(LCD_DMA_STREAM, DMA_FLAG_DMEIF3) != RESET ||
				DMA_GetFlagStatus(LCD_DMA_STREAM, DMA_FLAG_TEIF3) != RESET)
			{
				return 0;
			}
			if (Timeout -- == 0)
			{
				return 0;
			}
		}
	}

	return 1;
}
#endif /* LCD_USE_DMA && !LCD_USE_SOFT_SPI */

/**
  * 函    数：DMA2数据流3中断（SPI1_TX）
  * 说    明：中断中只记录结果并通知等待任务，耗时很短
  */
void DMA2_Stream3_IRQHandler(void)
{
	BaseType_t HigherPriorityTaskWoken = pdFALSE;
	TaskHandle_t WaitingTask;
	uint8_t Result = LCD_DMA_RESULT_PENDING;

	if (DMA_GetITStatus(LCD_DMA_STREAM, DMA_IT_FEIF3) != RESET ||
		DMA_GetITStatus(LCD_DMA_STREAM, DMA_IT_DMEIF3) != RESET ||
		DMA_GetITStatus(LCD_DMA_STREAM, DMA_IT_TEIF3) != RESET)
	{
		Result = LCD_DMA_RESULT_ERROR;
	}
	else if (DMA_GetITStatus(LCD_DMA_STREAM, DMA_IT_TCIF3) != RESET)
	{
		Result = LCD_DMA_RESULT_COMPLETE;
	}

	if (Result != LCD_DMA_RESULT_PENDING)
	{
		DMA_ITConfig(LCD_DMA_STREAM, LCD_DMA_IT_SOURCES, DISABLE);
		DMA_ClearITPendingBit(LCD_DMA_STREAM, LCD_DMA_ALL_IT_FLAGS);
		LCD_DMA_Result = Result;
		WaitingTask = LCD_DMA_WaitingTask;

		if (WaitingTask != NULL)
		{
			vTaskNotifyGiveFromISR(WaitingTask, &HigherPriorityTaskWoken);
			portYIELD_FROM_ISR(HigherPriorityTaskWoken);
		}
	}
}

/**
  * 函    数：DMA方式发送一批字节（内部调用）
  * 参    数：Bytes 字节数据（位于RAM或Flash），像素颜色需按"高字节在前"排布
  * 参    数：Count 字节数，范围：1~65535
  * 返 回 值：1成功，0失败（超时或DMA错误，此时SPI已复位）
  * 说    明：调用前需已设置显示区域且CS为低；SPI全程保持8位模式，不切换数据宽度，
  *           避免DMA期间切换16位模式的时序风险；LCD_USE_DMA置0时退化为纯轮询发送
  */
static uint8_t LCD_DMA_SendBytes(const uint8_t *Bytes, uint32_t Count)
{
#if (LCD_USE_DMA && !LCD_USE_SOFT_SPI)
	DMA_InitTypeDef DMA_InitStructure;
	uint32_t Timeout;

	/*等待数据流允许配置*/
	DMA_Cmd(LCD_DMA_STREAM, DISABLE);
	Timeout = 100000;
	while (DMA_GetCmdStatus(LCD_DMA_STREAM) != DISABLE)
	{
		if (Timeout -- == 0)
		{
			return 0;
		}
	}
	DMA_DeInit(LCD_DMA_STREAM);

	DMA_InitStructure.DMA_Channel = LCD_DMA_CHANNEL;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Bytes;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = Count;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(LCD_DMA_STREAM, &DMA_InitStructure);
	DMA_ClearFlag(LCD_DMA_STREAM, LCD_DMA_ALL_FLAGS);

	if (!LCD_DMA_StartAndWait())
	{
		LCD_DMA_Abort();
		return 0;
	}

	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, DISABLE);
	DMA_Cmd(LCD_DMA_STREAM, DISABLE);
	DMA_ClearFlag(LCD_DMA_STREAM, LCD_DMA_ALL_FLAGS);

	LCD_SPI_WaitIdle();						//等待最后一个字节移位完成
	(void)SPI_I2S_ReceiveData(SPI1);		//DMA期间收到无用回环数据，读DR清除RXNE/OVR，保证后续轮询可靠
	return 1;
#else
	uint32_t i;
	for (i = 0; i < Count; i ++)			//DMA关闭时退化为轮询发送
	{
		LCD_SPI_WriteByte(Bytes[i]);
	}
	return 1;
#endif
}

/**
  * 函    数：LCD初始化（GPIO、SPI1、DMA和ST7735S初始化序列）
  * 参    数：无
  * 返 回 值：无
  */
void LCD_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/*开启时钟*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	//开启GPIOA的时钟（PA7=SPI1_MOSI、PA6=CS）
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	//开启GPIOB的时钟（PB3=SPI1_SCK、PB9=DC、PB0=RES）
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);	//开启GPIOI的时钟（PI2=BL 背光）
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);	//开启DMA2的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);	//开启SPI1的时钟（SPI1挂在APB2上）

	/*GPIO初始化*/
	/*PB0(RES)、PB10(RES 备用，兼容手册版接线)、PB9(DC)：普通推挽输出*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*PI2(BL 背光)：普通推挽输出（A板 8路PWM排针的 TIM8_CH4，与 SPI 无关）*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOI, &GPIO_InitStructure);

	/*PA6(CS)：普通推挽输出*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

#if LCD_USE_SOFT_SPI
	/*软件SPI模式：SCK、PA7(MOSI) 当普通推挽输出，不配复用*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = LCD_SCK_PIN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LCD_SCK_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	LCD_SCK_Clr();
	LCD_MOSI_Clr();
#else
	/*SCK：SPI1复用推挽输出（PB3 或 PA5，见 LCD_SCK_USE_PA5）*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = LCD_SCK_PIN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LCD_SCK_PORT, &GPIO_InitStructure);

	/*PA7(MOSI/SDA)：SPI1复用推挽输出*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*引脚复用映射：SCK→SPI1_SCK、PA7→SPI1_MOSI（AF5）
	  PB3复位后默认被JTAG的JTDO/TRACESWO占用，STM32F4没有F1那样的SWJ配置寄存器，
	  只要把PB3的复用功能改写成AF5即释放；SWD调试口PA13/PA14全程未动。
	  ★注意：Keil 里若开着 Event Recorder / Trace(SWO)，调试运行期间 PB3 会被跟踪模块
	    占用（TRACESWO），SCK 出不来 → 白屏。要么关 Trace，要么用 PA5 当 SCK。*/
	GPIO_PinAFConfig(LCD_SCK_PORT, LCD_SCK_SOURCE, GPIO_AF_SPI1);	//SCK复用为SPI1_SCK
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);		//PA7复用为SPI1_MOSI
#endif

	/*SPI初始化*/
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;				//模式，选择为SPI主模式
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	//方向，选择2线全双工（LCD只写不读，MISO脚未接LCD，收到数据直接丢弃）
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;			//数据宽度，选择为8位
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;			//先行位，选择高位先行
	SPI_InitStructure.SPI_BaudRatePrescaler = LCD_SPI_PRESCALER;	//波特率分频，见文件顶部 LCD_SPI_PRESCALER（默认21MHz）
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;					//SPI极性，选择低极性
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;				//SPI相位，选择第一个时钟边沿采样，极性和相位决定选择SPI模式0
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;					//NSS，选择由软件控制
	SPI_InitStructure.SPI_CRCPolynomial = 7;					//CRC多项式，暂时用不到，给默认值7
	SPI_Init(SPI1, &SPI_InitStructure);							//将结构体变量交给SPI_Init，配置SPI1

	/*SPI使能*/
	SPI_Cmd(SPI1, ENABLE);										//使能SPI1，开始运行

	/*DMA中断配置：优先级数值需大于等于configMAX_SYSCALL_INTERRUPT_PRIORITY（5）*/
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = LCD_DMA_IRQ_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/*创建递归互斥锁（分配自FreeRTOS堆，仅需约百字节）*/
	if (LCD_Mutex == NULL)
	{
		LCD_Mutex = xSemaphoreCreateRecursiveMutex();
	}

	/*设置通信空闲状态，避免上电初始化期间误选中LCD*/
	LCD_CS_Set();
	LCD_DC_Set();
	LCD_BLK_Clr();
	LCD_RES_Set();

	/*复位LCD*/
	LCD_RES_Clr();												//复位
	LCD_DelayMs(100);
	LCD_RES_Set();
	LCD_DelayMs(100);

	LCD_BLK_Set();												//打开背光
	LCD_DelayMs(100);

	/*ST7735S初始化序列（来自原例程）*/
	LCD_WR_REG(0x11);											//Sleep out
	LCD_DelayMs(120);											//Delay 120ms

	/*ST7735S Frame Rate*/
	LCD_WR_REG(0xB1);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x3C);
	LCD_WR_REG(0xB2);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x3C);
	LCD_WR_REG(0xB3);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x3C);

	LCD_WR_REG(0xB4);											//Dot inversion
	LCD_WR_DATA8(0x03);

	/*ST7735S Power Sequence*/
	LCD_WR_REG(0xC0);
	LCD_WR_DATA8(0x28);
	LCD_WR_DATA8(0x08);
	LCD_WR_DATA8(0x04);
	LCD_WR_REG(0xC1);
	LCD_WR_DATA8(0xC0);
	LCD_WR_REG(0xC2);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x00);
	LCD_WR_REG(0xC3);
	LCD_WR_DATA8(0x8D);
	LCD_WR_DATA8(0x2A);
	LCD_WR_REG(0xC4);
	LCD_WR_DATA8(0x8D);
	LCD_WR_DATA8(0xEE);

	LCD_WR_REG(0xC5);											//VCOM
	LCD_WR_DATA8(0x1A);

	LCD_WR_REG(0x36);											//MX, MY, RGB mode
	if (USE_HORIZONTAL == 0)			LCD_WR_DATA8(0x00);
	else if (USE_HORIZONTAL == 1)		LCD_WR_DATA8(0xC0);
	else if (USE_HORIZONTAL == 2)		LCD_WR_DATA8(0x70);
	else								LCD_WR_DATA8(0xA0);

	/*ST7735S Gamma Sequence*/
	LCD_WR_REG(0xE0);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x22);
	LCD_WR_DATA8(0x07);
	LCD_WR_DATA8(0x0A);
	LCD_WR_DATA8(0x2E);
	LCD_WR_DATA8(0x30);
	LCD_WR_DATA8(0x25);
	LCD_WR_DATA8(0x2A);
	LCD_WR_DATA8(0x28);
	LCD_WR_DATA8(0x26);
	LCD_WR_DATA8(0x2E);
	LCD_WR_DATA8(0x3A);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x01);
	LCD_WR_DATA8(0x03);
	LCD_WR_DATA8(0x13);
	LCD_WR_REG(0xE1);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x16);
	LCD_WR_DATA8(0x06);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x2D);
	LCD_WR_DATA8(0x26);
	LCD_WR_DATA8(0x23);
	LCD_WR_DATA8(0x27);
	LCD_WR_DATA8(0x27);
	LCD_WR_DATA8(0x25);
	LCD_WR_DATA8(0x2D);
	LCD_WR_DATA8(0x3B);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x01);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x13);

	LCD_WR_REG(0x3A);											//65k mode
	LCD_WR_DATA8(0x05);
	LCD_WR_REG(0x29);											//Display on

	LCD_Clear();												//清屏
}

/**
  * 函    数：设置前景色（文字、画笔的颜色）
  * 参    数：Color 前景色，RGB565格式，可用LCD_WHITE等宏，或用RGB565(R,G,B)自定义
  * 返 回 值：无
  */
void LCD_SetColor(uint16_t Color)
{
	LCD_Lock();
	LCD_Color = Color;
	LCD_Unlock();
}

/**
  * 函    数：设置背景色（清屏、文字底色的颜色）
  * 参    数：Color 背景色，RGB565格式
  * 返 回 值：无
  */
void LCD_SetBackgroundColor(uint16_t Color)
{
	LCD_Lock();
	LCD_BackgroundColor = Color;
	LCD_Unlock();
}

/**
  * 函    数：以指定颜色填充矩形区域（内部实现，不加锁）
  * 参    数：X, Y 区域左上角的坐标；Width, Height 区域的宽高；Color 填充颜色
  * 返 回 值：无
  * 说    明：像素通过DMA批量搬运，超过暂存缓冲大小时分块发送，全程CPU阻塞等待不忙等
  */
static void LCD_FillInternal(int16_t X, int16_t Y, int16_t Width, int16_t Height, uint16_t Color)
{
	uint32_t n;
	uint16_t Chunk;
	uint16_t i;
	uint8_t BufferFilled = 0;

	LCD_Address_Set(X, Y, X + Width - 1, Y + Height - 1);		//设置显示区域
	n = (uint32_t)Width * Height;								//计算区域的像素点数

	while (n > 0)
	{
		Chunk = (n > LCD_DMA_CHUNK_PIXELS) ? (uint16_t)LCD_DMA_CHUNK_PIXELS : (uint16_t)n;

		if (!BufferFilled)										//首块生成"高字节在前"的颜色字节序列，后续分块复用
		{														//（分块从大到小，首块填满后必定够用）
			for (i = 0; i < Chunk; i ++)
			{
				LCD_DMATxBuffer[i * 2] = (uint8_t)(Color >> 8);
				LCD_DMATxBuffer[i * 2 + 1] = (uint8_t)Color;
			}
			BufferFilled = 1;
		}

		(void)LCD_DMA_SendBytes(LCD_DMATxBuffer, (uint32_t)Chunk * 2);
		n -= Chunk;
	}
	LCD_SPI_End();
}

/**
  * 函    数：以指定颜色填充矩形区域
  * 参    数：X, Y 区域左上角的坐标
  * 参    数：Width, Height 区域的宽度和高度
  * 参    数：Color 填充的颜色
  * 返 回 值：无
  */
void LCD_Fill(int16_t X, int16_t Y, int16_t Width, int16_t Height, uint16_t Color)
{
	if (Width <= 0 || Height <= 0 || X < 0 || Y < 0 ||
		X + Width > LCD_W || Y + Height > LCD_H)	//尺寸无效或超出屏幕的内容不显示
	{
		return;
	}
	LCD_Lock();
	LCD_FillInternal(X, Y, Width, Height, Color);
	LCD_Unlock();
}

/**
  * 函    数：LCD清屏，以背景色填充全屏
  * 参    数：无
  * 返回值：无
  */
void LCD_Clear(void)
{
	LCD_Lock();
	LCD_FillInternal(0, 0, LCD_W, LCD_H, LCD_BackgroundColor);
	LCD_Unlock();
}

/**
  * 函    数：以背景色填充指定区域
  * 参    数：X, Y 区域左上角的坐标
  * 参    数：Width, Height 区域的宽度和高度
  * 返回值：无
  */
void LCD_ClearArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height)
{
	LCD_Lock();
	if (Width > 0 && Height > 0 && X >= 0 && Y >= 0 &&
		X + Width <= LCD_W && Y + Height <= LCD_H)
	{
		LCD_FillInternal(X, Y, Width, Height, LCD_BackgroundColor);
	}
	LCD_Unlock();
}

/**
  * 函    数：LCD在指定位置画一个指定颜色的点（供上层函数调用，带边界检查）
  * 参    数：X, Y 点的坐标
  * 参    数：Color 点的颜色
  * 返 回 值：无
  */
static void LCD_DrawPoint_Color(int16_t X, int16_t Y, uint16_t Color)
{
	if (X >= 0 && X < LCD_W && Y >= 0 && Y < LCD_H)		//超出屏幕的内容不显示
	{
		LCD_Address_Set(X, Y, X, Y);					//设置单点区域
		LCD_SPI_WriteData16(Color);					//写入颜色
		LCD_SPI_End();
	}
}

/**
  * 函    数：LCD显示单个字符
  * 参    数：X 指定字符左上角的横坐标，范围：-32768~32767，屏幕区域：0~127
  * 参    数：Y 指定字符左上角的纵坐标，范围：-32768~32767，屏幕区域：0~159
  * 参    数：Char 指定要显示的字符，范围：ASCII码可见字符
  * 参    数：FontSize 指定字体大小
  *           范围：LCD_8X16		宽8像素，高16像素
  *                 LCD_6X8		宽6像素，高8像素
  * 返 回 值：无
  * 说    明：字符的笔画使用当前前景色，底使用当前背景色（分别由LCD_SetColor/LCD_SetBackgroundColor设置）
  */
void LCD_ShowChar(int16_t X, int16_t Y, char Char, uint8_t FontSize)
{
	uint8_t i, j, k, Data;
	uint16_t ForeColor, BackColor;
	uint16_t Pixel;
	uint16_t Index = 0;

	/*字库仅包含ASCII 0x20~0x7E，非法字符统一显示为问号*/
	if ((uint8_t)Char < (uint8_t)' ' || (uint8_t)Char > (uint8_t)'~')
	{
		Char = '?';
	}

	LCD_Lock();

	ForeColor = LCD_Color;								//一次性取出颜色，避免渲染过程中被其他任务修改
	BackColor = LCD_BackgroundColor;

	if (FontSize == LCD_8X16)		//字体为宽8像素，高16像素
	{
		if (X >= 0 && Y >= 0 && X + 8 <= LCD_W && Y + 16 <= LCD_H)
		{
			/*字符完整位于屏幕内：先在RAM中展开为像素字节（高字节在前），再一次DMA发送*/
			LCD_Address_Set(X, Y, X + 7, Y + 15);
			for (j = 0; j < 2; j ++)					//字符分上下两部分，每部分8行
			{
				for (k = 0; k < 8; k ++)				//按LCD显存的行优先顺序展开
				{
					for (i = 0; i < 8; i ++)			//遍历当前行的8列
					{
						Data = LCD_F8x16[Char - ' '][j * 8 + i];
						Pixel = ((Data >> k) & 0x01) ? ForeColor : BackColor;
						LCD_DMATxBuffer[Index * 2] = (uint8_t)(Pixel >> 8);
						LCD_DMATxBuffer[Index * 2 + 1] = (uint8_t)Pixel;
						Index ++;
					}
				}
			}
			(void)LCD_DMA_SendBytes(LCD_DMATxBuffer, 256);
			LCD_SPI_End();
		}
		else	//字符超出屏幕范围，逐点绘制并裁剪（只画笔画，不画背景）
		{
			for (j = 0; j < 2; j ++)
			{
				for (i = 0; i < 8; i ++)
				{
					Data = LCD_F8x16[Char - ' '][j * 8 + i];
					for (k = 0; k < 8; k ++)
					{
						if ((Data >> k) & 0x01)
						{
							LCD_DrawPoint_Color(X + i, Y + j * 8 + k, ForeColor);
						}
					}
				}
			}
		}
	}
	else if (FontSize == LCD_6X8)	//字体为宽6像素，高8像素
	{
		if (X >= 0 && Y >= 0 && X + 6 <= LCD_W && Y + 8 <= LCD_H)
		{
			/*字符完整位于屏幕内：先在RAM中展开为像素字节（高字节在前），再一次DMA发送*/
			LCD_Address_Set(X, Y, X + 5, Y + 7);
			for (k = 0; k < 8; k ++)					//按LCD显存的行优先顺序展开
			{
				for (i = 0; i < 6; i ++)				//遍历当前行的6列
				{
					Data = LCD_F6x8[Char - ' '][i];
					Pixel = ((Data >> k) & 0x01) ? ForeColor : BackColor;
					LCD_DMATxBuffer[Index * 2] = (uint8_t)(Pixel >> 8);
					LCD_DMATxBuffer[Index * 2 + 1] = (uint8_t)Pixel;
					Index ++;
				}
			}
			(void)LCD_DMA_SendBytes(LCD_DMATxBuffer, 96);
			LCD_SPI_End();
		}
		else	//字符超出屏幕范围，逐点绘制并裁剪（只画笔画，不画背景）
		{
			for (i = 0; i < 6; i ++)
			{
				Data = LCD_F6x8[Char - ' '][i];
				for (k = 0; k < 8; k ++)
				{
					if ((Data >> k) & 0x01)
					{
						LCD_DrawPoint_Color(X + i, Y + k, ForeColor);
					}
				}
			}
		}
	}

	LCD_Unlock();
}

/**
  * 函    数：LCD显示字符串
  * 参    数：X 指定字符串左上角的横坐标，范围：-32768~32767，屏幕区域：0~127
  * 参    数：Y 指定字符串左上角的纵坐标，范围：-32768~32767，屏幕区域：0~159
  * 参    数：String 指定要显示的字符串，范围：ASCII码可见字符组成的字符串
  * 参    数：FontSize 指定字体大小，取值：LCD_8X16、LCD_6X8
  * 返 回 值：无
  */
void LCD_ShowString(int16_t X, int16_t Y, char *String, uint8_t FontSize)
{
	uint8_t i;
	LCD_Lock();
	for (i = 0; String[i] != '\0'; i ++)		//遍历字符串的每个字符
	{
		LCD_ShowChar(X + i * FontSize, Y, String[i], FontSize);
	}
	LCD_Unlock();
}

/**
  * 函    数：次方函数（内部调用）
  * 返回值：X的Y次方
  */
static uint32_t LCD_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

/**
  * 函    数：LCD显示数字（十进制，正整数）
  * 参    数：X, Y 指定数字左上角的坐标
  * 参    数：Number 指定要显示的数字，范围：0~4294967295
  * 参    数：Length 指定数字的长度，范围：0~10
  * 参    数：FontSize 指定字体大小，取值：LCD_8X16、LCD_6X8
  * 返 回 值：无
  */
void LCD_ShowNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
	uint8_t i;
	LCD_Lock();
	for (i = 0; i < Length; i ++)		//遍历数字的每一位
	{
		LCD_ShowChar(X + i * FontSize, Y, Number / LCD_Pow(10, Length - i - 1) % 10 + '0', FontSize);
	}
	LCD_Unlock();
}

/**
  * 函    数：LCD显示有符号数字（十进制，整数）
  * 参    数：X, Y 指定数字左上角的坐标
  * 参    数：Number 指定要显示的数字，范围：-2147483648~2147483647
  * 参    数：Length 指定数字的长度，范围：0~10
  * 参    数：FontSize 指定字体大小，取值：LCD_8X16、LCD_6X8
  * 返 回 值：无
  */
void LCD_ShowSignedNum(int16_t X, int16_t Y, int32_t Number, uint8_t Length, uint8_t FontSize)
{
	uint8_t i;
	uint32_t Number1;

	LCD_Lock();

	if (Number >= 0)						//数字大于等于0
	{
		LCD_ShowChar(X, Y, '+', FontSize);	//显示+号
		Number1 = Number;
	}
	else									//数字小于0
	{
		LCD_ShowChar(X, Y, '-', FontSize);	//显示-号
		Number1 = -Number;
	}

	for (i = 0; i < Length; i ++)			//遍历数字的每一位
	{
		LCD_ShowChar(X + (i + 1) * FontSize, Y, Number1 / LCD_Pow(10, Length - i - 1) % 10 + '0', FontSize);
	}

	LCD_Unlock();
}

/**
  * 函    数：LCD显示十六进制数字（十六进制，正整数）
  * 参    数：X, Y 指定数字左上角的坐标
  * 参    数：Number 指定要显示的数字，范围：0x00000000~0xFFFFFFFF
  * 参    数：Length 指定数字的长度，范围：0~8
  * 参    数：FontSize 指定字体大小，取值：LCD_8X16、LCD_6X8
  * 返 回 值：无
  */
void LCD_ShowHexNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
	uint8_t i, SingleNumber;

	LCD_Lock();

	for (i = 0; i < Length; i ++)		//遍历数字的每一位
	{
		SingleNumber = Number / LCD_Pow(16, Length - i - 1) % 16;	//以十六进制提取数字的每一位

		if (SingleNumber < 10)			//单个数字小于10
		{
			LCD_ShowChar(X + i * FontSize, Y, SingleNumber + '0', FontSize);
		}
		else							//单个数字大于10
		{
			LCD_ShowChar(X + i * FontSize, Y, SingleNumber - 10 + 'A', FontSize);
		}
	}

	LCD_Unlock();
}

/**
  * 函    数：LCD显示二进制数字（二进制，正整数）
  * 参    数：X, Y 指定数字左上角的坐标
  * 参    数：Number 指定要显示的数字，范围：0x00000000~0xFFFFFFFF
  * 参    数：Length 指定数字的长度，范围：0~16
  * 参    数：FontSize 指定字体大小，取值：LCD_8X16、LCD_6X8
  * 返 回 值：无
  */
void LCD_ShowBinNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
	uint8_t i;
	LCD_Lock();
	for (i = 0; i < Length; i ++)		//遍历数字的每一位
	{
		LCD_ShowChar(X + i * FontSize, Y, Number / LCD_Pow(2, Length - i - 1) % 2 + '0', FontSize);
	}
	LCD_Unlock();
}

/**
  * 函    数：LCD显示浮点数字（十进制，小数）
  * 参    数：X, Y 指定数字左上角的坐标
  * 参    数：Number 指定要显示的数字，范围：-4294967295.0~4294967295.0
  * 参    数：IntLength 指定数字的整数位长度，范围：0~10
  * 参    数：FraLength 指定数字的小数位长度，范围：0~9，小数进行四舍五入显示
  * 参    数：FontSize 指定字体大小，取值：LCD_8X16、LCD_6X8
  * 返 回 值：无
  */
void LCD_ShowFloatNum(int16_t X, int16_t Y, double Number, uint8_t IntLength, uint8_t FraLength, uint8_t FontSize)
{
	uint32_t PowNum, IntNum, FraNum;

	LCD_Lock();

	if (Number >= 0)						//数字大于等于0
	{
		LCD_ShowChar(X, Y, '+', FontSize);	//显示+号
	}
	else									//数字小于0
	{
		LCD_ShowChar(X, Y, '-', FontSize);	//显示-号
		Number = -Number;
	}

	/*提取整数部分和小数部分*/
	IntNum = Number;						//直接赋值给整型变量，提取整数
	Number -= IntNum;						//将Number的整数减掉，防止之后将小数乘到整数时因数过大造成错误
	PowNum = LCD_Pow(10, FraLength);		//根据指定小数的位数，确定乘数
	FraNum = round(Number * PowNum);		//将小数乘到整数，同时四舍五入，避免显示误差
	IntNum += FraNum / PowNum;				//若四舍五入造成了进位，则需要再加给整数

	LCD_ShowNum(X + FontSize, Y, IntNum, IntLength, FontSize);		//显示整数部分
	LCD_ShowChar(X + (IntLength + 1) * FontSize, Y, '.', FontSize);	//显示小数点
	LCD_ShowNum(X + (IntLength + 2) * FontSize, Y, FraNum, FraLength, FontSize);	//显示小数部分

	LCD_Unlock();
}

/**
  * 函    数：LCD显示图像
  * 参    数：X, Y 指定图像左上角的坐标
  * 参    数：Width, Height 指定图像的宽度和高度
  * 参    数：Image 指定要显示的图像数据，RGB565格式的颜色数组（uint16_t）
  * 返 回 值：无
  * 说    明：此处Image为每个像素16位颜色值，用Image2Lcd软件取模时选择
  *           "16位真彩色"（RGB565）、高位在前（MSB），扫描方式：垂直扫描、从左到右、从上到下
  *           图像分块经暂存缓冲转换字节序后DMA发送，发送期间任务阻塞等待、不占CPU
  */
void LCD_ShowImage(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, const uint16_t *Image)
{
	uint32_t i, j, n, Offset = 0;
	uint16_t Chunk;
	uint16_t Pixel;

	if (Image == NULL || Width == 0 || Height == 0)
	{
		return;
	}

	LCD_Lock();

	if (X >= 0 && Y >= 0 && X + Width <= LCD_W && Y + Height <= LCD_H)
	{
		/*图像完整位于屏幕内：分块拷贝到暂存缓冲（转为高字节在前）后DMA发送*/
		LCD_Address_Set(X, Y, X + Width - 1, Y + Height - 1);
		n = (uint32_t)Width * Height;
		while (n > 0)
		{
			Chunk = (n > LCD_DMA_CHUNK_PIXELS) ? (uint16_t)LCD_DMA_CHUNK_PIXELS : (uint16_t)n;
			for (i = 0; i < Chunk; i ++)
			{
				Pixel = Image[Offset + i];
				LCD_DMATxBuffer[i * 2] = (uint8_t)(Pixel >> 8);
				LCD_DMATxBuffer[i * 2 + 1] = (uint8_t)Pixel;
			}
			(void)LCD_DMA_SendBytes(LCD_DMATxBuffer, (uint32_t)Chunk * 2);
			Offset += Chunk;
			n -= Chunk;
		}
		LCD_SPI_End();
	}
	else	//图像超出屏幕范围，逐点绘制并裁剪
	{
		for (j = 0; j < Height; j ++)
		{
			for (i = 0; i < Width; i ++)
			{
				LCD_DrawPoint_Color(X + i, Y + j, Image[j * Width + i]);
			}
		}
	}

	LCD_Unlock();
}

/**
  * 函    数：LCD使用printf函数打印格式化字符串
  * 参    数：X 指定格式化字符串左上角的横坐标，范围：-32768~32767，屏幕区域：0~127
  * 参    数：Y 指定格式化字符串左上角的纵坐标，范围：-32768~32767，屏幕区域：0~159
  * 参    数：FontSize 指定字体大小，取值：LCD_8X16、LCD_6X8
  * 参    数：format 指定要显示的格式化字符串
  * 参    数：... 格式化字符串参数列表
  * 返 回 值：无
  */
void LCD_Printf(int16_t X, int16_t Y, uint8_t FontSize, char *format, ...)
{
	char String[256];
	va_list arg;
	LCD_Lock();
	va_start(arg, format);
	vsnprintf(String, sizeof(String), format, arg);
	va_end(arg);
	LCD_ShowString(X, Y, String, FontSize);
	LCD_Unlock();
}

/**
  * 函    数：LCD在指定位置画一个点（使用当前前景色）
  * 参    数：X 指定点的横坐标，范围：-32768~32767，屏幕区域：0~127
  * 参    数：Y 指定点的纵坐标，范围：-32768~32767，屏幕区域：0~159
  * 返 回 值：无
  */
void LCD_DrawPoint(int16_t X, int16_t Y)
{
	uint16_t Color;
	LCD_Lock();
	Color = LCD_Color;
	LCD_DrawPoint_Color(X, Y, Color);
	LCD_Unlock();
}

/**
  * 函    数：LCD画线
  * 参    数：X0, Y0 指定一个端点的坐标
  * 参    数：X1, Y1 指定另一个端点的坐标
  * 返 回 值：无
  * 说    明：横线、竖线使用DMA批量填充；斜线仍逐点绘制
  */
void LCD_DrawLine(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1)
{
	int16_t x, y, dx, dy, d, incrE, incrNE, temp;
	int16_t x0 = X0, y0 = Y0, x1 = X1, y1 = Y1;
	uint8_t yflag = 0, xyflag = 0;
	uint16_t Color;

	LCD_Lock();
	Color = LCD_Color;

	if (y0 == y1)		//横线单独处理：一次DMA批量填充
	{
		if (x0 > x1) {temp = x0; x0 = x1; x1 = temp;}

		if (y0 >= 0 && y0 < LCD_H && x0 >= 0 && x1 < LCD_W)
		{
			LCD_FillInternal(x0, y0, x1 - x0 + 1, 1, Color);
		}
	}
	else if (x0 == x1)	//竖线单独处理：一次DMA批量填充
	{
		if (y0 > y1) {temp = y0; y0 = y1; y1 = temp;}

		if (x0 >= 0 && x0 < LCD_W && y0 >= 0 && y1 < LCD_H)
		{
			LCD_FillInternal(x0, y0, 1, y1 - y0 + 1, Color);
		}
	}
	else				//斜线
	{
		/*使用Bresenham算法画直线，可以避免耗时的浮点运算，效率更高*/

		if (x0 > x1)	//0号点X坐标大于1号点X坐标
		{
			/*交换两点坐标*/
			temp = x0; x0 = x1; x1 = temp;
			temp = y0; y0 = y1; y1 = temp;
		}

		if (y0 > y1)	//0号点Y坐标大于1号点Y坐标
		{
			/*将Y坐标取负*/
			y0 = -y0;
			y1 = -y1;
			yflag = 1;
		}

		if (y1 - y0 > x1 - x0)	//画线斜率大于1
		{
			/*将X坐标与Y坐标互换*/
			temp = x0; x0 = y0; y0 = temp;
			temp = x1; x1 = y1; y1 = temp;
			xyflag = 1;
		}

		/*以下为Bresenham算法画直线*/
		dx = x1 - x0;
		dy = y1 - y0;
		incrE = 2 * dy;
		incrNE = 2 * (dy - dx);
		d = 2 * dy - dx;
		x = x0;
		y = y0;

		/*画起始点，同时判断标志位，将坐标换回来*/
		if (yflag && xyflag)		{LCD_DrawPoint_Color(y, -x, Color);}
		else if (yflag)				{LCD_DrawPoint_Color(x, -y, Color);}
		else if (xyflag)			{LCD_DrawPoint_Color(y, x, Color);}
		else						{LCD_DrawPoint_Color(x, y, Color);}

		while (x < x1)
		{
			x ++;
			if (d < 0)
			{
				d += incrE;
			}
			else
			{
				y ++;
				d += incrNE;
			}

			/*画每一个点，同时判断标志位，将坐标换回来*/
			if (yflag && xyflag)	{LCD_DrawPoint_Color(y, -x, Color);}
			else if (yflag)			{LCD_DrawPoint_Color(x, -y, Color);}
			else if (xyflag)		{LCD_DrawPoint_Color(y, x, Color);}
			else					{LCD_DrawPoint_Color(x, y, Color);}
		}
	}

	LCD_Unlock();
}

/**
  * 函    数：LCD矩形
  * 参    数：X, Y 指定矩形左上角的坐标
  * 参    数：Width 指定矩形的宽度，范围：0~128
  * 参    数：Height 指定矩形的高度，范围：0~160
  * 参    数：IsFilled 指定矩形是否填充，取值：LCD_UNFILLED、LCD_FILLED
  * 返 回 值：无
  */
void LCD_DrawRectangle(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, uint8_t IsFilled)
{
	LCD_Lock();

	if (X >= 0 && Y >= 0 && X + Width <= LCD_W && Y + Height <= LCD_H &&
		Width > 0 && Height > 0)
	{
		if (!IsFilled)		//指定矩形不填充
		{
			/*四条边分别用DMA横线/竖线批量绘制*/
			LCD_DrawLine(X, Y, X + Width - 1, Y);
			LCD_DrawLine(X, Y + Height - 1, X + Width - 1, Y + Height - 1);
			LCD_DrawLine(X, Y, X, Y + Height - 1);
			LCD_DrawLine(X + Width - 1, Y, X + Width - 1, Y + Height - 1);
		}
		else				//指定矩形填充：一次DMA批量填充
		{
			LCD_FillInternal(X, Y, Width, Height, LCD_Color);
		}
	}

	LCD_Unlock();
}

/**
  * 函    数：判断指定点是否在指定多边形内部（内部调用）
  */
static uint8_t LCD_pnpoly(uint8_t nvert, int16_t *vertx, int16_t *verty, int16_t testx, int16_t testy)
{
	int16_t i, j, c = 0;

	for (i = 0, j = nvert - 1; i < nvert; j = i ++)
	{
		if (((verty[i] > testy) != (verty[j] > testy)) &&
			(testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i]))
		{
			c = !c;
		}
	}
	return c;
}

/**
  * 函    数：LCD三角形
  * 参    数：X0~Y2 指定三个端点的坐标
  * 参    数：IsFilled 指定三角形是否填充，取值：LCD_UNFILLED、LCD_FILLED
  * 返 回 值：无
  */
void LCD_DrawTriangle(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1, int16_t X2, int16_t Y2, uint8_t IsFilled)
{
	int16_t minx = X0, miny = Y0, maxx = X0, maxy = Y0;
	int16_t i, j;
	int16_t vx[] = {X0, X1, X2};
	int16_t vy[] = {Y0, Y1, Y2};

	LCD_Lock();

	if (!IsFilled)			//指定三角形不填充
	{
		/*调用画线函数，将三个点用直线连接*/
		LCD_DrawLine(X0, Y0, X1, Y1);
		LCD_DrawLine(X0, Y0, X2, Y2);
		LCD_DrawLine(X1, Y1, X2, Y2);
	}
	else					//指定三角形填充
	{
		/*找到三个点最小的X、Y坐标*/
		if (X1 < minx) {minx = X1;}
		if (X2 < minx) {minx = X2;}
		if (Y1 < miny) {miny = Y1;}
		if (Y2 < miny) {miny = Y2;}

		/*找到三个点最大的X、Y坐标*/
		if (X1 > maxx) {maxx = X1;}
		if (X2 > maxx) {maxx = X2;}
		if (Y1 > maxy) {maxy = Y1;}
		if (Y2 > maxy) {maxy = Y2;}

		/*最小最大坐标之间的矩形为可能需要填充的区域，遍历此区域中所有的点*/
		for (i = minx; i <= maxx; i ++)
		{
			for (j = miny; j <= maxy; j ++)
			{
				/*调用LCD_pnpoly，判断指定点是否在指定三角形之中*/
				if (LCD_pnpoly(3, vx, vy, i, j)) {LCD_DrawPoint(i, j);}
			}
		}
	}

	LCD_Unlock();
}

/**
  * 函    数：LCD画圆
  * 参    数：X, Y 指定圆的圆心坐标
  * 参    数：Radius 指定圆的半径，范围：0~255
  * 参    数：IsFilled 指定圆是否填充，取值：LCD_UNFILLED、LCD_FILLED
  * 返 回 值：无
  */
void LCD_DrawCircle(int16_t X, int16_t Y, uint8_t Radius, uint8_t IsFilled)
{
	int16_t x, y, d, j;

	LCD_Lock();

	/*使用Bresenham算法画圆，可以避免耗时的浮点运算，效率更高*/

	d = 1 - Radius;
	x = 0;
	y = Radius;

	/*画每个八分之一圆弧的起始点*/
	LCD_DrawPoint(X + x, Y + y);
	LCD_DrawPoint(X - x, Y - y);
	LCD_DrawPoint(X + y, Y + x);
	LCD_DrawPoint(X - y, Y - x);

	if (IsFilled)		//指定圆填充
	{
		for (j = -y; j < y; j ++)		//遍历起始点Y坐标
		{
			LCD_DrawPoint(X, Y + j);	//在指定区域画点，填充部分圆
		}
	}

	while (x < y)		//遍历X轴的每个点
	{
		x ++;
		if (d < 0)
		{
			d += 2 * x + 1;
		}
		else
		{
			y --;
			d += 2 * (x - y) + 1;
		}

		/*画每个八分之一圆弧的点*/
		LCD_DrawPoint(X + x, Y + y);
		LCD_DrawPoint(X + y, Y + x);
		LCD_DrawPoint(X - x, Y - y);
		LCD_DrawPoint(X - y, Y - x);
		LCD_DrawPoint(X + x, Y - y);
		LCD_DrawPoint(X + y, Y - x);
		LCD_DrawPoint(X - x, Y + y);
		LCD_DrawPoint(X - y, Y + x);

		if (IsFilled)	//指定圆填充
		{
			for (j = -y; j < y; j ++)	//遍历中间部分
			{
				LCD_DrawPoint(X + x, Y + j);
				LCD_DrawPoint(X - x, Y + j);
			}

			for (j = -x; j < x; j ++)	//遍历两侧部分
			{
				LCD_DrawPoint(X - y, Y + j);
				LCD_DrawPoint(X + y, Y + j);
			}
		}
	}

	LCD_Unlock();
}

/**
  * 函    数：LCD画椭圆
  * 参    数：X, Y 指定椭圆的圆心坐标
  * 参    数：A 指定椭圆的横向半轴长度，范围：0~255
  * 参    数：B 指定椭圆的纵向半轴长度，范围：0~255
  * 参    数：IsFilled 指定椭圆是否填充，取值：LCD_UNFILLED、LCD_FILLED
  * 返 回 值：无
  */
void LCD_DrawEllipse(int16_t X, int16_t Y, uint8_t A, uint8_t B, uint8_t IsFilled)
{
	int16_t x, y, j;
	int16_t a = A, b = B;
	float d1, d2;

	LCD_Lock();

	/*使用Bresenham算法画椭圆，可以避免部分耗时的浮点运算，效率更高*/

	x = 0;
	y = b;
	d1 = b * b + a * a * (-b + 0.5);

	if (IsFilled)	//指定椭圆填充
	{
		for (j = -y; j < y; j ++)		//遍历起始点Y坐标
		{
			LCD_DrawPoint(X, Y + j);	//在指定区域画点，填充部分椭圆
		}
	}

	/*画椭圆弧的起始点*/
	LCD_DrawPoint(X + x, Y + y);
	LCD_DrawPoint(X - x, Y - y);
	LCD_DrawPoint(X - x, Y + y);
	LCD_DrawPoint(X + x, Y - y);

	/*画椭圆中间部分*/
	while (b * b * (x + 1) < a * a * (y - 0.5))
	{
		if (d1 <= 0)
		{
			d1 += b * b * (2 * x + 3);
		}
		else
		{
			d1 += b * b * (2 * x + 3) + a * a * (-2 * y + 2);
			y --;
		}
		x ++;

		if (IsFilled)	//指定椭圆填充
		{
			for (j = -y; j < y; j ++)	//遍历中间部分
			{
				LCD_DrawPoint(X + x, Y + j);
				LCD_DrawPoint(X - x, Y + j);
			}
		}

		/*画椭圆中间部分圆弧*/
		LCD_DrawPoint(X + x, Y + y);
		LCD_DrawPoint(X - x, Y - y);
		LCD_DrawPoint(X - x, Y + y);
		LCD_DrawPoint(X + x, Y - y);
	}

	/*画椭圆两侧部分*/
	d2 = b * b * (x + 0.5) * (x + 0.5) + a * a * (y - 1) * (y - 1) - a * a * b * b;

	while (y > 0)
	{
		if (d2 <= 0)
		{
			d2 += b * b * (2 * x + 2) + a * a * (-2 * y + 3);
			x ++;
		}
		else
		{
			d2 += a * a * (-2 * y + 3);
		}
		y --;

		if (IsFilled)	//指定椭圆填充
		{
			for (j = -y; j < y; j ++)	//遍历两侧部分
			{
				LCD_DrawPoint(X + x, Y + j);
				LCD_DrawPoint(X - x, Y + j);
			}
		}

		/*画椭圆两侧部分圆弧*/
		LCD_DrawPoint(X + x, Y + y);
		LCD_DrawPoint(X - x, Y - y);
		LCD_DrawPoint(X - x, Y + y);
		LCD_DrawPoint(X + x, Y - y);
	}

	LCD_Unlock();
}

/**
  * 函    数：判断指定点是否在指定角度内部（内部调用）
  */
static uint8_t LCD_IsInAngle(int16_t X, int16_t Y, int16_t StartAngle, int16_t EndAngle)
{
	int16_t PointAngle;
	PointAngle = atan2(Y, X) / 3.14 * 180;	//计算指定点的弧度，并转换为角度表示
	if (StartAngle < EndAngle)	//起始角度小于终止角度的情况
	{
		if (PointAngle >= StartAngle && PointAngle <= EndAngle)
		{
			return 1;
		}
	}
	else			//起始角度大于终止角度的情况
	{
		if (PointAngle >= StartAngle || PointAngle <= EndAngle)
		{
			return 1;
		}
	}
	return 0;
}

/**
  * 函    数：LCD画圆弧
  * 参    数：X, Y 指定圆弧的圆心坐标
  * 参    数：Radius 指定圆弧的半径，范围：0~255
  * 参    数：StartAngle 指定圆弧的起始角度，范围：-180~180
  *           水平向右为0度，水平向左为180度或-180度，下方为正数，上方为负数，顺时针旋转
  * 参    数：EndAngle 指定圆弧的终止角度，范围：-180~180
  * 参    数：IsFilled 指定圆弧是否填充，填充后为扇形，取值：LCD_UNFILLED、LCD_FILLED
  * 返 回 值：无
  */
void LCD_DrawArc(int16_t X, int16_t Y, uint8_t Radius, int16_t StartAngle, int16_t EndAngle, uint8_t IsFilled)
{
	int16_t x, y, d, j;

	LCD_Lock();

	/*此函数借用Bresenham算法画圆的方法*/

	d = 1 - Radius;
	x = 0;
	y = Radius;

	/*在画圆的每个点时，判断指定点是否在指定角度内，在，则画点，不在，则不做处理*/
	if (LCD_IsInAngle(x, y, StartAngle, EndAngle))		{LCD_DrawPoint(X + x, Y + y);}
	if (LCD_IsInAngle(-x, -y, StartAngle, EndAngle))	{LCD_DrawPoint(X - x, Y - y);}
	if (LCD_IsInAngle(y, x, StartAngle, EndAngle))		{LCD_DrawPoint(X + y, Y + x);}
	if (LCD_IsInAngle(-y, -x, StartAngle, EndAngle))	{LCD_DrawPoint(X - y, Y - x);}

	if (IsFilled)	//指定圆弧填充
	{
		for (j = -y; j < y; j ++)		//遍历起始点Y坐标
		{
			if (LCD_IsInAngle(0, j, StartAngle, EndAngle)) {LCD_DrawPoint(X, Y + j);}
		}
	}

	while (x < y)		//遍历X轴的每个点
	{
		x ++;
		if (d < 0)
		{
			d += 2 * x + 1;
		}
		else
		{
			y --;
			d += 2 * (x - y) + 1;
		}

		/*在画圆的每个点时，判断指定点是否在指定角度内，在，则画点，不在，则不做处理*/
		if (LCD_IsInAngle(x, y, StartAngle, EndAngle))		{LCD_DrawPoint(X + x, Y + y);}
		if (LCD_IsInAngle(y, x, StartAngle, EndAngle))		{LCD_DrawPoint(X + y, Y + x);}
		if (LCD_IsInAngle(-x, -y, StartAngle, EndAngle))	{LCD_DrawPoint(X - x, Y - y);}
		if (LCD_IsInAngle(-y, -x, StartAngle, EndAngle))	{LCD_DrawPoint(X - y, Y - x);}
		if (LCD_IsInAngle(x, -y, StartAngle, EndAngle))		{LCD_DrawPoint(X + x, Y - y);}
		if (LCD_IsInAngle(y, -x, StartAngle, EndAngle))		{LCD_DrawPoint(X + y, Y - x);}
		if (LCD_IsInAngle(-x, y, StartAngle, EndAngle))		{LCD_DrawPoint(X - x, Y + y);}
		if (LCD_IsInAngle(-y, x, StartAngle, EndAngle))		{LCD_DrawPoint(X - y, Y + x);}

		if (IsFilled)	//指定圆弧填充
		{
			for (j = -y; j < y; j ++)	//遍历中间部分
			{
				if (LCD_IsInAngle(x, j, StartAngle, EndAngle))		{LCD_DrawPoint(X + x, Y + j);}
				if (LCD_IsInAngle(-x, j, StartAngle, EndAngle))	{LCD_DrawPoint(X - x, Y + j);}
			}

			for (j = -x; j < x; j ++)	//遍历两侧部分
			{
				if (LCD_IsInAngle(y, j, StartAngle, EndAngle))		{LCD_DrawPoint(X + y, Y + j);}
				if (LCD_IsInAngle(-y, j, StartAngle, EndAngle))	{LCD_DrawPoint(X - y, Y + j);}
			}
		}
	}

	LCD_Unlock();
}
