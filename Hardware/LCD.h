#ifndef __LCD_H
#define __LCD_H

#include <stdint.h>

/*
 * 1.8寸 TFT-LCD（ST7735S，128*160）驱动
 * 【A板（STM32F427IIH6）接线】PB3=SCK(SPI1_SCK/AF5)、PA7=SDA(SPI1_MOSI/AF5)、
 *                        PB9=DC、PB0=RES、PA6=CS、PI2=BL（背光，A板8路PWM排针/TIM8_CH4），VCC=3.3V、GND=GND
 * 上层函数接口与点阵屏驱动同构：xxx_ShowString、xxx_DrawLine 等参数格式一致
 * 区别1：LCD为彩色屏，通过LCD_SetColor/LCD_SetBackgroundColor设置前景色/背景色（RGB565）
 * 区别2：LCD自带显存，直接写屏，无需（也没有）LCD_Update函数
 * 区别3：LCD_ShowImage的Image参数为RGB565格式的颜色数组（uint16_t），而非单色位图
 * 说明1：批量像素（填充、字符、图像、横竖线）由DMA2数据流3搬运（SPI1_TX），DMA期间任务阻塞
 *        等待、不占用CPU，不拖慢其他任务
 * 说明2：所有上层函数内部使用递归互斥锁，可以从多个任务直接并发调用，驱动会自动排队
 */

/*参数宏定义*********************/

/*FontSize参数取值*/
/*此参数值不仅用于判断，而且用于计算横向字符偏移，默认值为字体像素宽度*/
#define LCD_8X16				8
#define LCD_6X8					6

/*IsFilled参数数值*/
#define LCD_UNFILLED			0
#define LCD_FILLED				1

/*屏幕分辨率（竖屏）*/
#define LCD_W					128
#define LCD_H					160

/*横竖屏设置：0或1为竖屏，2或3为横屏*/
#define USE_HORIZONTAL			1

/*常用颜色（RGB565格式：高5位红，中6位绿，低5位蓝）*/
#define LCD_WHITE				0xFFFF
#define LCD_BLACK				0x0000
#define LCD_RED					0xF800
#define LCD_GREEN				0x07E0
#define LCD_BLUE				0x001F
#define LCD_YELLOW				0xFFE0
#define LCD_CYAN				0x07FF
#define LCD_MAGENTA				0xF81F
#define LCD_BROWN				0xBC40		//棕色
#define LCD_BRRED				0xFC07		//棕红色
#define LCD_GRAY				0x8430		//灰色
#define LCD_DARKBLUE			0x01CF		//深蓝色
#define LCD_LIGHTBLUE			0x7D7C		//浅蓝色
#define LCD_LIGHTGREEN			0x841F		//浅绿色
#define LCD_LGRAY				0xC618		//浅灰色

/*自定义颜色：RGB565(R,G,B) = ((R&0x1F)<<11)|((G&0x3F)<<5)|(B&0x1F)，R:0~31，G:0~63，B:0~31*/

/*********************参数宏定义*/


/*函数声明*********************/

/*初始化函数*/
void LCD_Init(void);

/*颜色设置函数*/
void LCD_SetColor(uint16_t Color);					//设置前景色（文字、画笔的颜色）
void LCD_SetBackgroundColor(uint16_t Color);		//设置背景色（清屏、文字底色的颜色）

/*清屏函数*/
void LCD_Clear(void);								//清屏，以背景色填充全屏
void LCD_ClearArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height);	//以背景色填充指定区域

/*区域填充函数*/
void LCD_Fill(int16_t X, int16_t Y, int16_t Width, int16_t Height, uint16_t Color);	//以指定颜色填充区域

/*显示函数*/
void LCD_ShowChar(int16_t X, int16_t Y, char Char, uint8_t FontSize);
void LCD_ShowString(int16_t X, int16_t Y, char *String, uint8_t FontSize);
void LCD_ShowNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize);
void LCD_ShowSignedNum(int16_t X, int16_t Y, int32_t Number, uint8_t Length, uint8_t FontSize);
void LCD_ShowHexNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize);
void LCD_ShowBinNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize);
void LCD_ShowFloatNum(int16_t X, int16_t Y, double Number, uint8_t IntLength, uint8_t FraLength, uint8_t FontSize);
void LCD_ShowImage(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, const uint16_t *Image);
void LCD_Printf(int16_t X, int16_t Y, uint8_t FontSize, char *format, ...);

/*绘图函数（均使用当前前景色，可用LCD_SetColor更换）*/
void LCD_DrawPoint(int16_t X, int16_t Y);
void LCD_DrawLine(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1);
void LCD_DrawRectangle(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, uint8_t IsFilled);
void LCD_DrawTriangle(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1, int16_t X2, int16_t Y2, uint8_t IsFilled);
void LCD_DrawCircle(int16_t X, int16_t Y, uint8_t Radius, uint8_t IsFilled);
void LCD_DrawEllipse(int16_t X, int16_t Y, uint8_t A, uint8_t B, uint8_t IsFilled);
void LCD_DrawArc(int16_t X, int16_t Y, uint8_t Radius, int16_t StartAngle, int16_t EndAngle, uint8_t IsFilled);

/*********************函数声明*/

#endif
