#ifndef __SERVO_H
#define __SERVO_H

/* 180°舵机: PA1 = TIM5_CH2, 脉宽500~2500us */
void Servo_Init(void);
void Servo_SetAngle(float Angle);      /* 0~180°, 须在 Servo_Init() 后调用 */

/* 270°舵机: PA0 = TIM5_CH1, 脉宽500~2500us(脉宽规格不同改 Servo.c 宏) */
#define SERVO270_STARTUP_DEG 30.0f
#define SERVO270_SLOT_A_DEG   0.0f
#define SERVO270_SLOT_B_DEG 120.0f
#define SERVO270_SLOT_C_DEG 240.0f
void Servo270_Init(void);              /* 须在 Servo_Init() 后调用(共用其时基) */
void Servo270_SetAngle(float Angle);   /* 0~270° */

#endif
