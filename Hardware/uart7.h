#ifndef __UART7_H
#define __UART7_H

#include "stm32f4xx.h"

void UART7_ZigBee_Init(uint32_t baud);          /* PE7=RX / PE8=TX，AF8，中断接收 */
void UART7_SendByte(uint8_t byte);
void UART7_SendString(const char *str);

#endif
