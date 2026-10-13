#include "rdk_center.h"

#define RDK_HEAD       0x66u
#define RDK_CMD_LOCATE 0x0Au
#define RDK_CMD_CENTER 0x01u
#define RDK_TAIL       0x77u

static volatile uint8_t  rdk_enabled;
static volatile uint8_t  rdk_valid;
static volatile uint16_t rdk_x;
static volatile uint16_t rdk_y;
static uint8_t rx_state;
static uint8_t rx_buf[7];
static uint8_t rx_index;

static void rdk_tx_byte(uint8_t byte)
{
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET) {}
    USART_SendData(USART3, byte);
}

static uint8_t colors_valid(uint8_t c1, uint8_t c2, uint8_t c3)
{
    return (uint8_t)(c1 >= 1 && c1 <= 6 && c2 >= 1 && c2 <= 6 &&
                     c3 >= 1 && c3 <= 6 && c1 != c2 && c1 != c3 && c2 != c3);
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
    nvic.NVIC_IRQChannelPreemptionPriority = 4;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    RDK_Center_Clear();
    RDK_Center_Enable(0);
}

void RDK_Center_IRQHandler(void)
{
    uint8_t byte;

    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        byte = (uint8_t)USART_ReceiveData(USART3);
        if (!rdk_enabled) return;
        if (rx_state == 0)
        {
            if (byte == RDK_HEAD) { rx_buf[0] = byte; rx_index = 1; rx_state = 1; }
        }
        else if (rx_state == 1)
        {
            if (byte == RDK_CMD_CENTER) { rx_buf[1] = byte; rx_index = 2; rx_state = 2; }
            else if (byte == RDK_HEAD) { rx_buf[0] = byte; }
            else rx_state = 0;
        }
        else
        {
            rx_buf[rx_index++] = byte;
            if (rx_index >= sizeof(rx_buf))
            {
                if (rx_buf[6] == RDK_TAIL && rx_buf[1] == RDK_CMD_CENTER)
                {
                    rdk_x = (uint16_t)(((uint16_t)rx_buf[2] << 8) | rx_buf[3]);
                    rdk_y = (uint16_t)(((uint16_t)rx_buf[4] << 8) | rx_buf[5]);
                    if (rdk_x < 640 && rdk_y < 480)
                    {
                        rdk_valid = 1;
                    }
                }
                rx_state = 0;
                rx_index = 0;
            }
        }
    }
    if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)
    {
        (void)USART3->SR;
        (void)USART3->DR;
        rx_state = 0;
    }
}

void RDK_Center_Enable(uint8_t enable)
{
    rdk_enabled = enable ? 1 : 0;
    rx_state = 0;
    rx_index = 0;
}

void RDK_Center_Clear(void)
{
    rdk_valid = 0;
    rdk_x = 0;
    rdk_y = 0;
}

void RDK_Center_SendLocate(uint8_t c1, uint8_t c2, uint8_t c3)
{
    uint8_t sum;
    if (!colors_valid(c1, c2, c3)) return;
    sum = (uint8_t)(RDK_CMD_LOCATE + c1 + c2 + c3);
    rdk_tx_byte(RDK_HEAD); rdk_tx_byte(RDK_CMD_LOCATE);
    rdk_tx_byte(c1); rdk_tx_byte(c2); rdk_tx_byte(c3); rdk_tx_byte(sum); rdk_tx_byte(RDK_TAIL);
}

void RDK_Center_Poll(void)
{
}

uint8_t RDK_CenterValid(void) { return rdk_valid; }
uint16_t RDK_CenterX(void) { return rdk_x; }
uint16_t RDK_CenterY(void) { return rdk_y; }

void USART3_IRQHandler(void)
{
    RDK_Center_IRQHandler();
}
