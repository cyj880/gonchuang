#include "Delay.h"
#include "FreeRTOS.h"
#include "task.h"

/* DWT is independent of the FreeRTOS SysTick. Never reset CYCCNT: startup
 * lift homing also uses its monotonic counter for deadlines. */
void Delay_Init(u8 SYSCLK)
{
    (void)SYSCLK;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void Delay_us(u32 nus)
{
    uint32_t cycles_per_us = SystemCoreClock / 1000000u;
    Delay_Init(0);
    while (nus)
    {
        uint32_t chunk = nus > 1000u ? 1000u : nus;
        uint32_t start = DWT->CYCCNT;
        uint32_t cycles = chunk * cycles_per_us;
        while ((uint32_t)(DWT->CYCCNT - start) < cycles) {}
        nus -= chunk;
    }
}

/* Tasks yield to the scheduler. Before startup, in an ISR or while scheduling
 * is suspended, use DWT instead; none of these paths reprogram SysTick. */
void Delay_ms(u16 nms)
{
    if (!nms) return;
    if (__get_IPSR() == 0u && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        TickType_t ticks = pdMS_TO_TICKS(nms);
        vTaskDelay(ticks ? ticks : 1u);
        return;
    }
    Delay_us((uint32_t)nms * 1000u);
}
