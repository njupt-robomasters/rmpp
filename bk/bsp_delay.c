#include "bsp_delay.h"
#include "main.h"

void delay_ticks(const uint32_t ticks) {
    const uint32_t reload = SysTick->LOAD;
    uint32_t told = SysTick->VAL;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    while (1) {
        tnow = SysTick->VAL;
        if (tnow != told) {
            if (tnow < told) {
                tcnt += told - tnow;
            } else {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks) {
                break;
            }
        }
    }
}

void delay_us(const uint16_t nus) {
    const uint32_t ticks = nus * (SystemCoreClock / 1000000);
    delay_ticks(ticks);
}

void delay_ms(const uint16_t nms) {
    const uint32_t ticks = nms * (SystemCoreClock / 1000);
    delay_ticks(ticks);
}

void delay(const uint16_t nms) {
    delay_ms(nms);
}
