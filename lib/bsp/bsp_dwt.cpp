#include "bsp_dwt.h"
#include "main.h"

using namespace BSP;

void Dwt::Init() {
    // 使能DWT外设
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // DWT CYCCNT寄存器计数清0
    DWT->CYCCNT = 0;

    // 使能Cortex-M DWT CYCCNT寄存器
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void Dwt::Delay(const float seconds) {
    const uint32_t start_tick = DWT->CYCCNT;
    const auto need_ticks = (uint32_t)(seconds * (float)SystemCoreClock);
    while (DWT->CYCCNT - start_tick < need_ticks) {}
}

float Dwt::GetDT() const {
    const uint32_t now_tick = DWT->CYCCNT;
    const float dt = (float)(now_tick - last_tick) / (float)SystemCoreClock;
    return dt;
}

float Dwt::UpdateDT() {
    const uint32_t now_tick = DWT->CYCCNT;
    dt = (float)(now_tick - last_tick) / (float)SystemCoreClock;
    freq = 1 / dt;
    last_tick = now_tick;
    return dt;
}

void Dwt::Reset() {
    last_tick = DWT->CYCCNT;
}

float BSP_DWT_GetDT(const uint32_t last_tick) {
    const uint32_t now_tick = DWT->CYCCNT;
    const float dt = (float)(now_tick - last_tick) / (float)SystemCoreClock;
    return dt;
}

float BSP_DWT_UpdateDT(uint32_t* last_tick) {
    const uint32_t now_tick = DWT->CYCCNT;
    const float dt = (float)(now_tick - *last_tick) / (float)SystemCoreClock;
    *last_tick = now_tick;
    return dt;
}

void BSP_DWT_Delay(const float seconds) {
    Dwt::Delay(seconds);
}