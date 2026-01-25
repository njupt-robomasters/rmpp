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

void Dwt::Delay(const UnitFloat<>& time) {
    const uint32_t start_tick = DWT->CYCCNT;
    const auto need_ticks = (uint32_t)(time.toFloat(s) * (float)SystemCoreClock);
    while (DWT->CYCCNT - start_tick < need_ticks) {}
}

UnitFloat<> Dwt::GetDT() {
    const uint32_t now_tick = DWT->CYCCNT;
    const UnitFloat<> dt = (float)(now_tick - last_tick) / (float)SystemCoreClock * s;
    if (dt > MAX_DT) is_max_dt = true;
    if (is_max_dt) return MAX_DT;
    return dt;
}

UnitFloat<> Dwt::UpdateDT() {
    const uint32_t now_tick = DWT->CYCCNT;
    dt = (float)(now_tick - last_tick) / (float)SystemCoreClock * s;
    freq = 1 / dt;
    last_tick = now_tick;
    is_max_dt = false;
    return dt;
}

bool Dwt::PollTimeout(const UnitFloat<>& time) {
    if (GetDT() >= time) {
        UpdateDT();
        return true;
    } else {
        return false;
    }
}

void Dwt::Reset() {
    last_tick = DWT->CYCCNT;
    is_max_dt = false;
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
    Dwt::Delay(seconds * s);
}