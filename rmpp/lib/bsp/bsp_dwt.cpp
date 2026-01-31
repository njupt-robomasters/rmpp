#include "bsp_dwt.h"
#include "main.h"

using namespace BSP;

Dwt::Dwt(const UnitFloat<>& min_dt, const UnitFloat<>& max_dt) : min_dt(min_dt), max_dt(max_dt), last_tick(DWT->CYCCNT) {}

void Dwt::Init() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 使能DWT外设
    DWT->CYCCNT = 0;                                // DWT CYCCNT寄存器计数清0
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // 使能DWT CYCCNT寄存器
}

void Dwt::Delay(const UnitFloat<>& time) {
    const uint32_t start_tick = DWT->CYCCNT;
    const auto need_ticks = (uint32_t)(time.toFloat(s) * (float)SystemCoreClock);
    while (DWT->CYCCNT - start_tick < need_ticks);
}

UnitFloat<> Dwt::GetDT() {
    const uint32_t now_tick = DWT->CYCCNT;
    dt = (float)(now_tick - last_tick) / (float)SystemCoreClock * s;

    if (dt < min_dt) dt = min_dt;
    if (dt > max_dt) dt = max_dt, is_max_dt = true;

    if (is_max_dt) {
        return max_dt;
    } else {
        return dt;
    }
}

UnitFloat<> Dwt::UpdateDT() {
    const uint32_t now_tick = DWT->CYCCNT;
    dt = (float)(now_tick - last_tick) / (float)SystemCoreClock * s;
    last_tick = now_tick;

    if (dt < min_dt) dt = min_dt;
    if (dt > max_dt) dt = max_dt, is_max_dt = true;

    freq = 1 / dt;

    if (is_max_dt) {
        is_max_dt = false;
        return max_dt;
    } else {
        return dt;
    }
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