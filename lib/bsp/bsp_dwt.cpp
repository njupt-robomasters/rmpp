#include "bsp_dwt.h"
#include "main.h"

using namespace BSP;

uint64_t Dwt::tick64 = 0;
uint32_t Dwt::tick32 = 0;

void Dwt::Init() {
    // 使能DWT外设
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // DWT CYCCNT寄存器计数清0
    DWT->CYCCNT = 0;

    // 使能Cortex-M DWT CYCCNT寄存器
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void Dwt::onLoop() {
    const uint32_t now_tick32 = DWT->CYCCNT;
    tick64 += now_tick32 - tick32;
    tick32 = now_tick32;
}

double Dwt::GetTime() {
    onLoop();
    const double seconds = (double)tick64 / (double)SystemCoreClock;
    return seconds;
}

void Dwt::Delay(const float seconds) {
    const uint32_t start_tick = DWT->CYCCNT;
    onLoop();
    const auto need_ticks = (uint32_t)(seconds * (float)SystemCoreClock);
    while (DWT->CYCCNT - start_tick < need_ticks) {}
}

float Dwt::GetDT() {
    onLoop();
    if (last_tick == 0) { // 第一次固定返回0.001
        last_tick = DWT->CYCCNT;
        return 0.001f;
    }
    const uint32_t now_tick = DWT->CYCCNT;
    dt = (float)(now_tick - last_tick) / (float)SystemCoreClock;
    freq = 1 / dt;
    last_tick = now_tick;
    return dt;
}

void Dwt::Update() {
    GetDT();
}

void Dwt::Clear() {
    last_tick = 0;
}

float BSP_DWT_GetDT(uint32_t* last_tick) {
    if (*last_tick == 0) { // 第一次固定返回0.001
        *last_tick = DWT->CYCCNT;
        return 0.001f;
    }
    const uint32_t now_tick = DWT->CYCCNT;
    const float dt = (float)(now_tick - *last_tick) / (float)SystemCoreClock;
    *last_tick = now_tick;
    return dt;
}

double BSP_DWT_GetTime() {
    return Dwt::GetTime();
}

void BSP_DWT_Delay(const float seconds) {
    Dwt::Delay(seconds);
}