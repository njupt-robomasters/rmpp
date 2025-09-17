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

void Dwt::Update() {
    const uint32_t now_tick32 = DWT->CYCCNT;
    tick64 += now_tick32 - tick32;
    tick32 = now_tick32;
}

float Dwt::GetTime() {
    Update();
    const float second = (float)tick64 / (float)SystemCoreClock;
    return second;
}

void Dwt::Delay(const float second) {
    const uint32_t start_tick = DWT->CYCCNT;
    Update();
    const auto need_ticks = (uint32_t)(second * (float)SystemCoreClock);
    while (DWT->CYCCNT - start_tick < need_ticks) {}
}

float Dwt::GetDT() {
    Update();
    if (tick == 0) { // 第一次固定返回0.001
        tick = DWT->CYCCNT;
        return 0.001f;
    }
    const uint32_t now_tick = DWT->CYCCNT;
    dt = (float)(now_tick - tick) / (float)SystemCoreClock;
    tick = now_tick;
    return dt;
}

void Dwt::Clear() {
    tick = 0;
}

float BSP_DWT_GetDT(uint32_t* tick) {
    Dwt::Update();
    if (*tick == 0) { // 第一次固定返回0.001
        *tick = DWT->CYCCNT;
        return 0.001f;
    }
    const uint32_t now_tick = DWT->CYCCNT;
    const float dt = (float)(now_tick - *tick) / (float)SystemCoreClock;
    *tick = now_tick;
    return dt;
}

float BSP_DWT_GetTime() {
    return Dwt::GetTime();
}

void BSP_DWT_Delay(const float second) {
    Dwt::Delay(second);
}
