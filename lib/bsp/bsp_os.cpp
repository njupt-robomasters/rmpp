#include "bsp_os.hpp"
#include "main.h"

using namespace BSP;

void OS::TaskCreate(const osThreadFunc_t func, const char* name, const uint32_t stack_bytes, const osPriority_t priority) {
    const osThreadAttr_t task_attributes = {
        .name = name,
        .stack_size = stack_bytes,
        .priority = priority,
    };
    osThreadNew(func, nullptr, &task_attributes);
}

void OS::Delay(const uint32_t ms) {
    osDelay(ms);
}

// 用于FreeRTOS运行时间统计
extern "C" void configureTimerForRunTimeStats(void) {}

extern "C" unsigned long getRunTimeCounterValue(void) {
    return DWT->CYCCNT;
}