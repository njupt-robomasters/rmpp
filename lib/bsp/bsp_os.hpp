#pragma once

#include "cmsis_os.h"

namespace BSP {
    class OS {
    public:
        static void TaskCreate(osThreadFunc_t func, const char* name, uint32_t stack_bytes, osPriority_t priority);

        static void Delay(uint32_t ms);
    };
}