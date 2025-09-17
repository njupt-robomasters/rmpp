#pragma once

#include "cmsis_os.h"

namespace BSP {
    class OS {
    public:
        static void Create(const char* name, os_pthread thread, osPriority priority, uint32_t stacksz);

        static void Delay(uint32_t millisec);
    };
}
