#include "bsp_os.hpp"

using namespace BSP;

void OS::Create(const char* name, const os_pthread thread, const osPriority priority, uint32_t stacksz) {
    const osThreadDef_t os_thread_def = {(char*)name, thread, priority, 0, stacksz, NULL, NULL};
    osThreadCreate(&os_thread_def, NULL);
}

void OS::Delay(const uint32_t millisec) {
    osDelay(millisec);
}
