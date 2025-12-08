#include "scheduler.hpp"
#include "bsp/bsp.hpp"

void Scheduler::RegisterLoop(const char* name, const TaskFunc& func, const UnitFloat<>& freq) {
    for (auto& task : tasks) {
        if (task.name == nullptr) {
            task = {name, func, freq};
            break;
        }
    }
}

void Scheduler::OnLoop() {
    for (auto& task : tasks) {
        if (BSP::Dwt::GetTime() - task.last_exec_time_f64 >= 1.0f / task.freq.toFloat(Hz)) {
            task.exec_delay = (float)(BSP::Dwt::GetTime() - task.last_exec_time_f64) * s;
            task.last_exec_time_f64 = BSP::Dwt::GetTime();
            task.func();
            task.exec_time = (float)(BSP::Dwt::GetTime() - task.last_exec_time_f64) * s;
            task.cpu_usage = task.exec_time / (1.0f / task.freq);
        }
    }

    // 统计总CPU占用率
    total_cpu_usage = 0 * default_unit;
    for (const auto& task : tasks) {
        total_cpu_usage += task.cpu_usage;
    }
}