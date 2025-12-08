#pragma once

#include <cstdint>
#include <functional>
#include "unit/include_me.hpp"

class Scheduler {
public:
    static constexpr size_t MAX_TASK_NUM = 100;

    using TaskFunc = std::function<void()>;

    struct task_t {
        const char* name;
        TaskFunc func;
        UnitFloat<Hz> freq;
        double last_exec_time_f64; // 上次执行时间
        UnitFloat<us> exec_delay; // 比预期执行延迟的时间
        UnitFloat<us> exec_time; // 本次执行时间
        UnitFloat<pct> cpu_usage; // cpu使用率
    } tasks[MAX_TASK_NUM] = {};

    UnitFloat<pct> total_cpu_usage;

    void RegisterLoop(const char* name, const TaskFunc& func, const UnitFloat<>& freq);

    void OnLoop();
};