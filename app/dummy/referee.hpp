#pragma once

// 靶车无裁判系统，用超电反馈模拟底盘功率检测与缓冲能量；SWD 观察
inline constexpr float BATTERY_LIMIT = 20; // 电池侧功率上限，W
inline float referee_power = 0;            // 模拟裁判功率，W
inline float buffer_energy = 60;           // 模拟缓冲能量，J

// 10 Hz 结算模拟缓冲能量
void referee_loop();
