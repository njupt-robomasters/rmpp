#pragma once

// 靶车无裁判系统，用超电反馈模拟底盘功率检测与缓冲能量；SWD 观察
inline float REFEREE_LIMIT = 50;  // 裁判电池侧功率上限，W
inline float referee_power = 0;   // 模拟裁判测量功率，W
inline float referee_energy = 60; // 模拟裁判缓冲能量，J

// 10 Hz 结算模拟缓冲能量
void referee_loop();
