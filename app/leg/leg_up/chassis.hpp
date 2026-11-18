#pragma once

#include "../leg_protocol.hpp"
#include "power/super_cap.hpp"
#include "math/unit.hpp"

static constexpr float VX_MAX = 3.0f;        // 前向速度上限，m/s
static constexpr float WR_MAX = 360 * deg_s; // 手动航向角速度上限，rad/s

// 超级电容
inline SuperCap super_cap({.can = Can(1)});

// 上下板通信
inline LegDownFrame down;

void chassis_loop();

// 组 0x002 控制帧
void chassis_write_can_frame(CanFrame& frame);

// 接收 0x003 状态帧，由 setup() 注册到 Can(1)
void chassis_can_callback(const CanFrame& frame);
