#pragma once

#include "../leg_protocol.hpp"
#include "power/super_cap.hpp"

// 腿长档位，m
inline constexpr float LEG_LENGTH_LOW = 0.14f;
inline constexpr float LEG_LENGTH_MID = 0.22f;
inline constexpr float LEG_LENGTH_HIGH = 0.33f;

// 超级电容
inline SuperCap super_cap({.can = Can(1)});

// 上下板通信
inline LegDownFrame down;

void chassis_loop();

// 组 0x002 控制帧
void chassis_write_can_frame(CanFrame& frame);

// 接收 0x003 状态帧，由 setup() 注册到 Can(1)
void chassis_can_callback(const CanFrame& frame);
