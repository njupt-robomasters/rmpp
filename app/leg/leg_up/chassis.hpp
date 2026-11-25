#pragma once

#include "../leg_protocol.hpp"
#include "power/super_cap.hpp"

// 腿长档位，m
inline constexpr float LEG_LENGTH_LOW = 0.14f;
inline constexpr float LEG_LENGTH_MID = 0.22f;
inline constexpr float LEG_LENGTH_HIGH = 0.33f;

// 上下板 CAN1 经滑环连接，协议见 ../leg_protocol.hpp；接收回调在构造时注册
inline LegUpProtocol leg_protocol(Can(1));

// 超级电容
inline SuperCap super_cap({.can = Can(1)});

void chassis_loop();
