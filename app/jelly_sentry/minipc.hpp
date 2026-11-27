#pragma once

#include "minipc/aim.hpp"
#include "minipc/nav.hpp"

// 上位机独立通道：CDC1 自瞄，CDC2 导航（单字节帧头 + 载荷 + MODBUS CRC）
inline Aim aim({.cdc = Cdc(1)});
inline Nav nav({.cdc = Cdc(2)});

// 最近有效导航帧的过洞状态，minipc_loop() 更新，断联时保留
inline bool is_nav_through_hole = false;

void minipc_loop();
