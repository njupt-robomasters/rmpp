#pragma once

#include "minipc/aim.hpp"
#include "minipc/nav.hpp"

// 上位机独立通道：CDC1 自瞄，CDC2 导航（单字节帧头 + 载荷 + MODBUS CRC）
inline Aim aim({.cdc = Cdc(1)});
inline Nav nav({.cdc = Cdc(2)});

void minipc_loop();
