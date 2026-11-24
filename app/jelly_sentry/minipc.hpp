#pragma once

#include "minipc/aim.hpp"
#include "minipc/nav.hpp"

// 上位机独立通道：CDC1 自瞄，CDC2 导航（单字节帧头 + 载荷 + MODBUS CRC）
inline Aim aim({.cdc = Cdc(1)});
inline Nav nav({.cdc = Cdc(2)});

// 每 5 拍发送一次自瞄上行帧（200 Hz）；需要 attitude 与 referee，故实现放在 minipc.cpp
void minipc_send();
