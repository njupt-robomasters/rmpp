#pragma once

#include "melody/melody.hpp"
#include "rc/fsi6x.hpp"
#include "referee/referee_relay.hpp"

inline Melody melody({});
inline FSi6X fsi6x({.uart = Uart(3)});

// 裁判系统：USART1 接电源管理模块，经 CAN1 与上板 Referee 互转；按发送板命名，0x00 为下板发送、0x01 为上板发送
inline RefereeRelay referee_relay({.uart = Uart(1), .can = Can(1)});
