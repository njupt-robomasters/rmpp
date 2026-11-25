#pragma once

#include "referee/referee.hpp"
#include "referee/robot_ui.hpp"

// 裁判系统：电源管理模块接下板 USART1，经 CAN1 由下板 RefereeRelay 转发；接收 ID 0x00、发送 ID 0x01
inline Referee referee({.can = Can(1)});
inline RefereeSender referee_sender({.can = Can(1)});
inline RobotUi ui({});

void referee_loop();
