#pragma once

#include "referee/referee.hpp"
#include "referee/robot_ui.hpp"

// 裁判系统经 CAN1 转发模块接入，接收 ID 0x000、发送 ID 0x001
inline Referee referee({.can = Can(1)});
inline RefereeSender referee_sender({.can = Can(1)});
inline RobotUi ui({});

// 接收解析、本机 ID 与 UI 填写；referee_sender.send() 由 app.cpp 的 send() 安排
void referee_loop();
