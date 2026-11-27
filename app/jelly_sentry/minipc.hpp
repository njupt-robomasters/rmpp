#pragma once

#include "minipc/aim.hpp"
#include "minipc/nav.hpp"
#include <cstdint>

// 上位机独立通道：CDC1 自瞄，CDC2 导航（单字节帧头 + 载荷 + MODBUS CRC）
inline Aim aim({.cdc = Cdc(1)});
inline Nav nav({.cdc = Cdc(2)});

// 比赛导航状态机，minipc_loop() 更新，供各模块与 SWD 查看
enum class NavState : uint8_t {
    IDLE,      // 非自主阶段：目标点清零
    GO_CENTER, // 前往中心点：小陀螺移动，云台扫描并响应自瞄击打
    AT_CENTER, // 到达中心点：裁判 RFID 确认在点，保持小陀螺、扫描与自瞄击打
    GO_HOME,   // 残血回家：小陀螺移动，保持扫描，屏蔽自瞄锁定与开火直至回满血
};
inline NavState nav_state = NavState::IDLE;

// 最近有效导航帧的过洞状态，minipc_loop() 更新，断联时保留
inline bool is_nav_through_hole = false;

// 当前自主状态是否允许自瞄锁定与开火（前往中心与驻留中心时允许，回家与赛前禁止）
inline bool sentry_can_engage() {
    return nav_state == NavState::GO_CENTER || nav_state == NavState::AT_CENTER;
}

void minipc_loop();
