#pragma once

#include "math/angle.hpp"
#include <cstdint>

// 比赛导航状态，sentry_update() 更新，供模块与 SWD 查看
enum class NavState : uint8_t {
    IDLE,      // 非自主阶段，导航目标清零
    GO_CENTER, // 前往中心点，允许自瞄锁定与开火
    AT_CENTER, // 裁判 RFID 确认驻留中心点，允许自瞄锁定与开火
    GO_HOME,   // 血量低于 100 回家，满血前屏蔽自瞄锁定与开火
};
inline NavState nav_state = NavState::IDLE;

// 战术状态：正赛或 fn 测试时按血量和 RFID 更新导航目标，其余阶段复位
void sentry_update();

// 底盘：雷达系导航叠加遥控平移；过洞随头，其余 90～60 rpm 小陀螺
void sentry_chassis();

// 云台：过洞停扫；场下人工输入接管；允许交战时锁定目标，否则扫描
void sentry_gimbal(bool is_detect, Angle target_yaw, Angle target_pitch, float distance);

// 裁判自检阶段的云台 yaw 匀速旋转，rad/s；非自检阶段为 0
float sentry_self_check_yaw_speed();

// 发射机构：自主摩擦轮常开，允许交战且视觉许可时拨弹
void sentry_shooter();

// 当前战术阶段允许自瞄锁定与开火
inline bool sentry_can_engage() {
    return nav_state == NavState::GO_CENTER || nav_state == NavState::AT_CENTER;
}
