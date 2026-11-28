#pragma once

#include "math/unit.hpp"
#include <cstdint>

// 匹配模式
inline constexpr float CENTER_X = -4.954f, CENTER_Y = 2.097f;
inline constexpr float HOME_X_BLUE = 0.197f, HOME_Y_BLUE = -0.301f;
inline constexpr float HOME_X_RED = -10.160f, HOME_Y_RED = 4.663f;

// 建图模式
// inline constexpr float CENTER_X = -4.954f, CENTER_Y = 2.097f;
// inline constexpr float HOME_X_BLUE = 0.0f, HOME_Y_BLUE = 0.0f;
// inline constexpr float HOME_X_RED = 0.0f, HOME_Y_RED = 0.0f;

// 调试模式
// inline constexpr float CENTER_X = 1.0f, CENTER_Y = 0.0f;
// inline constexpr float HOME_X_BLUE = 0.0f, HOME_Y_BLUE = 0.0f;
// inline constexpr float HOME_X_RED = 0.0f, HOME_Y_RED = 0.0f;

// 返航血量：低于 100 HP 回家，回满 maximum_HP 后再出击
inline constexpr uint16_t GO_HOME_HP = 100;

// 扫描：yaw 连续旋转，pitch 目标在编码器软限位间往返，rad/s
inline constexpr float SCAN_YAW_SPEED = 180 * deg_s, SCAN_PITCH_SPEED = 60 * deg_s;

// 锁定：自瞄有效距离，m；脱锁时间，s
inline constexpr float LOCK_DISTANCE = 5, LOCK_TIMEOUT = 500 * ms;

// 底盘：按血量更新导航目标，雷达系速度叠加遥控平移；小陀螺取 `vt13.config.wr_max`
void sentry_chassis();

// 云台：测试模式人工输入接管；不在回家途中时锁定目标，否则扫描
void sentry_gimbal();

// 发射机构：比赛自动开摩擦轮并放行开火，场下测试需手动 S 挡开轮并作为开火批准
void sentry_shooter();
