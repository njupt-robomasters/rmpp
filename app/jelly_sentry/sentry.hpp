#pragma once

#include "math/angle.hpp"

// 自主行为：比赛，或 VT13 按 fn 锁存时接管底盘、云台与发射机构
// 底盘按血量切换导航目标点并小陀螺，云台扫描并在允许时锁定自瞄目标，发射机构按视觉开火许可发弹

// 底盘：导航目标点与速度，过洞随头、其余小陀螺
void sentry_chassis();

// 云台：有效距离内识别到目标时锁定视觉角度，否则扫描；低血量回家期间只扫描；自检阶段扫描叠加匀速旋转
void sentry_gimbal(bool is_detect, Angle target_yaw, Angle target_pitch, float distance);

// 裁判自检阶段的云台 yaw 匀速旋转，rad/s；遥控与自主叠加，非自检阶段为 0
float sentry_self_check_yaw_speed();

// 发射机构：摩擦轮常开，视觉开火许可时按设定弹频
void sentry_shooter();
