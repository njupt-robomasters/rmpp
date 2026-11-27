#include "chassis.hpp"
#include "app.hpp"
#include "gimbal.hpp"
#include "referee.hpp"

// 操控参数
static constexpr float VXY_MAX = 3.0f;     // 平移最大速度，m/s
static constexpr float WR_MAX = 150 * rpm; // 手动旋转最大角速度
static constexpr float SPIN_WR = 60 * rpm; // 小陀螺角速度

void chassis_loop() {
    const bool is_enable = fsi6x.swa == FSi6X::DOWN; // SWA 下挡使能
    const bool is_spin = fsi6x.swd == FSi6X::DOWN;   // SWD 下挡小陀螺
    const bool is_follow = fsi6x.swb == FSi6X::DOWN; // SWB 下挡跟随云台

    // 遥控：右摇杆平移；小陀螺固定角速度，否则不跟随时左摇杆水平控制底盘旋转
    float vx = 0, vy = 0, wr = 0;
    if (is_enable) {
        vx = fsi6x.x * VXY_MAX;
        vy = fsi6x.y * VXY_MAX;
        if (is_spin)
            wr = SPIN_WR;
        else if (!is_follow)
            wr = fsi6x.yaw * WR_MAX;
    }

    // 底盘控制：小陀螺或跟随时以云台为前方，小陀螺角速度优先于跟随；云台离线时不跟随，以底盘原始前方
    const bool is_gimbal_front = (is_spin || is_follow) && yaw.is_connect;
    chassis.set_enable(is_enable);
    chassis.set_mode(is_follow && yaw.is_connect ? ChassisMatrix::FOLLOW_MODE : ChassisMatrix::DETACH_MODE);
    chassis.set_gimbal_yaw(is_gimbal_front ? yaw.angle : 0.0f);
    chassis.set_speed(vx, vy, wr);

    // 超电：遥控在线时 SWC 中挡充电、下挡叠加放电补偿；断联时全部关闭
    const auto swc = fsi6x.is_connect ? fsi6x.swc : FSi6X::ERR;
    super_cap.set_enable(swc == FSi6X::MID || swc == FSi6X::DOWN);
    super_cap.set_boost_enable(swc == FSi6X::DOWN);

    // 裁判电池侧功率上限与缓冲能量由 referee_loop 模拟
    super_cap.set_referee_limit(REFEREE_LIMIT);
    super_cap.set_referee_energy(referee_energy);
    super_cap.update();

    // 底盘功率目标由超电给出
    chassis.set_power_ref(super_cap.power_chassis_ref);
    chassis.update();
}
