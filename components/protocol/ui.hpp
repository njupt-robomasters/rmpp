#pragma once

#include <cstdint>

class UI {
public:
    void Init(uint8_t robot_id); // 初始化UI

    void Update(); // 更新UI

    typedef enum {
        TURNING,
        FOLLOW
    } chassis_mode_t;

    uint8_t set_bullet_frequency = 10; // 弹丸发射频率
    uint8_t set_gimbal_is_imu_mode = 1; // 云台模式，是imu模式为1，否则为0
    uint8_t set_chassis_max_speed = 1; // 底盘平移速度
    uint8_t set_is_locked = 0; // 自瞄是否锁定
    uint8_t set_is_firing = 0; // 是否自动火控
    chassis_mode_t set_chassis_mode = FOLLOW; // 底盘模式
    uint16_t set_chassis_vr = 0; // 底盘自转速度
    uint8_t set_center_gain_status = 0; // 中心增益点占领情况
    float set_super_cap_percent = 0; // 超电百分比
    uint8_t set_shooter_is_on = 0; // 摩擦轮电机状态

private:
};
