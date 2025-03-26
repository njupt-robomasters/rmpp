#pragma once

#include "ui.h"
#include <cstdint>

class UI {
public:
    void ForceInit();

    uint8_t Update(); // 更新UI

    typedef enum {
        TURNING,
        FOLLOW
    } chassis_mode_t;

    uint8_t robot_id = 0;

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
    uint8_t set_chassis_power_limit = 0;

private:
    uint8_t set_bullet_frequency_last = 10; // 弹丸发射频率
    uint8_t set_gimbal_is_imu_mode_last = 1; // 云台模式，是imu模式为1，否则为0
    uint8_t set_chassis_max_speed_last = 1; // 底盘平移速度
    uint8_t set_is_locked_last = 0; // 自瞄是否锁定
    uint8_t set_is_firing_last = 0; // 是否自动火控
    chassis_mode_t set_chassis_mode_last = FOLLOW; // 底盘模式
    uint16_t set_chassis_vr_last = 0; // 底盘自转速度
    uint8_t set_center_gain_status_last = 0; // 中心增益点占领情况
    float set_super_cap_percent_last = 0; // 超电百分比
    uint8_t set_shooter_is_on_last = 0; // 摩擦轮电机状态
    uint8_t set_chassis_power_limit_last = 0;

    uint8_t is_force_init = 0; // 是否init

    void save_last(); // 保存上次值

    void show_string(ui_interface_string_t *ui_string);

    void hide_string(ui_interface_string_t *ui_string);
};
