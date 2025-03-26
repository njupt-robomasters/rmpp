#include "task_protocol.h"
#include "app.hpp"

[[noreturn]] void task_protocol_ui_entry(void const *argument) {
    while (true) {
        // 必须告知机器人ID
        ui.robot_id = referee.robot_id;

        // 从裁判系统获得的数据
        ui.set_center_gain_status = referee.center_gain_status;
        ui.set_chassis_power_limit = referee.chassis_power_limit;

        // 底盘
        ui.set_chassis_max_speed = status.chassis.vxy_limit;
        ui.set_chassis_vr = status.chassis.rpm;
        ui.set_super_cap_percent = 0;

        // 云台
        ui.set_gimbal_is_imu_mode = (status.gimbal.mode == Gimbal::IMU_MODE);
        ui.set_shooter_is_on = status.gimbal.is_prepare_shoot;
        ui.set_bullet_frequency = status.gimbal.shoot_freq;

        // 自瞄
        ui.set_is_locked = false;
        ui.set_is_firing = false;

        ui.Update();
        osDelay(1);
    }
}

void task_protocol_rc_callback(const uint8_t *data, const uint16_t size) {
    dj6.ParseData(data, size);
}

void task_protocol_referee_callback(const uint8_t *data, const uint16_t size) {
    referee.PhaseData(data, size);
}
