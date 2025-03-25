#include "task_protocol.h"
#include "../app.hpp"

[[noreturn]] void task_protocol_ui_entry(void const *argument) {
    uint32_t cnt = 0;
    while (true) {
        if (++cnt % 10 == 0) {
            // 1Hz
            ui.Init(referee.robot_id);
        }

        ui.set_center_gain_status = referee.center_gain_status;
        ui.set_chassis_power_limit = referee.chassis_power_limit;

        // 底盘
        ui.set_chassis_max_speed = status.chassis_vxy_max;
        ui.set_chassis_vr = status.chassis_vr_rpm;
        // 云台
        ui.set_bullet_frequency = status.shoot_freq;
        ui.set_gimbal_is_imu_mode = (status.gimbal_mode == Gimbal::IMU_MODE);

        // todo
        ui.set_chassis_mode = UI::TURNING;
        ui.set_shooter_is_on = true;
        ui.set_super_cap_percent = 0;
        ui.Update();

        osDelay(100);
    }
}

void task_protocol_rc_callback(const uint8_t *data, const uint16_t size) {
    dj6.ParseData(data, size);
}

void task_protocol_referee_callback(const uint8_t *data, const uint16_t size) {
    referee.PhaseData(data, size);
}
