#include "task_protocol.h"
#include "cmsis_os.h"
#include "app_variable.hpp"

[[noreturn]] void task_protocol_entry(void const *argument) {
    uint32_t cnt = 0;
    while (true) {
        if (++cnt % 10 == 0) { // 100Hz
            rv2.SendIMUData(imu.roll, imu.pitch, imu.yaw);
            rv2.SendRefereeData(referee.team_is_red, referee.shooter_bullet_speed);
        }

        // if (++cnt % 1000 == 0) { // 1Hz
        //     ui.Init(referee.robot_id);
        // }
        //
        // if (++cnt % 50 == 0) { // 20Hz
        //     ui.set_bullet_frequency = static_cast<uint8_t>(settings.shoot_freq);
        //     ui.set_center_gain_status = referee.center_gain_status;
        //     ui.set_chassis_max_speed =  static_cast<uint8_t>(settings.chassis_vxy_max);
        //     ui.set_chassis_mode = UI::FOLLOW;
        //     ui.set_chassis_vr =  static_cast<uint8_t>(settings.chassis_vr_max_rpm);
        //     ui.set_gimbal_is_imu_mode = (settings.gimbal_mode == Gimbal::IMU_MODE);
        //     ui.set_is_firing = referee.mouse_right_button_down;
        //     ui.set_is_locked = rv2.is_locked;
        //     ui.set_shooter_is_on = false; // todo
        //     ui.set_super_cap_percent = 0; // todo
        //     ui.Update();
        // }

        osDelay(1);
    }
}

void task_protocol_rc_callback(const uint8_t *data, const uint16_t size) {
    dj6.ParseData(data, size);
}

void task_protocol_referee_callback(const uint8_t *data, const uint16_t size) {
    referee.PhaseData(data, size);
}

void task_protocol_cdc_callback(const uint8_t *data, const int size) {
    rv2.ParseStreamingData(data, size);
}
