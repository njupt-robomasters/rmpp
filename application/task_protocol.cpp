#include "task_protocol.h"
#include "cmsis_os.h"
#include "app_variable.hpp"

[[noreturn]] void task_protocol_entry(void const *argument) {
    while (true) {
        rv2.SendIMUData(imu.roll, imu.pitch, imu.yaw);
        rv2.SendRefereeData(referee.team_is_red, referee.shooter_bullet_speed);
        osDelay(10);
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
