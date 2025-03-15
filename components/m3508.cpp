#include "m3508.hpp"

void M3508::Init(const float Kp, const float Ki, const float Kd, const float max_Iout) {
    speed_pid.Init(PID::PID_POSITION, Kp, Ki, Kd, MAX_CURRENT, max_Iout);;
}

void M3508::ParseCAN(const uint8_t data[8]) {
    ecd = static_cast<uint16_t>(data[0] << 8 | data[1]);
    speed_rpm = static_cast<float>(static_cast<int16_t>(data[2] << 8 | data[3])) / REDUCTION_RATIO;
    current =  static_cast<float>(static_cast<int16_t>(data[4] << 8 | data[5])) / 16384.0f * MAX_CURRENT;
    temperate = data[6];
}

void M3508::Update(const float speed_rpm_) {
    speed_rpm_set = speed_rpm_;
    current_set = speed_pid.Update(speed_rpm, speed_rpm_set);
}

// 释放电机，关闭动力输出
void M3508::Release() {
    speed_rpm_set = current_set = 0;
    speed_pid.Clear();
}

int16_t M3508::GetCurrentCmd() {
    // 3508电机（C620电调）：-20~0~20A => -16384~0~16384
    const auto cmd = static_cast<int16_t>(current_set / MAX_CURRENT * 16384.0f);
    return cmd;
}
