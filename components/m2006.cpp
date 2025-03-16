#include "m2006.hpp"

void M2006::Init(const float Kp, const float Ki, const float Kd, const float I_limit) {
    speed_pid.Init(Kp, Ki, Kd, MAX_CURRENT, I_limit);;
}

void M2006::ParseCAN(const uint8_t data[8]) {
    ecd = static_cast<uint16_t>(data[0] << 8 | data[1]);
    speed_rpm = static_cast<float>(static_cast<int16_t>(data[2] << 8 | data[3])) / REDUCTION_RATIO;
    current =  static_cast<float>(static_cast<int16_t>(data[4] << 8 | data[5])) / 10000.0f * MAX_CURRENT;
    temperate = data[6];
}

void M2006::Update(const float speed_rpm_set) {
    this->speed_rpm_set = speed_rpm_set;
    current_set = speed_pid.Calculate(speed_rpm, speed_rpm_set);
}

// 释放电机，关闭动力输出
void M2006::Release() {
    speed_rpm_set = current_set = 0;
    speed_pid.Clear();
}

int16_t M2006::GetCANCmd() {
    // 3508电机（C620电调）：-20~0~20A => -16384~0~16384
    const auto cmd = static_cast<int16_t>(current_set / MAX_CURRENT * 10000.0f);
    return cmd;
}
