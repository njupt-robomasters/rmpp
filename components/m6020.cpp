#include "m6020.hpp"
#include <cmath>

void M6020::Init(const float Kp, const float Ki, const float Kd, const float max_Iout) {
    pid.Init(PID::PID_POSITION, Kp, Ki, Kd, MAX_CURRENT, max_Iout);;
}

void M6020::ParseCAN(const uint8_t data[8]) {
    ecd = static_cast<uint16_t>(data[0] << 8 | data[1]);
    speed_rpm = static_cast<float>(static_cast<int16_t>(data[2] << 8 | data[3]));
    current =  static_cast<float>(static_cast<int16_t>(data[4] << 8 | data[5])) / 16384.0f * MAX_CURRENT;
    temperate = data[6];

    angle = static_cast<float>(ecd) / 8192.0f * 360.0f;
}

void M6020::Update(const float angle_) {
    angle_set = angle_;
    const float angle_err = calcAngleErr(speed_rpm, angle_set);
    current_set = pid.Update(angle_err, 0);
}

// 释放电机，关闭动力输出
void M6020::Release() {
    angle_set = current_set = 0;
    pid.Clear();
}

int16_t M6020::GetCurrentCmd() {
    // 3508电机（C620电调）：-3~0~3A => -16384~0~16384
    const auto cmd = static_cast<int16_t>(current_set / MAX_CURRENT * 16384.0f);
    return cmd;
}

float M6020::normAngle(const float angle) {
    float result = fmodf(angle, 360.0);
    if (result < 0) {
        result += 360.0;
    }
    return result;
}

float M6020::calcAngleErr(float now, float set) {
    now = normAngle(now);
    set = normAngle(set);

    const float err = fmodf(set - now + 540.0f, 360.0f) - 180.0f; // 最短路径误差

    return err;
}
