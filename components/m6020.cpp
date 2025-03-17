#include "m6020.hpp"
#include <cmath>

void M6020::Init(const float Kp, const float Ki, const float Kd, const float I_limit, const float feedforward) {
    pid.Init(Kp, Ki, Kd, MAX_CURRENT, I_limit);;
    this->feedforward = feedforward;
}

void M6020::ParseCAN(const uint8_t data[8]) {
    ecd = static_cast<uint16_t>(data[0] << 8 | data[1]);
    speed_rpm = static_cast<float>(static_cast<int16_t>(data[2] << 8 | data[3]));
    current =  static_cast<float>(static_cast<int16_t>(data[4] << 8 | data[5])) / 16384.0f * MAX_CURRENT;
    temperate = data[6];

    angle = static_cast<float>(ecd) / 8192.0f * 360.0f;
}

void M6020::Update(const float angle_set) {
    this->angle_set = angle_set;
    const float angle_err = calcAngleErr(angle, angle_set);
    current_set = pid.Calculate(0, angle_err) + feedforward;
    if (current_set > MAX_CURRENT)
        current_set = MAX_CURRENT;
    if (current_set < -MAX_CURRENT)
        current_set = -MAX_CURRENT;
}

// 释放电机，关闭动力输出
void M6020::Release() {
    angle_set = current_set = 0;
    pid.Clear();
}

int16_t M6020::GetCANCmd() {
    // 6020电机：-3~0~3A => -16384~0~16384
    const auto cmd = static_cast<int16_t>(current_set / MAX_CURRENT * 16384.0f);
    return cmd;
}

float M6020::normAngle(const float angle) {
    float result = fmodf(angle, 360.0f);
    if (result < 0) {
        result += 360.0f;
    }
    return result;
}

float M6020::calcAngleErr(float now, float set) {
    now = normAngle(now);
    set = normAngle(set);

    const float err = fmodf(set - now + 540.0f, 360.0f) - 180.0f; // 最短路径误差

    return err;
}
