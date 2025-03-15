#pragma once

#include <cstdint>

class PID {
public:
    enum PID_MODE {
        PID_POSITION = 0,
        PID_DELTA
    };

    float now = 0.0f;
    float set = 0.0f;

    // PID输出
    float out = 0.0f;
    float Pout = 0.0f;
    float Iout = 0.0f;
    float Dout = 0.0f;

    void Init(PID_MODE mode_, float Kp_, float Ki_, float Kd_, float max_out_, float max_Iout_);

    float Update(float now, float set);

    void Clear();

private:
    uint8_t mode{};

    // PID 三参数
    float Kp = 0.0f;
    float Ki = 0.0f;
    float Kd = 0.0f;

    float max_out = 0.0f; // 最大输出
    float max_Iout = 0.0f; // 最大积分输出

    float error[3]{}; // 误差项 0最新 1上一次 2上上次
    float Dbuf[3]{}; // 微分项 0最新 1上一次 2上上次
};
