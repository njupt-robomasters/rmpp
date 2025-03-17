#pragma once

#include "m6020.hpp"
#include "m2006.hpp"

class Gimbal {
public:
    // 云台设定值
    float pitch_angle_set = 0, yaw_angle_set = 0, shoot_freq_set = 0;

    // 云台实际值
    float pitch_angle = 0, yaw_angle = 0, shoot_freq = 0;

    Gimbal();

    void ParseCAN(uint32_t id, uint8_t data[8]);

    void Update(float pitch_angle_set, float yaw_angle_set, float shoot_freq_set);

    void Release();

private:
    // 云台结构参数
    static constexpr float SHOOT_NUM_PER_ROUND = 8.0f; // 拨弹电机每转一圈发射的弹数

    // PID参数
    // pitch
    static constexpr float PITCH_KP = 0.0f;
    static constexpr float PITCH_KD = 0.0f;
    static constexpr float PITCH_FEEDFORAWD = 0.8f; // 前馈
    // yaw
    static constexpr float YAW_KP = 0.7f;
    static constexpr float YAW_KD = 0.012f;
    // shoot
    static constexpr float SHOOT_MAXP_FREQ = 2.0f; // 射频误差达到此值后，P输出拉满【单位：Hz】
    static constexpr float SHOOT_KP = M2006::MAX_CURRENT / (SHOOT_MAXP_FREQ / SHOOT_NUM_PER_ROUND * 60.0f);

    M6020 m_pitch, m_yaw;
    M2006 m_shoot;

    void sendCANCmd() const;
};
