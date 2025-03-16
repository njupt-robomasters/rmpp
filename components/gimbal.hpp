#pragma once

#include "m6020.hpp"
#include "m2006.hpp"

class Gimbal {
public:
    void Init();

    void ParseCAN(uint32_t id, uint8_t data[8]);

    void Update(float pitch_angle_set, float yaw_angle_set, float shoot_rpm_set);

    void Release();

private:
    // PID参数
    // pitch
    static constexpr auto PITCH_KP = 0.0f;
    static constexpr auto PITCH_KI = 0.0f;
    static constexpr auto PITCH_KD = 0.0f;
    static constexpr auto PITCH_MAX_IOUT = 0.0f;
    // yaw
    static constexpr auto YAW_KP = 0.0f;
    static constexpr auto YAW_KI = 0.0f;
    static constexpr auto YAW_KD = 0.0f;
    static constexpr auto YAW_MAX_IOUT = 0.0f;
    // shoot
    static constexpr auto SHOOT_KP = M2006::MAX_CURRENT / 7.5f / 5.0f;
    static constexpr auto SHOOT_KI = 0.0f;
    static constexpr auto SHOOT_KD = 0.0f;
    static constexpr auto SHOOT_MAX_IOUT = 0.0f;

    // 云台设定值
    float pitch_angle_set = 0, yaw_angle_set = 0;
    float shoot_rpm_set = 0;

    // 云台实际值
    float pitch_angle = 0, yaw_angle = 0;
    float shoot_rpm = 0;

    M6020 m_pitch, m_yaw;
    M2006 m_shoot;

    void sendCANCmd();
};
