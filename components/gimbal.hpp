#pragma once

#include "m6020.hpp"
#include "m2006.hpp"

class Gimbal {
public:
    // 云台设定值
    float pitch_angle_set = 0, yaw_angle_set = 0, shoot_freq_set = 0; // 人类友好值
    float pitch_angle_absolute_set = 0, yaw_angle_absolute_set = 0, shoot_speed_tps_set = 0; // 对接电机控制值

    // 云台实际值
    float pitch_angle = 0, yaw_angle = 0, shoot_freq = 0; // 人类友好值
    float pitch_angle_absolute = 0, yaw_angle_absolute = 0, shoot_speed_tps = 0; // 对接电机控制值

    Gimbal();

    void ParseCAN(uint32_t id, uint8_t data[8]);

    void WaitPitchYawStartup();

    void PitchZero();

    void Update(float pitch_angle_set, float yaw_angle_set, float shoot_freq_set);

    void Release();

private:
    static constexpr int PITCH_RESET_MS = 1000; // pitch回零时间【单位：毫秒】
    static constexpr int YAW_OFFSET = 125; // yaw轴偏移【单位：角度】
    static constexpr float SHOOT_NUM_PER_ROUND = 8.0f; // 拨弹电机每转一圈发射的弹数

    // PID参数
    // pitch
    static constexpr float PITCH_KP = 1.0f;
    static constexpr float PITCH_KD = 0.014f;
    static constexpr float PITCH_FEEDFORAWD = 0.4f; // 前馈
    // yaw
    static constexpr float YAW_KP = 0.7f;
    static constexpr float YAW_KD = 0.012f;
    // shoot
    static constexpr float SHOOT_KP = 2.0f;
    static constexpr float SHOOT_FEEDFORWARD = 0.5f;

    float pitch_offset = 0;

    M6020 m_pitch, m_yaw;
    M2006 m_shoot;

    void sendCANCmd() const;
};
