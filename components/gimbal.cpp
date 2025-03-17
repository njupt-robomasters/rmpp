#include "gimbal.hpp"
#include "bsp_can.h"

void Gimbal::Init() {
    m_pitch.Init(PITCH_KP, PITCH_KI, PITCH_KD, PITCH_ILimit, PITCH_FEEDFORAWD);
    m_yaw.Init(YAW_KP, YAW_KI, YAW_KD, YAW_MAX_ILimit, 0);
    m_shoot.Init(SHOOT_KP, SHOOT_KI, SHOOT_KD, SHOOT_ILimit);
    Release();
}

void Gimbal::ParseCAN(const uint32_t id, uint8_t data[8]) {
    if (id == 0x205) // yaw
        m_yaw.ParseCAN(data);
    if (id == 0x206) // pitch
        m_pitch.ParseCAN(data);
    if (id == 0x207) // shoot
        m_shoot.ParseCAN(data);
    pitch_angle = m_pitch.angle;
    yaw_angle = m_yaw.angle;
    shoot_freq = m_shoot.speed_rpm / 60.0f * SHOOT_NUM_PER_ROUND;
}

void Gimbal::Update(const float pitch_angle_set, const float yaw_angle_set, const float shoot_freq_set) {
    this->pitch_angle_set = pitch_angle_set;
    this->yaw_angle_set = yaw_angle_set;
    this->shoot_freq_set = shoot_freq_set;

    // 电机PID计算
    m_pitch.Update(pitch_angle_set);
    m_yaw.Update(yaw_angle_set);
    m_shoot.Update(shoot_freq_set / SHOOT_NUM_PER_ROUND * 60.0f);

    sendCANCmd();
}

void Gimbal::Release() {
    m_pitch.Release();
    m_yaw.Release();
    m_shoot.Release();

    sendCANCmd();
}

void Gimbal::sendCANCmd() {
    const int16_t pitch_cmd = m_pitch.GetCANCmd();
    const int16_t yaw_cmd = m_yaw.GetCANCmd();
    const int16_t shoot_cmd = m_shoot.GetCANCmd();

    uint8_t data[8];
    data[0] = yaw_cmd >> 8; // yaw，6020电机，ID：5，电流环模式
    data[1] = yaw_cmd;
    data[2] = pitch_cmd >> 8; // pitch，6020电机，ID：6，电流环模式
    data[3] = pitch_cmd;
    data[4] = shoot_cmd >> 8; // shoot，2006电机，ID：7
    data[5] = shoot_cmd;
    data[6] = 0;
    data[7] = 0;
    BSP_CAN_Transmit(0x1FF, data, 8);
}
