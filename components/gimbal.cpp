#include "gimbal.hpp"
#include "bsp_can.h"

Gimbal::Gimbal() : m_pitch(PITCH_KP, PITCH_KD, PITCH_FEEDFORAWD),
                   m_yaw(YAW_KP, YAW_KD, 0),
                   m_shoot(SHOOT_KP, 0, 0) {

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
    shoot_freq = m_shoot.speed_tps_lpf * SHOOT_NUM_PER_ROUND;
}

void Gimbal::Update(const float pitch_angle_set, const float yaw_angle_set, const float shoot_freq_set) {
    this->pitch_angle_set = pitch_angle_set;
    this->yaw_angle_set = yaw_angle_set;
    this->shoot_freq_set = shoot_freq_set;

    // 电机PID计算
    m_pitch.Update(pitch_angle_set);
    m_yaw.Update(yaw_angle_set);
    m_shoot.Update(shoot_freq_set / SHOOT_NUM_PER_ROUND);

    // CAN发送控制报文
    sendCANCmd();
}

void Gimbal::Release() {
    m_pitch.Release();
    m_yaw.Release();
    m_shoot.Release();

    sendCANCmd();
}

void Gimbal::sendCANCmd() const {
    const int16_t pitch_cmd = m_pitch.GetCANCmd();
    const int16_t yaw_cmd = m_yaw.GetCANCmd();
    const int16_t shoot_cmd = m_shoot.GetCANCmd();

    uint8_t data[8];
    data[0] = yaw_cmd >> 8; // yaw，6020电机，ID：5
    data[1] = yaw_cmd;
    data[2] = pitch_cmd >> 8; // pitch，6020电机，ID：6
    data[3] = pitch_cmd;
    data[4] = shoot_cmd >> 8; // shoot，2006电机，ID：7
    data[5] = shoot_cmd;
    data[6] = 0;
    data[7] = 0;
    BSP_CAN_Transmit(0x1FF, data, 8);
}
