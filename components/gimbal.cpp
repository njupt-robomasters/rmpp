#include "gimbal.hpp"
#include "bsp_can.h"

void Gimbal::Init() {
    m_pitch.Init(PITCH_KP, PITCH_KI, PITCH_KD, PITCH_MAX_IOUT);
    m_yaw.Init(YAW_KP, YAW_KI, YAW_KD, YAW_MAX_IOUT);
    m_shoot.Init(SHOOT_KP, SHOOT_KI, SHOOT_KD, SHOOT_MAX_IOUT);
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
    shoot_rpm = m_shoot.speed_rpm;
}

void Gimbal::Update(float pitch_angle_, float yaw_angle_, float shoot_rpm_) {
    pitch_angle = pitch_angle_;
    yaw_angle = yaw_angle_;
    shoot_rpm = shoot_rpm_;

    // 电机PID计算
    m_pitch.Update(pitch_angle);
    m_yaw.Update(yaw_angle);
    m_shoot.Update(shoot_rpm);

    sendCANCmd();
}

void Gimbal::Release() {
    pitch_angle_set = yaw_angle_set = shoot_rpm_set = 0;
    m_pitch.Release();
    m_yaw.Release();
    m_shoot.Release();

    sendCANCmd();
}

void Gimbal::sendCANCmd() {
    const int16_t pitch_cmd = m_pitch.GetCurrentCmd();
    const int16_t yaw_cmd = m_yaw.GetCurrentCmd();
    const int16_t shoot_cmd = m_shoot.GetCurrentCmd();

    uint8_t data[8];
    data[0] = yaw_cmd >> 8; // yaw，6020电机，ID：5，电流环模式
    data[1] = yaw_cmd;
    data[2] = pitch_cmd >> 8; // pitch，6020电机，ID：6，电流环模式
    data[3] = pitch_cmd;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    BSP_CAN_Transmit(0x1FE, data, 8);

    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = shoot_cmd >> 8; // shoot，2006电机，ID：7
    data[5] = shoot_cmd;
    data[6] = 0;
    data[7] = 0;
    BSP_CAN_Transmit(0x1FF, data, 8);
}
