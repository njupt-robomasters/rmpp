#include "gimbal.hpp"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "utils.hpp"

Gimbal::Gimbal(const IMU &imu, const mit_t &pitch_mit, const mit_t &yaw_mit, const mit_t &shoot_mit) : imu(imu),
    yaw_mode(yaw_mode),
    m_pitch(pitch_mit),
    m_yaw(yaw_mit),
    m_shoot(shoot_mit) {
}

void Gimbal::Init() {
    // 等待pitch yaw电机启动
    m_pitch.is_ready = false;
    m_yaw.is_ready = false;
    while (m_pitch.is_ready == false) {
        osDelay(1);
    }
    while (m_yaw.is_ready == false) {
        osDelay(1);
    }

    // pitch复位
    m_pitch.current_ref = -1.0f;
    m_yaw.current_ref = 0.0f;
    m_shoot.current_ref = 0.0f;
    for (int i = 0; i < PITCH_RESET_MS; i++) {
        sendCANCmd();
        osDelay(1);
    }
    pitch_offset = m_pitch.angle;

    // 设置当前位置为目标位置
    SetCurrentAsTarget();
}

void Gimbal::ParseCAN(const uint32_t id, uint8_t data[8]) {
    if (id == 0x205) // yaw
        m_yaw.ParseCAN(data);
    if (id == 0x206) // pitch
        m_pitch.ParseCAN(data);
    if (id == 0x207) // shoot
        m_shoot.ParseCAN(data);

    pitch_angle_absolute = m_pitch.angle;
    pitch_angle = norm_angle(pitch_angle_absolute - pitch_offset);

    yaw_angle_absolute = m_yaw.angle;
    yaw_angle = norm_angle(yaw_angle_absolute - YAW_OFFSET);

    shoot_speed_tps = m_shoot.v_tps_lpf;
    shoot_freq = shoot_speed_tps * SHOOT_NUM_PER_ROUND;
}

void Gimbal::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable)
        return;

    this->is_enable = is_enable;
    m_pitch.is_enable = is_enable;
    m_yaw.is_enable = is_enable;
    m_shoot.is_enable = is_enable;

    if (is_enable)
        SetCurrentAsTarget();
}

void Gimbal::SetYawMode(const yaw_mode_t yaw_mode) {
    if (this->yaw_mode == yaw_mode)
        return;

    this->yaw_mode = yaw_mode;
    // 切换模式后，把yaw当前位置设置为目标位置
    if (this->yaw_mode == ECD_MODE) {
        yaw_angle_set = yaw_angle_absolute - YAW_OFFSET;
    } else if (this->yaw_mode == IMU_MODE) {
        yaw_angle_set = imu.yaw;
    }
}

// 设置当前位置为目标位置
void Gimbal::SetCurrentAsTarget() {
    pitch_angle_set = pitch_angle_absolute - pitch_offset;
    if (this->yaw_mode == ECD_MODE) {
        yaw_angle_set = yaw_angle_absolute - YAW_OFFSET;
    } else if (this->yaw_mode == IMU_MODE) {
        yaw_angle_set = imu.yaw;
    }
}

void Gimbal::SetAngle(const float pitch_angle_set, const float yaw_angle_set) {
    this->pitch_angle_set = pitch_angle_set;
    this->yaw_angle_set = yaw_angle_set;
}

void Gimbal::AddAngle(const float pitch_angle_add, const float yaw_angle_add) {
    pitch_angle_set += pitch_angle_add;
    clamp(pitch_angle, 0, PITCH_RANGE);
    yaw_angle_set = norm_angle(yaw_angle_set + yaw_angle_add);
}

void Gimbal::SetShootFreq(const float shoot_freq_set) {
    this->shoot_freq_set = shoot_freq_set;
}

void Gimbal::SetYAW_V_APS_FF(const float yaw_v_aps_ff) {
    this->yaw_v_aps_ff = yaw_v_aps_ff;
}

void Gimbal::Update() {
    if (is_enable) {
        // pitch
        pitch_angle_absolute_set = norm_angle(pitch_angle_set + pitch_offset);
        m_pitch.SetAngle(pitch_angle_absolute_set);

        // yaw
        if (yaw_mode == ECD_MODE) {
            yaw_angle_absolute_set = norm_angle(yaw_angle_set + YAW_OFFSET);
        } else if (yaw_mode == IMU_MODE) {
            // 抵消掉电机闭环中原来的 -angle 项，最终误差计算公式变成 yaw_angle_set - imu.yaw
            yaw_angle_absolute_set = norm_angle(yaw_angle_set - imu.yaw + yaw_angle_absolute);
        }
        m_yaw.SetAngle(yaw_angle_absolute_set);
        m_yaw.SetV_APS_FF(yaw_v_aps_ff);

        // shoot
        shoot_v_tps_set = shoot_freq_set / SHOOT_NUM_PER_ROUND;
        m_shoot.SetV_TPS(shoot_v_tps_set);
    }

    // 电机闭环控制计算（即使使能也要调用，防止dt计算出现问题）
    m_pitch.Update();
    m_yaw.Update();
    m_shoot.Update();

    // CAN发送控制报文
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
