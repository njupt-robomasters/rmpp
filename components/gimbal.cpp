#include "gimbal.hpp"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "bsp_pwm.h"
#include "app_variable.hpp"
#include "utils.hpp"

Gimbal::Gimbal(const IMU &imu, PID::param_t &pitch_pid, PID::param_t &yaw_pid, PID::param_t &shoot_pid) : imu(imu),
    m_pitch(pitch_pid),
    m_yaw(yaw_pid),
    m_shoot(shoot_pid) {
}

void Gimbal::Init() {
    // 等待pitch yaw电机启动
    m_pitch.ResetReady();
    m_yaw.ResetReady();
    while (not m_pitch.IsReady()) {
        osDelay(1);
    }
    while (not m_yaw.IsReady()) {
        osDelay(1);
    }

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

    measure.pitch.absolute = m_pitch.measure.angle; // pitch绝对角度
    measure.pitch.relative.degree = norm_angle(measure.pitch.absolute.degree - PITCH_MID); // pitch相对角度

    measure.yaw.absolute = m_yaw.measure.angle; // yaw绝对角度
    measure.yaw.relative.degree = norm_angle(measure.yaw.absolute.degree - YAW_OFFSET); // yaw相对角度

    measure.shoot.speed = m_shoot.measure.speed; // shoot电机转速
    measure.shoot.freq = measure.shoot.speed.tps * SHOOT_NUM_PER_ROUND; // shoot射频
}

void Gimbal::ResetReady() {
    m_pitch.ResetReady();
    m_yaw.ResetReady();
    m_shoot.ResetReady();
}

bool Gimbal::CheckReady() const {
    if (not m_pitch.IsReady())
        return false;
    if (not m_yaw.IsReady())
        return false;
    if (not m_shoot.IsReady())
        return false;
    return true;
}

void Gimbal::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable)
        return;

    this->is_enable = is_enable;

    // 设置子设备
    m_pitch.SetEnable(is_enable);
    m_yaw.SetEnable(is_enable);
    m_shoot.SetEnable(is_enable);
    SetPrepareShoot(is_enable);

    if (is_enable) {
        // 使能后，把当前位置设为目标位置
        SetCurrentAsTarget();
    } else {
        // 失能向电机发送0电流
        sendCurrentCmd();
    }
}

void Gimbal::SetYawMode(const mode_e yaw_mode) {
    if (this->mode == yaw_mode)
        return;

    this->mode = yaw_mode;
    SetCurrentAsTarget(); // 切换模式后，把当前位置设为目标位置
}

// 设置当前位置为目标位置
void Gimbal::SetCurrentAsTarget() {
    if (mode == ECD_MODE) {
        ref.pitch.relative = measure.pitch.relative; // pitch
        ref.yaw.relative = measure.yaw.relative; // yaw
    } else if (mode == IMU_MODE) {
        ref.pitch.imu_mode.degree = imu.pitch; // pitch
        ref.yaw.imu_mode.degree = imu.yaw; // yaw
    }
}

void Gimbal::SetAngle(const float pitch_degree, const float yaw_degree) {
    if (mode == ECD_MODE) {
        ref.pitch.relative.degree = pitch_degree;
        ref.yaw.relative.degree = yaw_degree;
    } else if (mode == IMU_MODE) {
        ref.pitch.imu_mode.degree = pitch_degree;
        ref.yaw.imu_mode.degree = yaw_degree;
    }
}

void Gimbal::AddAngle(const float pitch_degree, const float yaw_degree) {
    if (mode == ECD_MODE) {
        ref.pitch.relative.degree = norm_angle(ref.pitch.relative.degree + pitch_degree);
        ref.yaw.relative.degree = norm_angle(ref.yaw.relative.degree + yaw_degree);
    } else if (mode == IMU_MODE) {
        ref.pitch.imu_mode.degree = norm_angle(ref.pitch.imu_mode.degree + pitch_degree);
        ref.yaw.imu_mode.degree = norm_angle(ref.yaw.imu_mode.degree + yaw_degree);
    }
}

void Gimbal::SetYawSpeedFF(const Speed &yaw_speed_ff) {
    this->yaw_speed_ff = yaw_speed_ff;
}

void Gimbal::SetPrepareShoot(const bool is_on) {
    if (is_on) {
        BSP_PWM_SetDuty(100);
    } else {
        BSP_PWM_SetDuty(0);
    }
}

void Gimbal::SetShootFreq(const float shoot_freq) {
    ref.shoot.freq = shoot_freq;
}

void Gimbal::Update() {
    if (is_enable) {
        // pitch、yaw
        if (mode == ECD_MODE) {
            // pitch
            ref.pitch.absolute.degree = norm_angle(ref.pitch.relative.degree + PITCH_MID);
            // yaw
            ref.yaw.absolute.degree = norm_angle(ref.yaw.relative.degree + YAW_OFFSET);
        } else if (mode == IMU_MODE) {
            // 此处加上 measure，抵消掉电机PID计算中的 -measure，最终误差变成 ref - imu.yaw
            // pitch
            ref.pitch.absolute.degree = norm_angle(ref.pitch.imu_mode.degree - imu.pitch + measure.pitch.absolute.degree);
            // yaw
            ref.yaw.absolute.degree = norm_angle(ref.yaw.imu_mode.degree - imu.yaw + measure.yaw.absolute.degree);
        }
        ref.pitch.absolute.degree = clamp(ref.pitch.absolute.degree, PITCH_MIN, PITCH_MAX);
        ref.pitch.relative.degree = norm_angle(ref.pitch.absolute.degree - PITCH_MID);
        m_pitch.SetAngle(ref.pitch.absolute);
        m_yaw.SetAngle(ref.yaw.absolute, yaw_speed_ff);

        // shoot
        ref.shoot.speed.tps = ref.shoot.freq / SHOOT_NUM_PER_ROUND;
        m_shoot.SetSpeed(ref.shoot.speed);

        // 电机闭环控制计算
        m_pitch.Update();
        m_yaw.Update();
        m_shoot.Update();
    }

    // CAN发送控制报文
    sendCurrentCmd();
}

void Gimbal::sendCurrentCmd() const {
    const int16_t pitch_cmd = m_pitch.GetCurrentCMD();
    const int16_t yaw_cmd = m_yaw.GetCurrentCMD();
    const int16_t shoot_cmd = m_shoot.GetCurrentCMD();

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
