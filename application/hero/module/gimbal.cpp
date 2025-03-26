#include "gimbal.hpp"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "bsp_pwm.h"

Gimbal::Gimbal(const IMU &imu, PID::param_t &pitch_pid, PID::param_t &yaw_pid, PID::param_t &shoot_pid) : imu(imu),
    m_pitch(1, pitch_pid),
    m_yaw(yaw_pid),
    m_shoot(2, shoot_pid) {
}

void Gimbal::Init() {
    // 等待所有电机启动
    ResetReady();
    while (not CheckReady()) {
        m_pitch.SendCANEnable();
        osDelay(1);
        m_shoot.SendCANEnable();
        osDelay(1);
    }

    // 设置当前位置为目标位置
    SetCurrentAsTarget();
}

void Gimbal::ParseCAN(const uint32_t id, uint8_t data[8]) {
    if (id == 0x205) // yaw
        m_yaw.ParseCAN(data);
    m_pitch.ParseCAN(data);
    m_shoot.ParseCAN(data);

    measure.pitch.absolute = m_pitch.measure.angle; // pitch绝对角度
    measure.pitch.relative.degree = norm_angle(measure.pitch.absolute.degree - PITCH_MID); // pitch相对角度

    measure.yaw.absolute = m_yaw.measure.angle; // yaw绝对角度
    measure.yaw.relative.degree = norm_angle(-(measure.yaw.absolute.degree - YAW_OFFSET)); // yaw相对角度

    measure.shoot.absolute = m_shoot.measure.angle; // shoot绝对角度
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
    // if (not m_shoot.IsReady())
    //     return false;
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

    if (is_enable) {
        // 使能后，把当前位置设为目标位置
        SetCurrentAsTarget();
    } else {
        // 失能关闭摩擦轮
        SetPrepareShoot(false);
    }
}

void Gimbal::SetMode(const mode_e yaw_mode) {
    if (this->mode == yaw_mode)
        return;

    this->mode = yaw_mode;

    // 切换模式后，把当前位置设为目标位置
    SetCurrentAsTarget();
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
    ref.shoot.absolute = measure.shoot.absolute; // shoot
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

void Gimbal::SetPrepareShoot(const bool is_prepare_shoot) {
    this->is_prepare_shoot = is_prepare_shoot;
    if (is_prepare_shoot) {
        BSP_PWM_SetDuty(100);
    } else {
        BSP_PWM_SetDuty(0);
    }
}

void Gimbal::Shoot() {
    if (is_prepare_shoot) {
        const float degree_add = 360.0f / SHOOT_NUM_PER_ROUND;
        ref.shoot.absolute.degree = norm_angle(ref.shoot.absolute.degree + degree_add);
    }
}

void Gimbal::Update() {
    if (is_enable) {
        // pitch、yaw
        if (mode == ECD_MODE) {
            // pitch
            ref.pitch.absolute.degree = norm_angle(ref.pitch.relative.degree + PITCH_MID);
            ref.pitch.absolute.degree = clamp(ref.pitch.absolute.degree, PITCH_MIN, PITCH_MAX);
            // 逆解更新设置的角度以反应限位
            ref.pitch.relative.degree = norm_angle(ref.pitch.absolute.degree - PITCH_MID);
            // yaw
            ref.yaw.absolute.degree = norm_angle(-ref.yaw.relative.degree + YAW_OFFSET);
        } else if (mode == IMU_MODE) {
            // 此处加上 measure，抵消掉电机PID计算中的 -measure，最终误差变成 ref - imu.yaw
            // pitch
            ref.pitch.absolute.degree = norm_angle(
                ref.pitch.imu_mode.degree - imu.pitch + measure.pitch.absolute.degree);
            ref.pitch.absolute.degree = clamp(ref.pitch.absolute.degree, PITCH_MIN, PITCH_MAX);
            // 逆解更新设置的角度以反应限位
            ref.pitch.imu_mode.degree = norm_angle(
                ref.pitch.absolute.degree + imu.pitch - measure.pitch.absolute.degree);
            // yaw
            ref.yaw.absolute.degree = norm_angle(-(ref.yaw.imu_mode.degree - imu.yaw - measure.yaw.absolute.degree));
        }
        m_pitch.SetAngle(ref.pitch.absolute); // pitch
        m_yaw.SetAngle(ref.yaw.absolute, yaw_speed_ff); // yaw

        // shoot
        m_shoot.SetAngle(ref.shoot.absolute);

        // 电机闭环控制计算
        m_pitch.Update();
        m_yaw.Update();
        m_shoot.Update();
    }

    // CAN发送控制报文
    sendCANCmd();
}

void Gimbal::sendCANCmd() {
    switch (can_send_cnt++ % 3) {
        case 0: {
            // pitch
            m_pitch.SendCANCmd();
        }
        break;

        case 1: {
            // yaw
            const int16_t yaw_cmd = m_yaw.GetCurrentCMD();
            uint8_t data[8];
            data[0] = yaw_cmd >> 8; // yaw，6020电机，ID：5
            data[1] = yaw_cmd;
            data[2] = 0;
            data[3] = 0;
            data[4] = 0;
            data[5] = 0;
            data[6] = 0;
            data[7] = 0;
            BSP_CAN_Transmit(0x1FF, data, 8);
        }
        break;

        case 2: {
            // shoot
            m_shoot.SendCANCmd();
        }
        break;

        default:
            break;
    }
}
