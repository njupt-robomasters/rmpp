#include "mdji.hpp"
#include "bsp_dwt.h"

MDJI::MDJI(const float current_max, const uint16_t can_cmd_max, const float reduction_ratio,
           PID::param_t &pid_param) : CURRENT_MAX(current_max),
                                          CAN_CMD_MAX(can_cmd_max),
                                          REDUCTION_RATIO(reduction_ratio),
                                          pid(pid_param.SetDefaultMax(current_max)) {
}

void MDJI::ParseCAN(const uint8_t data[8]) {
    dt = BSP_DWT_GetDeltaT(&dwt_cnt);

    can_recv_freq = 1 / dt;

    // 解析CAN报文
    motor_data.ecd = static_cast<uint16_t>(data[0] << 8 | data[1]);
    motor_data.speed_rpm = static_cast<int16_t>(data[2] << 8 | data[3]);
    motor_data.current = static_cast<int16_t>(data[4] << 8 | data[5]);
    motor_data.temperate = data[6];

    // 转换为人类友好参数
    // 电流
    measure.current_raw = static_cast<float>(motor_data.current) / static_cast<float>(CAN_CMD_MAX) * CURRENT_MAX;
    // 电流低通滤波
    const float alpha1 = solve_alpha(CURRENT_LPF_FREQ, 1 / dt);
    measure.current = lowpass_filter(measure.current, measure.current_raw, alpha1);

    // 转子角度（减速前）【单位：角度】
    measure.angle.degree = static_cast<float>(motor_data.ecd) / 8192.0f * 360.0f;

    // 转速（减速后）
    measure.speed_raw.rpm = static_cast<float>(motor_data.speed_rpm) / REDUCTION_RATIO; // 【单位：rpm】
    // 转速低通滤波
    const float alpha2 = solve_alpha(SPEED_LPF_FREQ, 1 / dt);
    measure.speed.rpm = lowpass_filter(measure.speed.rpm, measure.speed_raw.rpm, alpha2);

    is_ready = true;
}

void MDJI::ResetReady() {
    is_ready = false;
}

bool MDJI::IsReady() const {
    return is_ready;
}

void MDJI::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable)
        return;

    this->is_enable = is_enable;

    pid.Clear(); // 清空PID

    if (not is_enable) {
        // 失能电流置0
        ref.current = 0;
    }
}

void MDJI::SetAngle(const Angle &angle) {
    ref.angle = angle;
}

void MDJI::SetAngle(const Angle &angle, const Speed &speed) {
    ref.angle = angle;
    ref.speed = speed;
}

void MDJI::SetSpeed(const Speed &speed) {
    ref.speed = speed;
}

int16_t MDJI::GetCurrentCMD() const {
    const auto cmd = static_cast<int16_t>(ref.current / CURRENT_MAX * static_cast<float>(CAN_CMD_MAX));
    return cmd;
}
