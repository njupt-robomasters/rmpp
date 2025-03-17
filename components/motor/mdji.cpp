#include "mdji.hpp"
#include "bsp_dwt.h"
#include "utils.hpp"

MDJI::MDJI(const float max_current, const uint16_t can_max_cmd, const float reduction_ratio) {
    this->max_current = max_current;
    this->can_max_cmd = can_max_cmd;
    this->reduction_ratio = reduction_ratio;
}

void MDJI::ParseCAN(const uint8_t data[8]) {
    dt = BSP_DWT_GetDeltaT(&dwt_cnt);
    can_recv_freq = 1 / dt;

    // 解析CAN报文
    ecd = static_cast<uint16_t>(data[0] << 8 | data[1]);
    speed_rpm_raw = static_cast<int16_t>(data[2] << 8 | data[3]);
    current_raw = static_cast<int16_t>(data[4] << 8 | data[5]);
    temperate = data[6];

    // 计算人类友好参数
    // 转子角度（减速前）【单位：角度】
    angle = static_cast<float>(ecd) / 8192.0f * 360.0f;
    // 转速（减速后）
    speed_rpm = static_cast<float>(speed_rpm_raw) / reduction_ratio; // 【单位：rpm】
    speed_tps = speed_rpm / 60.0f; // 【单位：圈/s】
    speed_aps = speed_tps * 360.0f; // 【单位：角度/s】
    // 电流
    current = static_cast<float>(current_raw) / static_cast<float>(can_max_cmd) * max_current;

    // 速度低通滤波
    const float alpha = solve_alpha(SPEED_LPF_FREQ, 1 / dt);
    lowpass_filter(speed_rpm_lpf, speed_rpm, alpha);
    lowpass_filter(speed_tps_lpf, speed_tps, alpha);
    lowpass_filter(speed_aps_lpf, speed_aps, alpha);
}

// 释放电机，关闭动力输出
void MDJI::Release() {
    current_ref = 0;
}

int16_t MDJI::GetCANCmd() const {
    const auto cmd = static_cast<int16_t>(current_ref / max_current * static_cast<float>(can_max_cmd));
    return cmd;
}
