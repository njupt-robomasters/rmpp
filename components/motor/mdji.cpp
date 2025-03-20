#include "mdji.hpp"
#include "bsp_dwt.h"
#include "utils.hpp"

MDJI::MDJI(const float current_max, const uint16_t can_cmd_max, const float reduction_ratio) {
    this->current_max = current_max;
    this->can_cmd_max = can_cmd_max;
    this->reduction_ratio = reduction_ratio;
}

void MDJI::ParseCAN(const uint8_t data[8]) {
    dt = BSP_DWT_GetDeltaT(&dwt_cnt);
    can_recv_freq = 1 / dt;

    // 解析CAN报文
    ecd = static_cast<uint16_t>(data[0] << 8 | data[1]);
    v_rpm_measure_raw = static_cast<int16_t>(data[2] << 8 | data[3]);
    current_measure_raw = static_cast<int16_t>(data[4] << 8 | data[5]);
    temperate = data[6];

    // 转换为人类友好参数
    // 电流
    current_measure = static_cast<float>(current_measure_raw) / static_cast<float>(can_cmd_max) * current_max;

    // 转子角度（减速前）【单位：角度】
    angle_measure = static_cast<float>(ecd) / 8192.0f * 360.0f;

    // 转速（减速后）
    v_measure_rpm = static_cast<float>(v_rpm_measure_raw) / reduction_ratio; // 【单位：rpm】
    v_measure_tps = v_measure_rpm / 60.0f; // 【单位：圈/s】
    v_measure_aps = v_measure_tps * 360.0f; // 【单位：角度/s】

    // 转速低通滤波
    const float alpha = solve_alpha(SPEED_LPF_FREQ, 1 / dt);
    lowpass_filter(v_measure_lpf_rpm, v_measure_rpm, alpha);
    lowpass_filter(v_measure_lpf_tps, v_measure_tps, alpha);
    lowpass_filter(v_measure_lpf_aps, v_measure_aps, alpha);

    // 加速度（减速后）【单位：圈/s^2】
    a_measure_tpss = (v_measure_lpf_tps - v_measure_tps_last) / dt;
    v_measure_tps_last = v_measure_lpf_tps;

    is_ready = true;
}

int16_t MDJI::GetCurrentCMD() const {
    const auto cmd = static_cast<int16_t>(current_ref / current_max * static_cast<float>(can_cmd_max));
    return cmd;
}
