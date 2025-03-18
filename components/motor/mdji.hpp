#pragma once

#include <cstdint>

class MDJI {
public:
    // 电机状态（人类友好参数）
    // 电流【单位：A】
    float current = 0;
    // 转子角度（减速前）【单位：角度】
    float angle = 0;
    // 转速（减速后）
    float v_rpm = 0, v_rpm_lpf = 0; // 【单位：rpm】
    float v_tps = 0, v_tps_lpf = 0; // 【单位：圈/s】
    float v_aps = 0, v_aps_lpf = 0; // 【单位：角度/s】
    // 加速度（减速后）
    float last_v_tps = 0, a_tpss = 0; // 【单位：圈/s^2】

    // 目标电流【单位：A】
    float current_ref = 0;

    // CAN报文接收频率【单位：Hz】
    float can_recv_freq = 0;

    MDJI(float max_current, uint16_t can_max_cmd, float reduction_ratio);

    void ParseCAN(const uint8_t data[8]);

    void Release();

    [[nodiscard]] int16_t GetCANCmd() const;

private:
    static constexpr float SPEED_LPF_FREQ = 20.0f; // 转速低通滤波参数【单位：Hz】

    // 电机参数
    float max_current; // 最大电流【单位；A】
    uint16_t can_max_cmd; // CAN控制报文最大值
    float reduction_ratio; // 电机减速比

    // 从CAN获得的参数
    uint16_t ecd = 0; // 转子角度（减速前）【单位：0~8191 -> 0~360°】
    int16_t v_rpm_raw = 0; // 转速（减速前）【单位：rpm】
    int16_t current_raw = 0; // 电流原始值
    uint8_t temperate = 0; // 电机温度【单位：℃】

    // 作用：计算CAN报文接收频率、计算加速度、速度加速度滤波
    uint32_t dwt_cnt = 0;
    float dt = 0;
};
