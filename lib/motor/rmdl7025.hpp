#pragma once

#include "motor/motor.hpp"
#include <cstdint>

// 脉塔 RMD-L-7025，tx_id = 0x140 + ID，rx_id = 0x240 + ID
class RMDL7025 : public Motor {
  public:
    // 型号默认物理参数：峰值电流、峰值转矩
    static constexpr float MAX_CURRENT = 8.5f;
    static constexpr float MAX_TORQUE = 4.1f;
    static constexpr float REDUCTION = 1;

  private:
    float temperature = 0;   // 电机温度，℃
    uint32_t send_count = 0; // 奇偶决定本次发送的报文

  public:
    explicit RMDL7025(const Config& config);

    // 轮流发送转矩闭环 0xA1 和读取单圈角度 0x94，电机总应答频率上限 500 Hz，调用周期不小于 2 ms
    void send();

  private:
    void can_callback(const CanFrame& frame);
};
