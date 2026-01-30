#pragma once

#include "Motor.hpp"

class MF9025 : public Motor {
public:
    static constexpr float reduction = 1.0f;
    static constexpr UnitFloat<> Kt = 0.32f * Nm_A;

    MF9025(const config_t& config);

    void SendCanCmd() override;

private:
    static constexpr UnitFloat MAX_CURRENT = 16.5f * A;
    static constexpr uint16_t MAX_CURRENT_CMD = 2048;

    uint32_t send_cnt = 0; // 用于定期发送使能命令

    // CAN回调函数
    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);

    // 发送电流
    void sendCurrent(const UnitFloat<>& current) const;
};