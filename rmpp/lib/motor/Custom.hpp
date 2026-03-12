#pragma once

#include "Motor.hpp"

class Custom : public Motor {
public:
    Custom(const config_t& config);

    int16_t GetCanCmd() const;

private:
    // CAN接收回调
    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);
};