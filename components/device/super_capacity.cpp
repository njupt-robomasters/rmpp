#include "super_capacity.hpp"
#include "bsp_can.h"
#include "utils.hpp"

void SuperCapacity::ParseCAN(const uint32_t id, const uint8_t data[8]) {
    if (id == 0x211) {
        input_voltage = static_cast<float>(data[0] | data[1] << 8) / 100.0f;
        capacity_voltage = static_cast<float>(data[2] | data[3] << 8) / 100.0f;
        input_current = static_cast<float>(data[4] | data[5] << 8) / 100.0f;
        power_limit = static_cast<float>(data[6] | data[7] << 8) / 100.0f;

        // 计算更多数据
        input_power = input_voltage * input_current;
        percentage = capacity_voltage * capacity_voltage / (25 * 25);
        percentage = clamp(percentage, 0, 1);
    }
}

void SuperCapacity::SendPowerLimit(const float power) {
    const auto power_limit = static_cast<uint16_t>(power * 100.0f);
    uint8_t data[8];
    data[0] = power_limit;
    data[1] = power_limit >> 8;
    BSP_CAN_Transmit(0x210, data, 2);
}

bool SuperCapacity::isLowEnergy() const {
    return capacity_voltage < 10.0f;
}
