#include "mdji.hpp"
#include "bsp.hpp"

MDJI::MDJI(const uint8_t can_port, const uint32_t feedback_can_id,
           const UnitFloat<>& current_max, const uint16_t current_cmd_max) :
    can_port(can_port), feedback_can_id(feedback_can_id),
    current_max(current_max), current_cmd_max(current_cmd_max) {
    BSP::CAN::RegisterCallback(std::bind(&MDJI::callback, this,
                                         std::placeholders::_1,
                                         std::placeholders::_2,
                                         std::placeholders::_3,
                                         std::placeholders::_4));
}

int16_t MDJI::GetCurrentCMD() const {
    int16_t current_cmd;
    if (!is_invert) {
        current_cmd = (int16_t)((current.ref / current_max).toFloat(A) * (float)current_cmd_max);
    } else {
        current_cmd = (int16_t)((-current.ref / current_max).toFloat(A) * (float)current_cmd_max);
    }
    return current_cmd;
}

void MDJI::callback(const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    if (port != can_port) return;
    if (id != feedback_can_id) return;
    if (dlc != 8) return;

    // 解析CAN报文
    const auto angle_u16 = (uint16_t)((data[0] << 8) | data[1]);
    const auto speed_i16 = (int16_t)((data[2] << 8) | data[3]);
    const auto current_i16 = (int16_t)((data[4] << 8) | data[5]);
    temperature_motor = data[6] * C;

    // 单位标准化
    current.absolute = (float)current_i16 / (float)current_cmd_max * current_max;
    speed.absolute = (float)speed_i16 / reduction * rpm;
    angle.absolute = (float)angle_u16 / 8192.0f * rev;

    // 调用父类公共回调函数
    Motor::callback();
}
