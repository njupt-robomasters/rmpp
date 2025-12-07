#include "m2006.hpp"

M2006::M2006(const uint8_t can_port, const uint8_t motor_id) :
    can_port(can_port), motor_id(motor_id),
    control_can_id(motor_id <= 4 ? 0x200 : 0x1FF), feedback_can_id(0x200 + motor_id) {
    SetReduction(REDUCTION);
    SetKt(Kt);
}

int16_t M2006::GetCurrentCmd() const {
    int16_t current_cmd;
    if (!is_invert) {
        current_cmd = (int16_t)(current.ref.toFloat(A) / MAX_CURRENT.toFloat(A) * (float)MAX_CURRENT_CMD);
    } else {
        current_cmd = (int16_t)(-current.ref.toFloat(A) / MAX_CURRENT.toFloat(A) * (float)MAX_CURRENT_CMD);
    }
    return current_cmd;
}

void M2006::callback(const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    if (port != can_port) return;
    if (id != feedback_can_id) return;
    if (dlc != 8) return;

    // 解析CAN报文
    const auto angle_u16 = (uint16_t)((data[0] << 8) | data[1]);
    const auto speed_i16 = (int16_t)((data[2] << 8) | data[3]);
    const auto current_i16 = (int16_t)((data[4] << 8) | data[5]);

    // 单位标准化
    current.raw = (float)current_i16 / (float)MAX_CURRENT_CMD * MAX_CURRENT;
    speed.raw = (float)speed_i16 / reduction * rpm;
    angle.raw = (float)angle_u16 / 8192.0f * rev;

    // 调用父类公共回调函数
    Motor::callback();
}