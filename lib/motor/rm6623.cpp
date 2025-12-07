#include "rm6623.hpp"

RM6623::RM6623(const uint8_t can_port, const uint8_t motor_id) :
    can_port(can_port), motor_id(motor_id),
    control_can_id(motor_id <= 3 ? 0x1FF : 0x2FF),
    feedback_can_id(0x205 + motor_id) {
    BSP::CAN::RegisterCallback(std::bind(&RM6623::callback, this,
                                         std::placeholders::_1,
                                         std::placeholders::_2,
                                         std::placeholders::_3,
                                         std::placeholders::_4));
}

int16_t RM6623::GetCurrentCmd() const {
    int16_t current_cmd;
    if (!is_invert) {
        current_cmd = (int16_t)(current.ref.toFloat(A) * 1000.0f);
    } else {
        current_cmd = (int16_t)((-current.ref).toFloat(A) * 1000.0f);
    }
    return current_cmd;
}

void RM6623::SendCalibrateCmd() const {
    uint8_t data[8] = {'c', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    BSP::CAN::TransmitStd(can_port, 0x3F0, data, 8); // 校准指令固定ID=0x3F0
}

void RM6623::callback(const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    // 端口、ID、长度校验
    if (port != can_port) return;
    if (id != feedback_can_id) return;
    if (dlc != 8) return;

    // 解析CAN报文
    const auto angle_u16 = (uint16_t)((data[0] << 8) | data[1]);
    const auto current_measure_i16 = (int16_t)((data[2] << 8) | data[3]);
    const auto current_ref_i16 = (int16_t)((data[4] << 8) | data[5]);

    // 单位标准化
    current.raw = (float)current_measure_i16 / 1000.0f * A;
    angle.raw = (float)angle_u16 / 8192.0f * rev;

    // 调用父类公共回调函数
    Motor::callback();
}
