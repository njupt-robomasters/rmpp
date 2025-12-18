#include "mf9025.hpp"

MF9025::MF9025(const uint8_t can_port, const uint8_t motor_id) :
    can_port(can_port), motor_id(motor_id) {
    auto callback = [this](const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
        this->callback(port, id, data, dlc);
    };
    BSP::CAN::RegisterCallback(callback);
    SetKt(0.32f * Nm_A);
}

void MF9025::OnLoop() {
    Motor::OnLoop();

    if (dwt_can_send_freq.GetDT() >= 1 / can_send_freq.toFloat(Hz)) {
        dwt_can_send_freq.UpdateDT();
        send_cnt++;
        if (is_enable && is_connected) {
            if (send_cnt % 100 == 0) { // 每100次调用重新发送使能
                sendEnable();
            } else {
                sendCurrent();
            }
        } else {
            // 失能，交替发送使能和读取CAN状态
            if (send_cnt % 2 == 0) {
                sendDisable();
            } else if (send_cnt % 2 == 1) {
                sendReadState();
            }
        }
    }
}

void MF9025::callback(const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    if (port != can_port) return;
    if (id != 0x140u + motor_id) return;
    if (dlc != 8) return;

    if (data[0] != 0x9C && data[0] != 0xA1) return;

    // 解析CAN报文
    temperature_motor = data[1] * C;
    const auto current_i16 = (int16_t)(data[2] | (data[3] << 8));
    const auto speed_i16 = (int16_t)(data[4] | (data[5] << 8));
    const auto angle_u16 = (uint16_t)(data[6] | (data[7] << 8));

    // 单位标准化
    current.raw = (float)current_i16 / 2048.0f * 16.5f * A;
    torque.raw = current.raw * Kt;
    angle.raw = (float)angle_u16 / 65535.0f * rev;
    speed.raw = speed_i16 * deg_s;

    // 调用父类公共回调函数
    Motor::callback();
}

void MF9025::sendEnable() const {
    uint8_t data[8] = {0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    BSP::CAN::TransmitStd(can_port, 0x140 + motor_id, data, 8);
}

void MF9025::sendDisable() const {
    uint8_t data[8] = {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    BSP::CAN::TransmitStd(can_port, 0x140 + motor_id, data, 8);
}

void MF9025::sendReadState() const {
    uint8_t data[8] = {0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    BSP::CAN::TransmitStd(can_port, 0x140 + motor_id, data, 8);
}

void MF9025::sendCurrent() const {
    int16_t current_cmd;
    if (!is_invert) {
        current_cmd = (int16_t)(current.ref.toFloat(A) / 16.5f * 2048.0f);
    } else {
        current_cmd = (int16_t)(-current.ref.toFloat(A) / 16.5f * 2048.0f);
    }

    uint8_t data[8];
    data[0] = 0xA1; // 力矩模式
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = current_cmd;
    data[5] = current_cmd >> 8;
    data[6] = 0;
    data[7] = 0;
    BSP::CAN::TransmitStd(can_port, 0x140 + motor_id, data);
}