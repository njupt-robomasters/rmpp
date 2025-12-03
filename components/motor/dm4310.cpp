#include "dm4310.hpp"

DM4310::DM4310(const uint8_t can_port, const uint32_t master_id, const uint32_t slave_id) :
    can_port(can_port), master_id(master_id), slave_id(slave_id) {
    BSP::CAN::RegisterCallback(std::bind(&DM4310::callback, this,
                                         std::placeholders::_1,
                                         std::placeholders::_2,
                                         std::placeholders::_3,
                                         std::placeholders::_4));
    SetKt(Kt);
}

void DM4310::OnLoop() {
    Motor::OnLoop();
    send_cnt++;
    if (is_enable) {
        if (send_cnt % 100 == 0) { // 每100次调用重新发送使能
            sendEnable();
        } else {
            sendMIT();
        }
    } else {
        sendDisable();
    }
}

void DM4310::callback(const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    if (port != can_port) return;
    if (id != master_id) return;
    if (dlc != 8) return;

    // 检查电机ID
    if ((data[0] & 0x0F) != (slave_id & 0x0F)) return;

    // 解析CAN报文
    err_code = err_code_e((data[0] & 0xF0) >> 4);
    const uint16_t angle_u16 = (data[1] << 8) | data[2];
    const uint16_t speed_u16 = (data[3] << 4) | (data[4] >> 4);
    const uint16_t torque_u16 = ((data[4] & 0x0F) << 8) | data[5];
    temperate_mos = data[6] * C;
    temperate_motor = data[7] * C;

    // 单位标准化
    angle.measure = uint_to_float(angle_u16, -P_MAX, P_MAX, 16) * rad;
    speed.measure = uint_to_float(speed_u16, -V_MAX, V_MAX, 12) * rad_s;
    torque.measure = uint_to_float(torque_u16, -T_MAX, T_MAX, 12) * Nm;

    // 调用父类公共回调函数
    Motor::callback();
}

float DM4310::uint_to_float(const int x_int, const float x_min, const float x_max, const int bits) {
    const float span = x_max - x_min;
    const float offset = x_min;
    return (float)x_int * span / (float)((1 << bits) - 1) + offset;
}

uint16_t DM4310::float_to_uint(const float x, const float x_min, const float x_max, const int bits) {
    const float span = x_max - x_min;
    const float offset = x_min;
    return (uint16_t)((x - offset) * (float)((1 << bits) - 1) / span);
}

void DM4310::sendEnable() const {
    uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
    BSP::CAN::TransmitStd(can_port, slave_id, data, 8);
}

void DM4310::sendDisable() const {
    uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
    BSP::CAN::TransmitStd(can_port, slave_id, data, 8);
}

void DM4310::sendMIT() const {
    uint16_t p_des = 0; // 位置给定
    uint16_t v_des = 0; // 速度给定
    uint16_t Kp = 0; // 位置比例系数
    uint16_t Kd = 0; // 位置微分系数
    uint16_t t_ff; // 转矩给定值
    if (!is_invert) {
        t_ff = float_to_uint(torque.ref.toFloat(Nm), -T_MAX, T_MAX, 12);
    } else {
        t_ff = float_to_uint(-torque.ref.toFloat(Nm), -T_MAX, T_MAX, 12);
    }

    uint8_t data[8];
    data[0] = p_des >> 8;
    data[1] = p_des;
    data[2] = v_des >> 4;
    data[3] = ((v_des & 0x0F) << 4) | (Kp >> 8);
    data[4] = Kp;
    data[5] = Kd >> 4;
    data[6] = ((Kd & 0x0F) << 4) | (t_ff >> 8);
    data[7] = t_ff;
    BSP::CAN::TransmitStd(can_port, slave_id, data, 8);
}
