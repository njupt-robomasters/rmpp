#include "cyber_dog_motor.hpp"

CyberDogMotor::CyberDogMotor(const Config& config) : Motor(config) {
    // 设置电机默认参数
    if (this->config.reduction == 0)
        this->config.reduction = REDUCTION;
    if (this->config.kt == 0)
        this->config.kt = KT;
    if (this->config.r == 0)
        this->config.r = R;

    // 注册CAN回调
    rmpp::board.can(config.can_port).add_rx_callback([this](const rmpp::CanFrame& frame) {
        this->callback(this->config.can_port, frame.id, frame.data.data(), frame.size);
    });
}

void CyberDogMotor::send_can_cmd() {
    send_cnt_++;
    if (is_connect && is_enable) {
        if (send_cnt_ % 100 == 0) { // 每100次调用重新发送使能
            send_enable();
        } else {
            send_mit();
        }
    } else {
        send_disable();
    }
}

void CyberDogMotor::callback(const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    // 端口、ID、长度校验
    if (port != config.can_port)
        return;
    if (id != config.master_id)
        return;
    if (dlc != 8)
        return;

    // slave_id校验
    if (data[0] != config.slave_id)
        return;

    // 解析CAN报文
    const uint16_t angle_u16 = (data[1] << 8) | data[2];
    const uint16_t speed_u12 = (data[3] << 4) | (data[4] >> 4);
    const uint16_t current_u12 = ((data[4] & 0x0F) << 8) | data[5];

    // 单位标准化
    Raw raw = {
        .current = uint_to_float(current_u12, -I_MAX, I_MAX, 12),
        .torque = raw.current * config.kt,
        .speed = uint_to_float(speed_u12, -V_MAX, V_MAX, 12),
        .angle = uint_to_float(angle_u16, -P_MAX, P_MAX, 16)};

    // 调用父类公共回调函数
    Motor::callback(raw);
}

float CyberDogMotor::uint_to_float(const int x_int, const float x_min, const float x_max, const int bits) {
    const float span = x_max - x_min;
    const float offset = x_min;
    return (float)x_int * span / (float)((1 << bits) - 1) + offset;
}

int CyberDogMotor::float_to_uint(const float x, const float x_min, const float x_max, const int bits) {
    const float span = x_max - x_min;
    const float offset = x_min;
    return (int)((x - offset) * (float)((1 << bits) - 1) / span);
}

void CyberDogMotor::send_enable() const {
    std::array<uint8_t, 8> data = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
    rmpp::board.can(config.can_port).send({.id = config.slave_id, .data = data, .size = 8});
}

void CyberDogMotor::send_disable() const {
    std::array<uint8_t, 8> data = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
    rmpp::board.can(config.can_port).send({.id = config.slave_id, .data = data, .size = 8});
}

void CyberDogMotor::send_zero() const {
    std::array<uint8_t, 8> data = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};
    rmpp::board.can(config.can_port).send({.id = config.slave_id, .data = data, .size = 8});
}

void CyberDogMotor::send_mit() const {
    uint16_t angle_u16 = float_to_uint(0, -P_MAX, P_MAX, 16); // 位置
    uint16_t speed_u12 = float_to_uint(0, -V_MAX, V_MAX, 12); // 速度
    uint16_t kp_u12 = float_to_uint(0, 0, KP_MAX, 12);        // 位置比例系数
    uint16_t kd_u12 = float_to_uint(0, 0, KD_MAX, 12);        // 位置微分系数
    uint16_t current_u12;                                     // 电流
    if (!config.is_invert) {
        current_u12 = float_to_uint(current.ref, -I_MAX, I_MAX, 12);
    } else {
        current_u12 = float_to_uint(-current.ref, -I_MAX, I_MAX, 12);
    }

    std::array<uint8_t, 8> data{};
    data[0] = angle_u16 >> 8;
    data[1] = angle_u16;
    data[2] = speed_u12 >> 4;
    data[3] = ((speed_u12 & 0x0F) << 4) | (kp_u12 >> 8);
    data[4] = kp_u12;
    data[5] = kd_u12 >> 4;
    data[6] = ((kd_u12 & 0x0F) << 4) | (current_u12 >> 8);
    data[7] = current_u12;

    rmpp::board.can(config.can_port).send({.id = config.slave_id, .data = data, .size = 8});
}