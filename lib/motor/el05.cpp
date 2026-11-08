#include "el05.hpp"
#include <algorithm>

EL05::EL05(const Config& config)
    : Motor(config) {
    config.bus.can.add_rx_callback([this](const CanFrame& frame) { can_callback(frame); });
}

void EL05::send() {
    // 失能或有故障时发停止帧；故障不自动清除，排查后重新上电或用上位机清除
    if (!is_enable_ || fault != 0) {
        send_stop();
        return;
    }

    // 定期补发使能帧，电机回到复位模式后能重新进入运行
    if (enable_timer_.elapsed() >= 1.0f / ENABLE_FREQUENCY) {
        enable_timer_.restart();
        send_enable();
        return;
    }

    send_torque();
}

void EL05::can_callback(const CanFrame& frame) {
    // 只接收扩展帧反馈：通信类型 2，bit15~8 为电机 ID，bit7~0 为主机 ID
    if (!frame.is_extend || frame.size != 8 || ((frame.id >> 24) & 0x1f) != FEEDBACK)
        return;
    if (((frame.id >> 8) & 0xff) != config.bus.tx_id || (frame.id & 0xff) != config.bus.rx_id)
        return;

    // bit21~16 故障，bit23~22 模式
    fault = (frame.id >> 16) & 0x3f;
    mode = static_cast<Mode>((frame.id >> 22) & 0x03);

    // 大端序：角度、速度、力矩按量程线性映射，温度 0.1 ℃/LSB；角度为 ±4π 多圈，由 Motor 归一化
    const auto* data = frame.data;
    raw.position = decode((data[0] << 8) | data[1], P_MAX);
    raw.speed = decode((data[2] << 8) | data[3], V_MAX);
    raw.torque = decode((data[4] << 8) | data[5], T_MAX);
    raw.current = raw.torque / config.drive.kt;
    temperature = static_cast<float>((data[6] << 8) | data[7]) / 10.0f;

    // 调用上级回调
    Motor::can_callback();
}

void EL05::send_torque() const {
    // 力矩指令放在扩展 ID 数据区 2；离线或电机未进入运行模式时发零力矩
    float torque = 0;
    if (is_connect && mode == MOTOR_MODE)
        torque = config.install.is_invert ? -torque_ref : torque_ref;

    // 纯力矩运控：p、v 为零的量化编码固定，kp、kd 为零
    const uint16_t position = encode(0, P_MAX);
    const uint16_t speed = encode(0, V_MAX);
    config.bus.can.send({
        .id = extend_id(REMOTE_CONTROL, encode(torque, T_MAX)),
        .is_extend = true,
        .size = 8,
        .data = {static_cast<uint8_t>(position >> 8), static_cast<uint8_t>(position), static_cast<uint8_t>(speed >> 8), static_cast<uint8_t>(speed), 0, 0, 0, 0},
    });
}

void EL05::send_enable() const {
    config.bus.can.send({.id = extend_id(ENABLE, config.bus.rx_id), .is_extend = true, .size = 8});
}

void EL05::send_stop() const {
    // data[0] 为 1 时同时清除故障，这里不清除
    config.bus.can.send({.id = extend_id(STOP, config.bus.rx_id), .is_extend = true, .size = 8});
}

uint32_t EL05::extend_id(const Type type, const uint16_t data) const {
    return (static_cast<uint32_t>(type & 0x1f) << 24) | (static_cast<uint32_t>(data) << 8) | (config.bus.tx_id & 0xff);
}

float EL05::decode(const uint16_t value, const float maximum) {
    return static_cast<float>(value) * (2 * maximum) / 65535.0f - maximum;
}

uint16_t EL05::encode(const float value, const float maximum) {
    return static_cast<uint16_t>((std::clamp(value, -maximum, maximum) + maximum) * 65535.0f / (2 * maximum));
}
