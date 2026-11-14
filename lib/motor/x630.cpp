#include "x630.hpp"
#include <cmath>

X630::X630(const Config& config, const Mute& mute)
    : Motor(config, MAX_CURRENT, MAX_TORQUE, REDUCTION),
      enter_current(mute.enter_current),
      exit_current(mute.exit_current),
      enter_speed(mute.enter_speed) {
    config.can.add_rx_callback([this](const CanFrame& frame) { can_callback(frame); });
}

void X630::set_current(const float current) {
    Motor::set_current(current);
    apply_mute();
}

void X630::set_torque(const float torque) {
    Motor::set_torque(torque);
    apply_mute();
}

void X630::write_can_frame(CanFrame& frame) const {
    // 计算can偏移量
    uint32_t first_id;
    if (frame.id == 0x200)
        first_id = 0x201;
    else if (frame.id == 0x1ff)
        first_id = 0x205;
    else
        return;
    if (frame.size != 8 || rx_id < first_id || rx_id >= first_id + 4)
        return;
    const uint8_t offset = static_cast<uint8_t>(rx_id - first_id) * 2;

    // 计算can命令
    float current = 0;
    if (is_connect && is_enable)
        current = is_invert ? -current_ref : current_ref;
    const auto command = static_cast<int16_t>(current / MAX_CURRENT * 16384.0f);

    // 构造can报文
    frame.data[offset] = static_cast<uint8_t>(command >> 8);
    frame.data[offset + 1] = static_cast<uint8_t>(command);
}

void X630::can_callback(const CanFrame& frame) {
    // 检查id和size
    if (frame.id != rx_id || frame.size != 8)
        return;

    // 解析can报文
    const auto* data = frame.data;
    raw_position = static_cast<float>((data[0] << 8) | data[1]) / 8192.0f * rev;
    // 转速、电流为有符号 16 位，先转 int16_t 保留反转符号
    raw_speed = static_cast<int16_t>((data[2] << 8) | data[3]) * rpm;
    raw_current = static_cast<int16_t>((data[4] << 8) | data[5]) / 16384.0f * MAX_CURRENT;
    motor_temperature = static_cast<int8_t>(data[6]);
    raw_torque = raw_current / max_current * max_torque;

    // 调用上级回调
    Motor::can_callback();
}

void X630::apply_mute() {
    // 滞回：低电流且低速进入静音，电流高于退出阈值才恢复
    const float current = std::fabs(current_ref);
    if (is_mute && current > exit_current)
        is_mute = false;
    else if (!is_mute && current < enter_current && std::fabs(speed) < enter_speed)
        is_mute = true;

    if (is_mute)
        current_ref = torque_ref = 0;
}
