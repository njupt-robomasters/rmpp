#include "el05.hpp"
#include <algorithm>
#include <cmath>

EL05::EL05(const Config& config)
    : Motor(config, MAX_CURRENT, MAX_TORQUE, REDUCTION) {
    config.can.add_rx_callback([this](const CanFrame& frame) { can_callback(frame); });
}

void EL05::set_output(const float output) {
    set_torque(output);
}

void EL05::send() {
    if (fault != 0) {
        enable_sent = false;
        if (!has_fault) {
            has_fault = true;
            clear_fault_timer.restart();
        }

        // 私有协议通信类型 4，Byte0=1 停止并清错
        if (clear_fault_timer.elapsed() >= 0.5f) {
            clear_fault_timer.restart();
            send_command(STOP, true);
            clear_fault_count++;
        } else {
            send_command(STOP);
        }
        return;
    }
    has_fault = false;

    if (!is_enable) {
        enable_sent = false;
        send_command(STOP);
        return;
    }

    if (!enable_sent || enable_timer.elapsed() >= 0.1f) {
        enable_timer.restart();
        enable_sent = true;
        send_command(ENABLE);
        return;
    }

    // 默认不主动上报，周期控制帧用于获取反馈，离线或未运行时发零力矩
    send_torque();
}

void EL05::can_callback(const CanFrame& frame) {
    if (!frame.is_extend || frame.is_remote || frame.size != 8 ||
        ((frame.id >> 24) & 0x1F) != FEEDBACK ||
        ((frame.id >> 8) & 0xFF) != config.tx_id || (frame.id & 0xFF) != config.rx_id)
        return;

    // 手册第 30 页通信类型 2：状态在扩展 ID，反馈三项均为 16 位大端
    const auto* data = frame.data;
    mode = static_cast<Mode>((frame.id >> 22) & 0x03);
    fault = (frame.id >> 16) & 0x3F;
    temperature = static_cast<float>((data[6] << 8) | data[7]) / 10.0f;

    Motor::can_callback({
        .torque = decode((data[4] << 8) | data[5], T_MAX),
        .speed = decode((data[2] << 8) | data[3], V_MAX),
        .angle = decode((data[0] << 8) | data[1], P_MAX),
    });
}

void EL05::send_torque() const {
    float torque = 0;
    if (is_connect && mode == MOTOR_MODE && fault == 0 && std::isfinite(torque_ref))
        torque = config.is_invert ? -torque_ref : torque_ref;

    // 私有协议通信类型 1：力矩编码在 ID bit23~8，数据区 p/v/kp/kd 为零
    const uint16_t p = encode(0, P_MAX);
    const uint16_t v = encode(0, V_MAX);
    const uint16_t t = encode(torque, T_MAX);
    config.can.send({
        .id = (static_cast<uint32_t>(TORQUE) << 24) | (static_cast<uint32_t>(t) << 8) | config.tx_id,
        .is_extend = true,
        .size = 8,
        .data = {static_cast<uint8_t>(p >> 8), static_cast<uint8_t>(p),
                 static_cast<uint8_t>(v >> 8), static_cast<uint8_t>(v), 0, 0, 0, 0},
    });
}

void EL05::send_command(const Command command, const bool clear_fault) const {
    config.can.send({
        .id = (static_cast<uint32_t>(command) << 24) | (config.rx_id << 8) | config.tx_id,
        .is_extend = true,
        .size = 8,
        .data = {static_cast<uint8_t>(clear_fault), 0, 0, 0, 0, 0, 0, 0},
    });
}

float EL05::decode(const uint16_t value, const float maximum) {
    return static_cast<float>(value) * (2 * maximum) / 65535.0f - maximum;
}

uint16_t EL05::encode(const float value, const float maximum) {
    return static_cast<uint16_t>((std::clamp(value, -maximum, maximum) + maximum) * 65535.0f / (2 * maximum));
}
