#include "el05.hpp"
#include <algorithm>
#include <cmath>

EL05::EL05(const Config& config)
    : Motor(config) {
    config.bus.can.add_rx_callback([this](const CanFrame& frame) { can_callback(frame); });
}

void EL05::send() {
    if (fault != 0) {
        enable_sent_ = false;
        if (!has_fault_) {
            has_fault_ = true;
            clear_fault_timer_.restart();
        }

        // 手册第 52 页指令 5：F_CMD=FF 清错，应答为正常状态帧
        if (clear_fault_timer_.elapsed() >= 0.5f) {
            clear_fault_timer_.restart();
            send_command(0xFB);
            clear_fault_count++;
        } else {
            send_command(0xFD);
        }
        return;
    }
    has_fault_ = false;

    if (!is_enable_) {
        enable_sent_ = false;
        send_command(0xFD);
        return;
    }

    if (!enable_sent_ || enable_timer_.elapsed() >= 0.1f) {
        enable_timer_.restart();
        enable_sent_ = true;
        send_command(0xFC);
        return;
    }

    // 默认不主动上报，周期控制帧用于获取反馈，离线或未运行时发零力矩
    send_torque();
}

void EL05::can_callback(const CanFrame& frame) {
    if (frame.is_extend || frame.is_remote || frame.size != 8 ||
        frame.id != config.bus.rx_id || frame.data[0] != config.bus.tx_id)
        return;

    // 手册第 51 页应答指令 1，12 位速度/力矩，状态与温度共用 Byte6
    const auto* data = frame.data;
    mode = static_cast<Mode>((data[6] >> 6) & 0x03);
    fault = (data[6] >> 5) & 0x01;
    warning = (data[6] >> 4) & 0x01;
    raw.position = decode((data[1] << 8) | data[2], P_MAX, 16);
    raw.speed = decode((data[3] << 4) | (data[4] >> 4), V_MAX, 12);
    raw.torque = decode(((data[4] & 0x0F) << 8) | data[5], T_MAX, 12);
    raw.current = raw.torque / config.drive.kt;
    temperature = static_cast<float>(((data[6] & 0x0F) << 8) | data[7]) / 10.0f;

    Motor::can_callback();
}

void EL05::send_torque() const {
    float torque = 0;
    if (is_connect && mode == MOTOR_MODE && fault == 0 && std::isfinite(torque_ref))
        torque = config.install.is_invert ? -torque_ref : torque_ref;

    // 手册第 52 页指令 3：p16、v12、kp12、kd12、t12，MIT Kp/Kd 为零
    const uint16_t p = encode(0, P_MAX, 16);
    const uint16_t v = encode(0, V_MAX, 12);
    const uint16_t t = encode(torque, T_MAX, 12);
    config.bus.can.send({
        .id = config.bus.tx_id,
        .size = 8,
        .data = {static_cast<uint8_t>(p >> 8), static_cast<uint8_t>(p),
                 static_cast<uint8_t>(v >> 4), static_cast<uint8_t>((v & 0x0F) << 4),
                 0, 0, static_cast<uint8_t>(t >> 8), static_cast<uint8_t>(t)},
    });
}

void EL05::send_command(const uint8_t command) const {
    config.bus.can.send({
        .id = config.bus.tx_id,
        .size = 8,
        .data = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, command},
    });
}

float EL05::decode(const uint16_t value, const float maximum, const uint8_t bits) {
    return static_cast<float>(value) * (2 * maximum) / ((1U << bits) - 1U) - maximum;
}

uint16_t EL05::encode(const float value, const float maximum, const uint8_t bits) {
    return static_cast<uint16_t>((std::clamp(value, -maximum, maximum) + maximum) * ((1U << bits) - 1U) / (2 * maximum));
}
