#include "dji_motor.hpp"
#include "math/unit.hpp"

DjiMotor::DjiMotor(const Config& config)
    : Motor(config) {}

float DjiMotor::directed_current() const {
    if (!is_connect || !is_enable)
        return 0;
    return config_.install.is_invert ? -current.ref : current.ref;
}

bool DjiMotor::write_command(CanFrame& frame, const int16_t command, const uint32_t first_rx_id, const uint8_t motor_count) const {
    if (frame.size != 8 || config_.bus.rx_id < first_rx_id || config_.bus.rx_id >= first_rx_id + motor_count)
        return false;

    const uint8_t offset = uint8_t(config_.bus.rx_id - first_rx_id) * 2;
    frame.data[offset] = command >> 8;
    frame.data[offset + 1] = command;
    return true;
}

void DjiMotor::can_callback(const uint16_t encoder, const float current, const int16_t speed) {
    raw.current = current;
    raw.torque = raw.current * config_.drive.kt;
    raw.speed = float(speed) / config_.drive.reduction * rpm;
    raw.position = float(encoder) / 8192 * rev;
    Motor::can_callback();
}
