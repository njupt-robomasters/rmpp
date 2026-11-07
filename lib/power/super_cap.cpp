#include "super_cap.hpp"
#include <algorithm>

SuperCap::SuperCap(const Config& config)
    : config_(config) {
    config.can.add_rx_callback([this](const CanFrame& frame) { can_callback(frame); });
}

const SuperCap::Config& SuperCap::config() const {
    return config_;
}

void SuperCap::set_mode(const Mode mode) {
    mode_ = mode;
}

void SuperCap::set_battery_power(const float battery_power) {
    battery_power_ref_ = battery_power;
}

void SuperCap::update() {
    // 应答超时
    if (rx_timer_.elapsed() > config_.timeout)
        is_connect = false;

    // 电容欠压回差
    if (voltage.cap < config_.voltage_cutoff)
        is_cap_low_ = true;
    else if (voltage.cap > config_.voltage_resume)
        is_cap_low_ = false;

    // 生效模式：离线为 OFF，BOOST 电容欠压回退 CHARGE
    if (!is_connect)
        active_mode = OFF;
    else if (mode_ == BOOST && is_cap_low_)
        active_mode = CHARGE;
    else
        active_mode = mode_;
}

void SuperCap::send() const {
    // data[0].bit0 使能 DCDC；data[1..2] 电池侧目标功率，0.01 W/LSB，大端
    const bool is_dcdc_enable = active_mode == CHARGE || active_mode == BOOST;
    const auto command = static_cast<int16_t>(std::clamp(battery_power_ref_, 0.0f, 327.67f) * 100.0f);
    config_.can.send({
        .id = config_.tx_id,
        .size = 8,
        .data = {is_dcdc_enable, static_cast<uint8_t>(command >> 8), static_cast<uint8_t>(command)},
    });
}

void SuperCap::can_callback(const CanFrame& frame) {
    // 检查id和size
    if (frame.id != config_.rx_id || frame.size != 8)
        return;

    // V_BAT、I_OUT、V_CAP、I_CAP，1 mV/LSB、1 mA/LSB，大端
    const auto* data = frame.data;
    voltage.out = static_cast<int16_t>((data[0] << 8) | data[1]) * 1.0e-3f;
    current.out = static_cast<int16_t>((data[2] << 8) | data[3]) * 1.0e-3f;
    voltage.cap = static_cast<int16_t>((data[4] << 8) | data[5]) * 1.0e-3f;
    current.cap = static_cast<int16_t>((data[6] << 8) | data[7]) * 1.0e-3f;
    power.out = voltage.out * current.out;

    is_connect = true;
    rx_timer_.restart();
}
