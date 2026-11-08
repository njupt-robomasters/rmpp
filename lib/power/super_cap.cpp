#include "super_cap.hpp"
#include "math/filter.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

SuperCap::SuperCap(const Config& config)
    : config_(config), rls_{.p = {{config.rls.p_init, 0}, {0, config.rls.p_init}}} {
    config.can.add_rx_callback([this](const CanFrame& frame) { can_callback(frame); });
}

void SuperCap::set_mode(const Mode mode) {
    mode_ = mode;
}

void SuperCap::set_battery_limit(const float battery_limit) {
    battery_limit_ = battery_limit;
}

void SuperCap::set_buffer_energy(const float buffer_energy) {
    buffer_energy_ = buffer_energy;
}

void SuperCap::set_estimate(const float estimate) {
    power.estimate = estimate;
}

void SuperCap::update() {
    const float dt = update_timer_.restart();

    // 断联检测
    if (callback_timer_.elapsed() > config_.timeout)
        is_connect = false;

    // 电池侧功率上限：缓冲能量 buffer.cutoff 为零、60 J 为 buffer.gain_max 倍裁判上限，其间线性
    // 测量偏小时缓冲能量下降、上限随之降低，稳定在上限等于真实裁判上限处
    const float buffer_ratio = std::clamp((buffer_energy_ - config_.buffer.cutoff) / (BUFFER_FULL - config_.buffer.cutoff), 0.0f, 1.0f);
    power.battery_limit = battery_limit_ * config_.buffer.gain_max * buffer_ratio;

    // 偏差：在线且非 OFF 时用实测修正估计
    if (is_connect && mode_ != OFF)
        power.error = lowpass(power.error, power.chassis - power.estimate, config_.error_fc, dt);
    else
        power.error = 0;

    // BOOST 补偿：电容电压在截止与满额之间线性
    if (is_connect && mode_ == BOOST) {
        const float cap_ratio = std::clamp((voltage.cap - config_.boost.voltage_cutoff) / (config_.boost.voltage_full - config_.boost.voltage_cutoff), 0.0f, 1.0f);
        power.boost_max = config_.boost.max_current * voltage.cap * cap_ratio;
    } else {
        power.boost_max = 0;
    }

    // 底盘功率上限；裁判不限功率时缓冲能量也异常，不限制
    if (battery_limit_ == UNLIMITED)
        power.chassis_limit = std::numeric_limits<float>::infinity();
    else
        power.chassis_limit = power.battery_limit + power.boost_max - power.error;
}

void SuperCap::send() const {
    // data[0].bit0 使能 DCDC；data[1..2] 电池侧目标功率，0.01 W/LSB，大端；不限功率时发协议最大值
    const bool is_dcdc_enable = is_connect && (mode_ == CHARGE || mode_ == BOOST);
    const float battery_power = battery_limit_ == UNLIMITED ? UNLIMITED : power.battery_limit;
    const auto command = static_cast<int16_t>(std::clamp(battery_power, 0.0f, 327.67f) * 100.0f);
    config_.can.send({
        .id = config_.tx_id,
        .size = 8,
        .data = {is_dcdc_enable, static_cast<uint8_t>(command >> 8), static_cast<uint8_t>(command)},
    });
}

void SuperCap::can_callback(const CanFrame& frame) {
    // 检查 id 和 size
    if (frame.id != config_.rx_id || frame.size != 8)
        return;

    // V_BAT、I_OUT、V_CAP、I_CAP，1 mV/LSB、1 mA/LSB，大端；电流按应答间隔低通
    const auto* data = frame.data;
    const float dt = callback_timer_.restart();
    voltage.chassis = static_cast<int16_t>((data[0] << 8) | data[1]) * 1.0e-3f;
    current.chassis = lowpass(current.chassis, static_cast<int16_t>((data[2] << 8) | data[3]) * 1.0e-3f, config_.current_fc, dt);
    voltage.cap = static_cast<int16_t>((data[4] << 8) | data[5]) * 1.0e-3f;
    const float current_cap = static_cast<int16_t>((data[6] << 8) | data[7]) * 1.0e-3f;
    current.cap = lowpass(current.cap, current_cap, config_.current_fc, dt);

    // 功率：battery = chassis + cap
    power.chassis = voltage.chassis * current.chassis;
    power.cap = voltage.cap * current.cap;
    power.battery = power.chassis + power.cap;

    // 容量辨识用未低通电流，需要本帧前的在线状态
    update_capacitance(current_cap, dt);

    is_connect = true;
}

void SuperCap::update_capacitance(const float current_cap, const float dt) {
    // 梯形积分窗口内电荷
    rls_.charge += (current_cap + rls_.last_current) / 2 * dt;
    rls_.time += dt;
    rls_.last_current = current_cap;

    // 首帧、应答中断或窗口超时时丢弃窗口；有效且电荷未达阈值时继续累计
    const bool is_valid = is_connect && dt <= config_.timeout && rls_.time <= config_.rls.max_window;
    if (is_valid && std::fabs(rls_.charge) < config_.rls.min_charge)
        return;

    // RLS 一步：ΔV = ΔQ·1/C + ΔI·ESR，差分消去未知的初始电荷
    if (is_valid) {
        const float phi[2] = {rls_.charge, current_cap - rls_.current};
        const float p_phi[2] = {
            rls_.p[0][0] * phi[0] + rls_.p[0][1] * phi[1],
            rls_.p[1][0] * phi[0] + rls_.p[1][1] * phi[1],
        };

        // 协方差迹超过初值时停止遗忘，防止 ESR 方向激励不足时协方差发散
        const float lambda = rls_.p[0][0] + rls_.p[1][1] < 2 * config_.rls.p_init ? config_.rls.lambda : 1;
        const float denominator = lambda + phi[0] * p_phi[0] + phi[1] * p_phi[1];
        const float gain[2] = {p_phi[0] / denominator, p_phi[1] / denominator};
        const float error = voltage.cap - rls_.voltage - (phi[0] * rls_.theta[0] + phi[1] * rls_.theta[1]);
        for (int i = 0; i < 2; i++) {
            rls_.theta[i] += gain[i] * error;
            for (int j = 0; j < 2; j++)
                rls_.p[i][j] = (rls_.p[i][j] - gain[i] * p_phi[j]) / lambda;
        }

        // 1/C 非正视为无效
        if (rls_.theta[0] > 0) {
            capacitance = 1 / rls_.theta[0];
            resistance = rls_.theta[1];
        } else {
            capacitance = 0;
            resistance = 0;
        }
    }

    // 以当前点开始下一窗口
    rls_.voltage = voltage.cap;
    rls_.current = current_cap;
    rls_.charge = 0;
    rls_.time = 0;
}
