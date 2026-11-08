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

void SuperCap::set_referee_limit(const float referee_limit) {
    referee_limit_ = referee_limit;
}

void SuperCap::set_referee_energy(const float referee_energy) {
    energy.referee = referee_energy;
}

void SuperCap::update() {
    const float dt = update_timer_.restart();

    // 断联检测
    if (callback_timer_.elapsed() > config_.timeout)
        is_connect = false;

    // BOOST 补偿：电容电压在截止与满额之间线性
    if (is_connect && mode_ == BOOST) {
        const float cap_ratio = std::clamp((voltage.cap - config_.boost.voltage_cutoff) / (config_.boost.voltage_full - config_.boost.voltage_cutoff), 0.0f, 1.0f);
        power.boost_max = config_.boost.max_current * voltage.cap * cap_ratio;
    } else {
        power.boost_max = 0;
    }

    // 裁判不限功率时缓冲能量也异常，不限制
    if (referee_limit_ == UNLIMITED) {
        power.battery_ref = power.chassis_ref = std::numeric_limits<float>::infinity();
        energy.local = energy.referee;
        return;
    }

    // 电池侧目标：裁判上限按裁判缓冲能量折算；实测偏小时裁判缓冲能量下降、目标随之降低
    power.battery_ref = referee_limit_ * buffer_gain(energy.referee);

    // 底盘供给与虚拟缓冲能量
    // DCDC 使能：供给为电池侧目标加电容补偿；底盘超出供给（电容额外放电或电池超用）时扣除，低于时增加
    // 否则：供给为裁判上限，虚拟缓冲能量跟随裁判值，切入 DCDC 时不跳变
    float supply = referee_limit_;
    if (is_dcdc_enable()) {
        supply = power.battery_ref + power.boost_max;
        energy.local = std::clamp(energy.local + (supply - power.chassis) * dt, 0.0f, BUFFER_FULL);
    } else {
        energy.local = energy.referee;
    }

    // 底盘功率目标；DCDC 未使能时等于电池侧目标
    power.chassis_ref = supply * buffer_gain(energy.local);
}

void SuperCap::send() const {
    // data[0].bit0 使能 DCDC；data[1..2] 电池侧目标功率，0.01 W/LSB，大端；不限功率时限幅到协议最大值
    const auto command = static_cast<int16_t>(std::clamp(power.battery_ref, 0.0f, 327.67f) * 100.0f);
    config_.can.send({
        .id = config_.tx_id,
        .size = 8,
        .data = {is_dcdc_enable(), static_cast<uint8_t>(command >> 8), static_cast<uint8_t>(command)},
    });
}

bool SuperCap::is_dcdc_enable() const {
    return is_connect && (mode_ == CHARGE || mode_ == BOOST);
}

float SuperCap::buffer_gain(const float energy) const {
    return config_.buffer.gain_max * std::clamp((energy - config_.buffer.cutoff) / (BUFFER_FULL - config_.buffer.cutoff), 0.0f, 1.0f);
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
