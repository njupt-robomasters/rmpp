#include "super_cap.hpp"
#include "math/filter.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

SuperCap::SuperCap(const Config& config)
    : can(config.can),
      tx_id(config.tx_id),
      rx_id(config.rx_id),
      timeout(config.timeout),
      current_fc(config.current_fc),
      buffer_cutoff(config.buffer_cutoff),
      buffer_gain_max(config.buffer_gain_max),
      boost_voltage_cutoff(config.boost_voltage_cutoff),
      boost_voltage_full(config.boost_voltage_full),
      boost_max_current(config.boost_max_current),
      rls{.min_charge = config.rls.min_charge, .max_window = config.rls.max_window, .lambda = config.rls.lambda, .p_init = config.rls.p_init, .p = {{config.rls.p_init, 0}, {0, config.rls.p_init}}} {
    config.can.add_rx_callback([this](const CanFrame& frame) { can_callback(frame); });
}

void SuperCap::set_mode(const Mode mode) {
    this->mode = mode;
}

void SuperCap::set_referee_limit(const float referee_limit) {
    this->referee_limit = referee_limit;
}

void SuperCap::set_referee_energy(const float referee_energy) {
    energy_referee = referee_energy;
}

void SuperCap::update() {
    const float dt = std::min(update_timer.restart(), DT_MAX);

    // 断联检测
    if (callback_timer.elapsed() > timeout)
        is_connect = false;

    // BOOST 补偿：电容电压在截止与满额之间线性
    if (is_connect && mode == BOOST) {
        const float cap_ratio = std::clamp((voltage_cap - boost_voltage_cutoff) / (boost_voltage_full - boost_voltage_cutoff), 0.0f, 1.0f);
        power_boost_max = boost_max_current * voltage_cap * cap_ratio;
    } else {
        power_boost_max = 0;
    }

    // 裁判不限功率时缓冲能量也异常，不限制
    if (referee_limit == UNLIMITED) {
        power_battery_ref = power_chassis_ref = std::numeric_limits<float>::infinity();
        energy_local = energy_referee;
        return;
    }

    // 电池侧目标：裁判上限按裁判缓冲能量折算；实测偏小时裁判缓冲能量下降、目标随之降低
    power_battery_ref = referee_limit * buffer_gain(energy_referee);

    // 底盘供给与虚拟缓冲能量
    // DCDC 使能：供给为电池侧目标加电容补偿；底盘超出供给（电容额外放电或电池超用）时扣除，低于时增加
    // 否则：供给为裁判上限，虚拟缓冲能量跟随裁判值，切入 DCDC 时不跳变
    float supply = referee_limit;
    if (is_dcdc_enable()) {
        supply = power_battery_ref + power_boost_max;
        energy_local = std::clamp(energy_local + (supply - power_chassis) * dt, 0.0f, BUFFER_FULL);
    } else {
        energy_local = energy_referee;
    }

    // 底盘功率目标；DCDC 未使能时等于电池侧目标
    power_chassis_ref = supply * buffer_gain(energy_local);
}

void SuperCap::send() const {
    // data[0].bit0 使能 DCDC；data[1..2] 电池侧目标功率，0.01 W/LSB，大端；不限功率时限幅到协议最大值
    const auto command = static_cast<int16_t>(std::clamp(power_battery_ref, -327.68f, 327.67f) * 100.0f);
    can.send({
        .id = tx_id,
        .size = 8,
        .data = {is_dcdc_enable(), static_cast<uint8_t>(command >> 8), static_cast<uint8_t>(command)},
    });
}

void SuperCap::can_callback(const CanFrame& frame) {
    // 检查 id 和 size
    if (frame.id != rx_id || frame.size != 8)
        return;

    // V_BAT、I_OUT、V_CAP、I_CAP，1 mV/LSB、1 mA/LSB，大端；电流按应答间隔低通
    const auto* data = frame.data;
    const float dt = callback_timer.restart();
    voltage_chassis = static_cast<int16_t>((data[0] << 8) | data[1]) * 1.0e-3f;
    current_chassis = lowpass(current_chassis, static_cast<int16_t>((data[2] << 8) | data[3]) * 1.0e-3f, current_fc, dt);
    voltage_cap = static_cast<int16_t>((data[4] << 8) | data[5]) * 1.0e-3f;
    const float raw_current_cap = static_cast<int16_t>((data[6] << 8) | data[7]) * 1.0e-3f;
    current_cap = lowpass(current_cap, raw_current_cap, current_fc, dt);

    // 功率：battery = chassis + cap
    power_chassis = voltage_chassis * current_chassis;
    power_cap = voltage_cap * current_cap;
    power_battery = power_chassis + power_cap;

    // 容量辨识用未低通电流，需要本帧前的在线状态
    update_capacitance(raw_current_cap, dt);

    is_connect = true;
}

bool SuperCap::is_dcdc_enable() const {
    return is_connect && (mode == CHARGE || mode == BOOST);
}

float SuperCap::buffer_gain(const float energy) const {
    return buffer_gain_max * std::clamp((energy - buffer_cutoff) / (BUFFER_FULL - buffer_cutoff), 0.0f, 1.0f);
}

void SuperCap::update_capacitance(const float current_cap, const float dt) {
    // 梯形积分窗口内电荷
    rls.charge += (current_cap + rls.last_current) / 2 * dt;
    rls.time += dt;
    rls.last_current = current_cap;

    // 首帧、应答中断或窗口超时时丢弃窗口；有效且电荷未达阈值时继续累计
    const bool is_valid = is_connect && dt <= timeout && rls.time <= rls.max_window;
    if (is_valid && std::fabs(rls.charge) < rls.min_charge)
        return;

    // RLS 一步：ΔV = ΔQ·1/C + ΔI·ESR，差分消去未知的初始电荷
    if (is_valid) {
        const float phi[2] = {rls.charge, current_cap - rls.current};
        const float p_phi[2] = {
            rls.p[0][0] * phi[0] + rls.p[0][1] * phi[1],
            rls.p[1][0] * phi[0] + rls.p[1][1] * phi[1],
        };

        // 协方差迹超过初值时停止遗忘，防止 ESR 方向激励不足时协方差发散
        const float lambda = rls.p[0][0] + rls.p[1][1] < 2 * rls.p_init ? rls.lambda : 1;
        const float denominator = lambda + phi[0] * p_phi[0] + phi[1] * p_phi[1];
        const float gain[2] = {p_phi[0] / denominator, p_phi[1] / denominator};
        const float error = voltage_cap - rls.voltage - (phi[0] * rls.theta[0] + phi[1] * rls.theta[1]);
        for (int i = 0; i < 2; i++) {
            rls.theta[i] += gain[i] * error;
            for (int j = 0; j < 2; j++)
                rls.p[i][j] = (rls.p[i][j] - gain[i] * p_phi[j]) / lambda;
        }

        // 1/C 非正视为无效
        if (rls.theta[0] > 0) {
            capacitance = 1 / rls.theta[0];
            resistance = rls.theta[1];
        } else {
            capacitance = 0;
            resistance = 0;
        }
    }

    // 以当前点开始下一窗口
    rls.voltage = voltage_cap;
    rls.current = current_cap;
    rls.charge = 0;
    rls.time = 0;
}
