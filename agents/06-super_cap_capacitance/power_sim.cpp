// 主机闭环仿真：SuperCap 虚拟缓冲能量功率控制
// 底盘功率需求固定为 demand，底盘按 chassis_ref / model_scale 限幅（model_scale < 1 模拟电机模型低估）
// 超电 DCDC：使能时电池侧稳在 battery_ref，电容 8 V 止放、满电停充；裁判 10 Hz 结算缓冲能量
// 构建与运行见同目录 README.md
#include "power/super_cap.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>

static double sim_time = 0;
static Can::RxCallback rx_callback;

bool Can::can_send() const {
    return true;
}

void Can::send(const CanFrame&) const {}

void Can::add_rx_callback(RxCallback callback) const {
    rx_callback = callback;
}

void Timer::delay(float) {}

float Timer::elapsed() const {
    return static_cast<float>(sim_time - last_cycles * 1e-6);
}

float Timer::restart() {
    const float dt = static_cast<float>(sim_time - last_cycles * 1e-6);
    last_cycles = static_cast<uint64_t>(sim_time * 1e6);
    return dt;
}

static void put(uint8_t* data, const float value) {
    const auto raw = static_cast<int16_t>(std::lround(value * 1000));
    data[0] = static_cast<uint8_t>(raw >> 8);
    data[1] = static_cast<uint8_t>(raw);
}

struct Phase {
    double until;
    SuperCap::Mode mode;
    float demand;
};

static void run(const char* name, const float referee_limit, const float model_scale, const float cap_voltage, const Phase* phases, const int count) {
    sim_time = 0;
    SuperCap super_cap({.can = Can(1)});
    const float capacitance = 6, cap_full = 24, cap_empty = 8, battery_voltage = 24;
    float cap_charge = cap_voltage * capacitance;
    float referee_energy = 60, referee_power_sum = 0, last_ref = 0, max_jump = 0, min_referee = 60;
    const double dt = 0.001;
    std::printf("%s：裁判上限 %.0f W，模型比例 %.2f，初始电容 %.1f V\n", name, referee_limit, model_scale, cap_voltage);
    std::printf("   t    mode  ref_chassis  chassis  cap    battery  local  referee  V_cap\n");

    int k = 0;
    for (int p = 0; p < count; p++) {
        super_cap.set_mode(phases[p].mode);
        while (sim_time < phases[p].until) {
            sim_time += dt;
            k++;

            // 底盘：按估计模型限到 chassis_ref，真实功率 = 估计 / model_scale
            super_cap.set_referee_limit(referee_limit);
            super_cap.set_referee_energy(referee_energy);
            super_cap.update();
            if (k > 1)
                max_jump = std::max(max_jump, std::fabs(super_cap.power_chassis_ref - last_ref));
            last_ref = super_cap.power_chassis_ref;
            const float chassis = std::min(phases[p].demand, super_cap.power_chassis_ref / model_scale);

            // 超电：电池侧稳在 battery_ref，电容补差；放空或充满时由电池承担
            const bool is_dcdc = phases[p].mode != SuperCap::OFF;
            const float voltage = cap_charge / capacitance;
            float cap_power = 0;
            if (is_dcdc) {
                cap_power = std::clamp(super_cap.power_battery_ref, 0.0f, 327.67f) - chassis;
                if ((cap_power < 0 && voltage <= cap_empty) || (cap_power > 0 && voltage >= cap_full))
                    cap_power = 0;
            }
            cap_charge += cap_power / voltage * static_cast<float>(dt);
            const float battery = chassis + cap_power;

            // 裁判 10 Hz 结算
            referee_power_sum += battery;
            if (k % 100 == 0) {
                const float referee_power = referee_power_sum / 100;
                referee_power_sum = 0;
                referee_energy = std::clamp(referee_energy + (referee_limit - referee_power) * 0.1f, 0.0f, 60.0f);
                min_referee = std::min(min_referee, referee_energy);
            }

            // 100 Hz 应答
            if (k % 10 == 0) {
                CanFrame frame{.id = 0x051, .size = 8};
                put(&frame.data[0], battery_voltage);
                put(&frame.data[2], chassis / battery_voltage);
                put(&frame.data[4], voltage);
                put(&frame.data[6], cap_power / voltage);
                rx_callback(frame);
            }

            if (k % 1000 == 0)
                std::printf("  %4.0f  %-6s %7.1f  %7.1f  %6.1f  %7.1f  %5.1f  %5.1f  %5.2f\n", sim_time, phases[p].mode == SuperCap::OFF ? "OFF" : phases[p].mode == SuperCap::CHARGE ? "CHARGE"
                                                                                                                                                                                      : "BOOST",
                            super_cap.power_chassis_ref, super_cap.power_chassis, super_cap.power_cap, super_cap.power_battery, super_cap.energy_local, referee_energy, voltage);
        }
    }
    std::printf("  裁判缓冲能量最低 %.1f J，单步 chassis_ref 最大跳变 %.2f W\n\n", min_referee, max_jump);
}

int main() {
    // 模式切换：OFF → CHARGE → BOOST → CHARGE → OFF，需求 200 W
    const Phase cycle[] = {{3, SuperCap::OFF, 200}, {8, SuperCap::CHARGE, 200}, {13, SuperCap::BOOST, 200}, {18, SuperCap::CHARGE, 200}, {21, SuperCap::OFF, 200}};
    run("模型准确", 60, 1, 20, cycle, 5);
    run("模型低估 20%", 60, 0.8f, 20, cycle, 5);

    // CHARGE 低需求充电、电容放空后 BOOST
    const Phase empty[] = {{4, SuperCap::CHARGE, 20}, {10, SuperCap::BOOST, 300}};
    run("电容将空", 60, 1, 10.5f, empty, 2);
}
