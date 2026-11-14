// 主机回归：同一输入比较重构前后的电机反馈、限幅、零点与 CAN 命令
// LAYOUT_BASELINE 用于编译 795e263 的嵌套配置接口，执行方法见 README.md
#include "motor/dm4310.hpp"
#include "motor/gm6020.hpp"
#include "motor/m2006.hpp"
#include "motor/m3508.hpp"
#include "motor/rm6623.hpp"
#include "motor/rmdl7025.hpp"
#include "motor/vesc.hpp"
#include "motor/x630.hpp"
#include "robot/shooter/shooter_17mm.hpp"
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

static double sim_time = 1;
static std::vector<Can::RxCallback> callbacks;
static CanFrame sent;

bool Can::can_send() const {
    return true;
}

void Can::send(const CanFrame& frame) const {
    sent = frame;
}

void Can::add_rx_callback(RxCallback callback) const {
    callbacks.push_back(callback);
}

void Timer::delay(float) {}

float Timer::elapsed() const {
#ifdef LAYOUT_BASELINE
    return static_cast<float>(sim_time - last_cycles_ * 1e-6);
#else
    return static_cast<float>(sim_time - last_cycles * 1e-6);
#endif
}

float Timer::restart() {
    const float dt = elapsed();
#ifdef LAYOUT_BASELINE
    last_cycles_ = static_cast<uint64_t>(sim_time * 1e6);
#else
    last_cycles = static_cast<uint64_t>(sim_time * 1e6);
#endif
    return dt;
}

static Motor::Config make_config(const uint32_t rx_id) {
#ifdef LAYOUT_BASELINE
    return {.bus = {.can = Can(1), .tx_id = 0x12, .rx_id = rx_id},
            .drive = {.max_current = 20, .max_torque = 6, .reduction = 19},
            .install = {.is_invert = true, .offset = 0.3f}};
#else
    return {.can = Can(1), .tx_id = 0x12, .rx_id = rx_id, .max_current = 20, .max_torque = 6, .reduction = 19, .is_invert = true, .offset = 0.3f};
#endif
}

static void print_motor(const Motor& motor) {
    std::printf("%d %.7g %.7g %.7g %.7g %.7g %.7g\n", motor.is_connect,
                motor.current, motor.torque, motor.speed, motor.position, motor.current_ref, motor.torque_ref);
}

static void print_frame() {
    std::printf("%x %d %d", sent.id, sent.is_extend, sent.size);
    for (const auto byte : sent.data)
        std::printf(" %02x", byte);
    std::puts("");
}

static void exercise(Motor& motor, const uint32_t rx_id) {
    motor.set_enable(true);
    CanFrame latest;
    for (const uint16_t position : {uint16_t(8100), uint16_t(20), uint16_t(4000), uint16_t(8100)}) {
        sim_time += 0.002;
        CanFrame frame{.id = rx_id, .size = 8, .data = {static_cast<uint8_t>(position >> 8), static_cast<uint8_t>(position), 0xff, 0x9c, 0xfe, 0, 36, 0}};
        if (rx_id == 0x13)
            frame.data[0] = 0x12; // DM：已使能，主 ID 为 2
        else if (rx_id == 0x14)
            frame.id |= 9u << 8; // VESC STATUS：转速、电流、占空比
        else if (rx_id == 0x252)
            frame.data[0] = 0xa1; // RMD：温度、电流、转速
        latest = frame;
        for (const auto& callback : callbacks)
            callback(frame);
        if (rx_id == 0x252) {
            frame.data[0] = 0x94;
            frame.data[6] = static_cast<uint8_t>(position);
            frame.data[7] = static_cast<uint8_t>(position >> 8);
            for (const auto& callback : callbacks)
                callback(frame);
            latest = frame;
        }
        motor.update();
        for (const float current : {-100.0f, -1.0f, 0.0f, 12.0f, 100.0f, std::numeric_limits<float>::infinity()}) {
            motor.set_current(current);
            print_motor(motor);
        }
        motor.set_torque(-100);
        print_motor(motor);
    }
    motor.set_zero();
    sim_time += 0.002;
    for (const auto& callback : callbacks)
        callback(latest);
    print_motor(motor);
    sim_time += 0.2;
    motor.update();
    print_motor(motor);
}

int main() {
    M3508 m3508(make_config(0x201));
    M2006 m2006(make_config(0x202));
    GM6020 gm6020(make_config(0x205), 0.4f);
    X630 x630(make_config(0x203), {});
    RM6623 rm6623(make_config(0x204));
    RMDL7025 rmdl(make_config(0x252));
    DM4310 dm(make_config(0x13));
    Vesc vesc(make_config(0x14));
    Motor* const motors[] = {&m3508, &m2006, &gm6020, &x630, &rm6623, &rmdl, &dm, &vesc};
    const uint32_t ids[] = {0x201, 0x202, 0x205, 0x203, 0x204, 0x252, 0x13, 0x14};
    for (int i = 0; i < 8; i++)
        exercise(*motors[i], ids[i]);
    sent = {.id = 0x200, .size = 8};
    m3508.write_can_frame(sent);
    m2006.write_can_frame(sent);
    x630.write_can_frame(sent);
    print_frame();
    for (const float speed : {-3.0f, 0.0f, 12.0f}) {
        vesc.set_enable(true);
        vesc.set_speed(speed);
        vesc.send();
        print_frame();
        vesc.set_position(speed);
        vesc.send();
        print_frame();
    }
    dm.set_enable(false);
    dm.send();
    print_frame();

    // 配置展开后，电流/力矩输出类型须保持各自路径。
    const Pid::Config pid = {.kp = 2, .max_out = 20};
    for (const auto type : {Shooter17mm::CURRENT_OUTPUT, Shooter17mm::TORQUE_OUTPUT}) {
        Shooter17mm shooter({vesc, vesc, m3508}, {.shoot_pid_config = &pid, .shoot_output = type});
        shooter.set_enable(true);
        shooter.set_bullet_speed(20);
        shooter.set_bullet_freq(3);
        for (int i = 0; i < 5; i++) {
            sim_time += 0.001;
            for (const auto& callback : callbacks)
                callback({.id = 0x201, .size = 8, .data = {0x08, 0x00, 0xff, 0x9c}});
            shooter.update();
            print_motor(m3508);
        }
    }
}
