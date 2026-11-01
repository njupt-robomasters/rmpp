#include "chassis_steer.hpp"
#include <algorithm>
#include <cmath>

// 正方形布置：x 向前、y 向左；各轮旋转切向方向（左前、左后、右后、右前）
// 轮速度 = 平移速度 + vz 沿切向，即 (vx + vz·cos t, vy + vz·sin t)
static constexpr std::array<float, 4> TANGENT = {135 * deg, -135 * deg, -45 * deg, 45 * deg};

static void update_steer_ui(const std::array<Steer, 4>& steer, RobotUi& ui) {
    ui.robot.s1 = steer[0].motor.is_connect ? RobotUi::GREEN : RobotUi::PINK;
    ui.robot.s2 = steer[1].motor.is_connect ? RobotUi::GREEN : RobotUi::PINK;
    ui.robot.s3 = steer[2].motor.is_connect ? RobotUi::GREEN : RobotUi::PINK;
    ui.robot.s4 = steer[3].motor.is_connect ? RobotUi::GREEN : RobotUi::PINK;
}

Steer::Steer(Motor& motor, const Pid::Config* pid_config)
    : motor(motor), pid_(pid_config) {}

void Steer::update() {
    motor.update();
    measure = motor.position;
}

float Steer::control(const float vx, const float vy) {
    // 离线时清空，重连后从当前舵向开始，避免突然转舵
    if (!motor.is_connect) {
        clear();
        motor.set_current(0);
        return 0;
    }
    if (!has_ref_) {
        ref = measure;
        has_ref_ = true;
    }

    // 低于最小转舵速度时保持舵向，避免零速抖动；误差超过 90° 时就近转舵并反转轮子
    float v = std::hypotf(vx, vy);
    if (v > MIN_V) {
        ref = std::atan2f(vy, vx);
        if (std::fabsf(Angle(ref - measure)) > 90 * deg) {
            ref += 180 * deg;
            v = -v;
        }
    } else {
        v = 0;
    }

    // 舵向闭环，误差取最短角度
    const Angle error = ref - measure;
    motor.set_current(pid_.calculate(error));

    // 轮速投影到当前舵向，误差达到 90° 时不驱动
    return v * std::clamp(std::cosf(error), 0.0f, 1.0f);
}

void Steer::clear() {
    pid_.clear();
    has_ref_ = false;
}

ChassisSteerSpeed::ChassisSteerSpeed(const Config& config, const Pid::Config* wheel_pid_config, const Pid::Config* steer_pid_config,
                                     const std::array<Motor*, 4>& wheel, const std::array<Motor*, 4>& steer)
    : ChassisSpeed(config, wheel_pid_config, wheel),
      steer{Steer(*steer[0], steer_pid_config), Steer(*steer[1], steer_pid_config), Steer(*steer[2], steer_pid_config), Steer(*steer[3], steer_pid_config)} {}

void ChassisSteerSpeed::update_ui(RobotUi& ui) {
    ChassisSpeed::update_ui(ui);
    update_steer_ui(steer, ui);
}

void ChassisSteerSpeed::set_motors_enable(const bool is_enable) {
    ChassisSpeed::set_motors_enable(is_enable);
    for (Steer& s : steer)
        s.motor.set_enable(is_enable);
}

void ChassisSteerSpeed::update_motors() {
    ChassisSpeed::update_motors();
    for (Steer& s : steer)
        s.update();
}

bool ChassisSteerSpeed::can_drive(const size_t i) const {
    // 舵电机离线时舵向不可控，对应轮停转
    return ChassisSpeed::can_drive(i) && steer[i].motor.is_connect;
}

void ChassisSteerSpeed::clear_pid() {
    ChassisSpeed::clear_pid();
    for (Steer& s : steer)
        s.clear();
}

void ChassisSteerSpeed::forward() {
    // 速度正解，各轮速度沿实测舵向分解后取平均
    float vx = 0, vy = 0, vz = 0;
    for (size_t i = 0; i < steer.size(); i++) {
        const float v = wheel_speed(i);
        const float s = steer[i].measure;
        vx += v * std::cosf(s);
        vy += v * std::sinf(s);
        vz += v * std::cosf(s - TANGENT[i]);
    }
    set_speed_measure(vx / 4.0f, vy / 4.0f, vz / 4.0f);
}

std::array<float, 4> ChassisSteerSpeed::backward() {
    // 速度逆解：设置各舵目标并闭环，得到投影后的轮速
    const auto [vx, vy, vz] = speed_ref();
    std::array<float, 4> v{};
    for (size_t i = 0; i < steer.size(); i++)
        v[i] = steer[i].control(vx + vz * std::cosf(TANGENT[i]), vy + vz * std::sinf(TANGENT[i]));
    return v;
}

ChassisSteerTorque::ChassisSteerTorque(const Config& config, const Pid::Config* vxyz_pid_config, const Pid::Config* steer_pid_config,
                                       const std::array<Motor*, 4>& wheel, const std::array<Motor*, 4>& steer)
    : ChassisTorque(config, vxyz_pid_config, wheel),
      steer{Steer(*steer[0], steer_pid_config), Steer(*steer[1], steer_pid_config), Steer(*steer[2], steer_pid_config), Steer(*steer[3], steer_pid_config)} {}

void ChassisSteerTorque::update_ui(RobotUi& ui) {
    ChassisTorque::update_ui(ui);
    update_steer_ui(steer, ui);
}

void ChassisSteerTorque::set_motors_enable(const bool is_enable) {
    ChassisTorque::set_motors_enable(is_enable);
    for (Steer& s : steer)
        s.motor.set_enable(is_enable);
}

void ChassisSteerTorque::update_motors() {
    ChassisTorque::update_motors();
    for (Steer& s : steer)
        s.update();
}

bool ChassisSteerTorque::can_drive(const size_t i) const {
    // 舵电机离线时舵向不可控，对应轮停转
    return ChassisTorque::can_drive(i) && steer[i].motor.is_connect;
}

void ChassisSteerTorque::clear_pid() {
    ChassisTorque::clear_pid();
    for (Steer& s : steer)
        s.clear();
}

void ChassisSteerTorque::forward() {
    // 速度与力学正解，各轮沿实测舵向分解；速度取平均，力求和
    float vx = 0, vy = 0, vz = 0;
    float fx = 0, fy = 0, fz = 0;
    for (size_t i = 0; i < steer.size(); i++) {
        const float v = wheel_speed(i);
        const float f = wheel_force(i);
        const float s = steer[i].measure;
        vx += v * std::cosf(s);
        vy += v * std::sinf(s);
        vz += v * std::cosf(s - TANGENT[i]);
        fx += f * std::cosf(s);
        fy += f * std::sinf(s);
        fz += f * std::cosf(s - TANGENT[i]);
    }
    set_speed_measure(vx / 4.0f, vy / 4.0f, vz / 4.0f);
    set_force_measure(fx, fy, fz);
}

void ChassisSteerTorque::backward() {
    const auto [vx, vy, vz] = speed_ref();
    const auto [fx, fy, fz] = force_ref();
    for (size_t i = 0; i < steer.size(); i++) {
        // 舵向由速度逆解决定
        steer[i].control(vx + vz * std::cosf(TANGENT[i]), vy + vz * std::sinf(TANGENT[i]));

        // 各轮平分期望力，投影到实测舵向；自然包含就近转舵反向和转舵未到位时的衰减
        const float fxi = (fx + fz * std::cosf(TANGENT[i])) / 4.0f;
        const float fyi = (fy + fz * std::sinf(TANGENT[i])) / 4.0f;
        const float s = steer[i].measure;
        wheel_[i]->set_torque((fxi * std::cosf(s) + fyi * std::sinf(s)) * config_.wheel_radius);
    }
}
