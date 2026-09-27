#include "chassis_omni.hpp"
#include "math/math.hpp"

static const float SQRT2 = std::sqrt(2.0f);
static const float SQRT2DIV2 = SQRT2 / 2.0f;

ChassisOmni::ChassisOmni(const Config& config, const Motors& motor)
    : Chassis(config), motor(motor) {}

void ChassisOmni::set_enable(const bool is_enable) {
    if (this->is_enable == is_enable)
        return;
    this->is_enable = is_enable;

    set_mode(DETACH_MODE); // 失能/使能后默认分离模式，防止车突然转动伤人

    motor.w1.set_enable(is_enable);
    motor.w2.set_enable(is_enable);
    motor.w3.set_enable(is_enable);
    motor.w4.set_enable(is_enable);
}

void ChassisOmni::update_ui(Ui& ui) {
    ui.robot.w1 = motor.w1.is_connected ? Ui::GREEN : Ui::PINK;
    ui.robot.w2 = motor.w2.is_connected ? Ui::GREEN : Ui::PINK;
    ui.robot.w3 = motor.w3.is_connected ? Ui::GREEN : Ui::PINK;
    ui.robot.w4 = motor.w4.is_connected ? Ui::GREEN : Ui::PINK;
}

void ChassisOmni::update() {
    Chassis::update();

    // 更新电机
    motor.w1.update_control();
    motor.w2.update_control();
    motor.w3.update_control();
    motor.w4.update_control();
}

void ChassisOmni::forward() {
    // 读取电机转速
    const float v1 = motor.w1.speed.measure * config.wheel_radius;
    const float v2 = motor.w2.speed.measure * config.wheel_radius;
    const float v3 = motor.w3.speed.measure * config.wheel_radius;
    const float v4 = motor.w4.speed.measure * config.wheel_radius;

    // 速度正解
    float vx{}, vy{}, vz{};
    // 全向轮速度正解 begin
    vx = SQRT2 * (-v1 - v2 + v3 + v4) / 4.0f;
    vy = SQRT2 * (+v1 - v2 - v3 + v4) / 4.0f;
    vz = (+v1 + v2 + v3 + v4) / 4.0f;
    // 全向轮速度正解 end
    std::tie(this->vx.measure, this->vy.measure) = rotate(vx, vy, -gimbal_yaw_);
    this->vz.measure = vz;
    wr.measure = vz / config.chassis_radius;

    // 读取电机扭矩
    const float f1 = motor.w1.torque.measure / config.wheel_radius;
    const float f2 = motor.w2.torque.measure / config.wheel_radius;
    const float f3 = motor.w3.torque.measure / config.wheel_radius;
    const float f4 = motor.w4.torque.measure / config.wheel_radius;

    // 力学正解
    float fx{}, fy{}, fz{};
    // 全向轮力学正解 begin
    fx = SQRT2DIV2 * (-f1 - f2 + f3 + f4);
    fy = SQRT2DIV2 * (+f1 - f2 - f3 + f4);
    fz = f1 + f2 + f3 + f4;
    // 全向轮力学正解 end
    std::tie(this->fx.measure, this->fy.measure) = rotate(fx, fy, -gimbal_yaw_);
    this->fz.measure = fz;
}

void ChassisOmni::backward() {
    // 速度逆解
    auto [vx, vy] = rotate(this->vx.ref, this->vy.ref, gimbal_yaw_);
    auto vz = this->vz.ref;
    float v1{}, v2{}, v3{}, v4{};
    // 全向轮速度逆解 begin
    v1 = SQRT2DIV2 * (-vx + vy) + vz;
    v2 = SQRT2DIV2 * (-vx - vy) + vz;
    v3 = SQRT2DIV2 * (+vx - vy) + vz;
    v4 = SQRT2DIV2 * (+vx + vy) + vz;
    // 全向轮速度逆解 end

    // 设置电机转速
    // 目前使用底盘速度PID，不使用轮速PID，这里设置不会生效，仅用于调试
    motor.w1.set_speed(v1 / config.wheel_radius);
    motor.w2.set_speed(v2 / config.wheel_radius);
    motor.w3.set_speed(v3 / config.wheel_radius);
    motor.w4.set_speed(v4 / config.wheel_radius);

    // 力学逆解
    auto [fx, fy] = rotate(this->fx.ref, this->fy.ref, gimbal_yaw_);
    auto fz = this->fz.ref;
    float f1{}, f2{}, f3{}, f4{};
    // 全向轮力学逆解 begin
    f1 = (SQRT2 * (-fx + fy) + fz) / 4.0f;
    f2 = (SQRT2 * (-fx - fy) + fz) / 4.0f;
    f3 = (SQRT2 * (+fx - fy) + fz) / 4.0f;
    f4 = (SQRT2 * (+fx + fy) + fz) / 4.0f;
    // 全向轮力学逆解 end

    // 设置电机扭矩
    motor.w1.set_torque(f1 * config.wheel_radius);
    motor.w2.set_torque(f2 * config.wheel_radius);
    motor.w3.set_torque(f3 * config.wheel_radius);
    motor.w4.set_torque(f4 * config.wheel_radius);
}