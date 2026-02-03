// #include "Chassis_Mecanum.hpp"
//
// static const float sqrt2 = std::sqrt(2.0f);
// static const float sqrt2div2 = std::sqrt(2.0f) / 2.0f;
//
// Chassis_Mecanum::Chassis_Mecanum(const config_t& config, const motor_t& motor) : Chassis(config), motor(motor) {}
//
// void Chassis_Mecanum::SetEnable(const bool is_enable) {
//     if (this->is_enable == is_enable) return;
//     this->is_enable = is_enable;
//
//     SetMode(DETACH_MODE); // 失能/使能后默认分离模式，防止车突然转动伤人
//
//     motor.w1.SetEnable(is_enable);
//     motor.w2.SetEnable(is_enable);
//     motor.w3.SetEnable(is_enable);
//     motor.w4.SetEnable(is_enable);
// }
//
// void Chassis_Mecanum::OnLoop() {
//     forward();
//     backward();
//
//     // 更新电机
//     motor.w1.OnLoop();
//     motor.w2.OnLoop();
//     motor.w3.OnLoop();
//     motor.w4.OnLoop();
// }
//
// void Chassis_Mecanum::backward() {
//     Chassis::backward();
//
//     // 速度解算
//     v1.ref = -vx.chassis.ref + vy.chassis.ref + sqrt2 * vz.ref;
//     v2.ref = -vx.chassis.ref - vy.chassis.ref + sqrt2 * vz.ref;
//     v3.ref = +vx.chassis.ref - vy.chassis.ref + sqrt2 * vz.ref;
//     v4.ref = +vx.chassis.ref + vy.chassis.ref + sqrt2 * vz.ref;
//
//     // 设置电机转速
//     // motor.w1.SetSpeed(v1.ref / config.wheel_radius);
//     // motor.w2.SetSpeed(v2.ref / config.wheel_radius);
//     // motor.w3.SetSpeed(v3.ref / config.wheel_radius);
//     // motor.w4.SetSpeed(v4.ref / config.wheel_radius);
//
//     // 力学解算
//     f1.ref = (-fx.chassis.ref + fy.chassis.ref + sqrt2div2 * fz.ref) / 4.0f;
//     f2.ref = (-fx.chassis.ref - fy.chassis.ref + sqrt2div2 * fz.ref) / 4.0f;
//     f3.ref = (+fx.chassis.ref - fy.chassis.ref + sqrt2div2 * fz.ref) / 4.0f;
//     f4.ref = (+fx.chassis.ref + fy.chassis.ref + sqrt2div2 * fz.ref) / 4.0f;
//
//     // 设置电机扭矩
//     motor.w1.SetTorque(f1.ref * config.wheel_radius);
//     motor.w2.SetTorque(f2.ref * config.wheel_radius);
//     motor.w3.SetTorque(f3.ref * config.wheel_radius);
//     motor.w4.SetTorque(f4.ref * config.wheel_radius);
// }
//
// void Chassis_Mecanum::forward() {
//     // 读取电机转速
//     v1.measure = motor.w1.speed.measure * config.wheel_radius;
//     v2.measure = motor.w2.speed.measure * config.wheel_radius;
//     v3.measure = motor.w3.speed.measure * config.wheel_radius;
//     v4.measure = motor.w4.speed.measure * config.wheel_radius;
//
//     // 速度解算
//     vx.chassis.measure = (-v1.measure - v2.measure + v3.measure + v4.measure) / 4.0f;
//     vy.chassis.measure = (+v1.measure - v2.measure - v3.measure + v4.measure) / 4.0f;
//     vz.measure = sqrt2div2 * (+v1.measure + v2.measure + v3.measure + v4.measure) / 4.0f;
//     wr.measure = vz.measure / config.chassis_radius;
//     std::tie(vx.gimbal.measure, vy.gimbal.measure) = unit::rotate(vx.chassis.measure, vy.chassis.measure, -gimbal_yaw); // 转换到云台参考系
//
//     // 读取电机扭矩
//     f1.measure = motor.w1.torque.measure / config.wheel_radius;
//     f2.measure = motor.w2.torque.measure / config.wheel_radius;
//     f3.measure = motor.w3.torque.measure / config.wheel_radius;
//     f4.measure = motor.w4.torque.measure / config.wheel_radius;
//
//     // 力学解算
//     fx.chassis.measure = -f1.measure - f2.measure + f3.measure + f4.measure;
//     fy.chassis.measure = +f1.measure - f2.measure - f3.measure + f4.measure;
//     fz.measure = sqrt2 * (f1.measure + f2.measure + f3.measure + f4.measure);
//     std::tie(fx.gimbal.measure, fy.gimbal.measure) = unit::rotate(fx.chassis.measure, fy.chassis.measure, -gimbal_yaw); // 转换到云台参考系
// }