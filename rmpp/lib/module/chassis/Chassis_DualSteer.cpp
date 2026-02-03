// #include "Chassis_DualSteer.hpp"
//
// static const float sqrt2div2 = std::sqrt(2.0f) / 2.0f;
//
// Chassis_DualSteer::Chassis_DualSteer(const config_t& config, const motor_t& motor) : Chassis(config), motor(motor) {}
//
// void Chassis_DualSteer::SetEnable(const bool is_enable) {
//     if (this->is_enable == is_enable) return; // 防止重复设置
//     this->is_enable = is_enable;
//
//     SetMode(DETACH_MODE); // 失能/使能后默认分离模式，防止车突然转动伤人
//
//     motor.w1.SetEnable(is_enable);
//     motor.w2.SetEnable(is_enable);
//     motor.s1.SetEnable(is_enable);
//     motor.s2.SetEnable(is_enable);
// }
//
// void Chassis_DualSteer::OnLoop() {
//     forward();
//     backward();
//
//     // 更新电机
//     motor.w1.OnLoop();
//     motor.w2.OnLoop();
//     motor.s1.OnLoop();
//     motor.s2.OnLoop();
// }
//
// void Chassis_DualSteer::backward() {
//     Chassis::backward();
//
//     // 速度解算
//     vz.ref = wr.ref * config.chassis_radius;
//     // 左前舵轮
//     const UnitFloat<m_s> vx1 = vx.chassis.ref - sqrt2div2 * vz.ref;
//     const UnitFloat<m_s> vy1 = vy.chassis.ref + sqrt2div2 * vz.ref;
//     if (unit::abs(vx1) > MIN_V || unit::abs(vy1) > MIN_V) { // 最小转舵速度
//         s1.ref = unit::atan2(vy1, vx1);
//     }
//     v1.ref = unit::sqrt(unit::square(vy1) + unit::square(vx1));
//     if (unit::abs(Angle(s1.ref - s1.measure)) > 90 * deg) { // 就近转向
//         s1.ref = s1.ref + 180 * deg;
//         v1.ref = -v1.ref;
//     }
//
//     // 右后舵轮
//     const UnitFloat<m_s> vx2 = vx.chassis.ref + sqrt2div2 * vz.ref;
//     const UnitFloat<m_s> vy2 = vy.chassis.ref - sqrt2div2 * vz.ref;
//     if (unit::abs(vx2) > MIN_V || unit::abs(vy2) > MIN_V) { // 最小转舵速度
//         s2.ref = unit::atan2(vy2, vx2);
//     }
//     v2.ref = unit::sqrt(unit::square(vy2) + unit::square(vx2));
//     if (unit::abs(Angle(s2.ref - s2.measure)) > 90 * deg) { // 就近转向
//         s2.ref = s2.ref + 180 * deg;
//         v2.ref = -v2.ref;
//     }
//
//     // 设置电机转速
//     motor.w1.SetSpeed(v1.ref / config.wheel_radius);
//     motor.w2.SetSpeed(v2.ref / config.wheel_radius);
//
//     // 设置舵向
//     motor.s1.SetAngle(s1.ref);
//     motor.s2.SetAngle(s2.ref);
// }
//
// void Chassis_DualSteer::forward() {
//     // 读取舵向
//     s1.measure = motor.s1.angle.measure;
//     s2.measure = motor.s2.angle.measure;
//
//     // 读取电机转速
//     v1.measure = motor.w1.speed.measure * config.wheel_radius;
//     v2.measure = motor.w2.speed.measure * config.wheel_radius;
//
//     // 速度解算
//     const UnitFloat<m_s> vx1 = v1.measure * unit::cos(s1.measure);
//     const UnitFloat<m_s> vy1 = v1.measure * unit::sin(s1.measure);
//     const UnitFloat<m_s> vz1 = v1.measure * unit::cos(s1.measure - 135 * deg);
//     const UnitFloat<m_s> vx2 = v2.measure * unit::cos(s2.measure);
//     const UnitFloat<m_s> vy2 = v2.measure * unit::sin(s2.measure);
//     const UnitFloat<m_s> vz2 = v2.measure * unit::cos(s2.measure + 45 * deg);
//     vx.chassis.measure = (vx1 + vx2) / 2;
//     vy.chassis.measure = (vy1 + vy2) / 2;
//     vz.measure = (vz1 + vz2) / 2;
//     wr.measure = vz.measure / config.chassis_radius;
//     std::tie(vx.gimbal.measure, vy.gimbal.measure) = rotate(vx.chassis.measure, vy.chassis.measure, -gimbal_yaw); // 转换到云台参考系
//
//     // 读取电机扭矩
//     f1.measure = motor.w1.torque.measure / config.wheel_radius;
//     f2.measure = motor.w2.torque.measure / config.wheel_radius;
//
//     // 力学解算
//     const UnitFloat<N> fx1 = f1.measure * unit::cos(s1.measure);
//     const UnitFloat<N> fy1 = f1.measure * unit::sin(s1.measure);
//     const UnitFloat<N> fz1 = f1.measure * unit::cos(s1.measure - 135 * deg);
//     const UnitFloat<N> fx2 = f2.measure * unit::cos(s2.measure);
//     const UnitFloat<N> fy2 = f2.measure * unit::sin(s2.measure);
//     const UnitFloat<N> fz2 = f2.measure * unit::cos(s2.measure + 45 * deg);
//     fx.chassis.measure = fx1 + fx2;
//     fy.chassis.measure = fy1 + fy2;
//     fz.measure = fz1 + fz2;
//     std::tie(fx.gimbal.measure, fy.gimbal.measure) = rotate(fx.chassis.measure, fy.chassis.measure, -gimbal_yaw); // 转换到云台参考系
// }