#include "Chassis.hpp"

Chassis::Chassis(const config_t& config) : config(config),
                                           vx_pid(config.vxyz_pid_config), vy_pid(config.vxyz_pid_config), vz_pid(config.vxyz_pid_config),
                                           follow_pid(config.follow_pid_config) {}

void Chassis::SetEnable(bool is_enable) {
    if (this->is_enable == is_enable) return;
    this->is_enable = is_enable;

    SetMode(DETACH_MODE); // 失能/使能后默认分离模式，防止车突然转动伤人

    // 这里设置电机
}

void Chassis::SetMode(mode_e mode) {
    this->mode = mode;
}

void Chassis::SetSpeed(const UnitFloat<>& vx, const UnitFloat<>& vy, const UnitFloat<>& wr) {
    this->vx.ref = vx;
    this->vy.ref = vy;
    this->wr.input = wr; // 这里是角速度，在calcPID中会计算底盘跟随，同时补充ref和vz
}

void Chassis::SetGimbalYaw(const Angle<>& gimbal_yaw) {
    this->gimbal_yaw = gimbal_yaw;
}

void Chassis::SetPowerLimit(const UnitFloat<>& power_limit) {
    power.limit = power_limit;
}

void Chassis::SetBufferEnergy(const UnitFloat<>& buffer_energy) {
    power.buffer_energy = buffer_energy;
}

void Chassis::OnLoop() {
    forward();
    backward();

    // 这里更新电机
}

void Chassis::backward() {
    calcPID();
    powerControl();
}

void Chassis::calcPID() {
    // 底盘跟随PID
    if (wr.input != 0) { // 优先用户旋转输入
        wr.ref = wr.input;
    } else {
        if (mode == DETACH_MODE) { // 底盘分离模式
            wr.ref = 0 * default_unit;
        } else if (mode == FOLLOW_MODE) { // 底盘跟随模式
            wr.ref = follow_pid.Calculate(gimbal_yaw);
        }
    }
    vz.ref = wr.ref * config.chassis_radius; // 计算线速度

    // xyz方向PID
    fx.ref = vx_pid.Calculate(vx.ref - vx.measure);
    fy.ref = vy_pid.Calculate(vy.ref - vy.measure);
    fz.ref = vz_pid.Calculate(vz.ref - vz.measure);
}

void Chassis::powerControl() {
    // 功率估计
    power.estimate_xy = fx.measure * vx.measure + fy.measure * vy.measure;
    power.estimate_z = fz.measure * vz.measure;
    power.estimate = power.estimate_xy + power.estimate_z;

    // 未连接裁判系统时，读出的功率是0，禁用功率控制
    // 裁判系统设置无限功率，读出来是65535，禁用功率控制（这时候缓冲能量还在异常工作，必须在这里禁用）
    if (power.limit == 0 * W || power.limit == 65535 * W) return;

    // 牵引力衰减系数
    auto& gain_x = power.gain_x;
    auto& gain_y = power.gain_y;
    auto& gain_xy = power.gain_xy;
    auto& gain_z = power.gain_z;
    gain_x = gain_y = gain_xy = gain_z = 1 * ratio; // 初始化为1，方便后面*=运算

    // 基于功率估计的功率控制
    const UnitFloat<W> power_xy = fx.ref * vx.measure + fy.ref * vy.measure;
    const UnitFloat<W> power_z = fz.ref * vz.measure;
    if (power_xy > power.limit) { // 衰减移动功率，停止旋转
        gain_xy *= power.limit / power_xy;
        gain_z *= 0;
    } else if (power_xy + power_z > power.limit) { // 只衰减旋转功率
        gain_z *= (power.limit - power_xy) / power_z;
    }

    // 基于缓冲能量的功率控制
    const auto& buffer_energy = power.buffer_energy;
    if (buffer_energy < 30 * J) { // 停止移动和旋转
        gain_xy *= 0;
        gain_z *= 0;
    } else if (buffer_energy < 45 * J) { // 衰减移动功率，停止旋转
        gain_xy *= (buffer_energy - 30 * J) / (15 * J);
        gain_z *= 0;
    } else if (buffer_energy < 60 * J) { // 只衰减旋转功率
        gain_z *= (buffer_energy - 45 * J) / (15 * J);
    }

    // 按xy牵引力比例分配
    gain_x = 1 * ratio - (1 * ratio - gain_xy) * fx.ref / (fx.ref + fy.ref);
    gain_y = 1 * ratio - (1 * ratio - gain_xy) * fy.ref / (fx.ref + fy.ref);

    // 应用牵引力衰减
    fx.ref *= gain_x;
    fy.ref *= gain_y;
    fz.ref *= gain_z;
}