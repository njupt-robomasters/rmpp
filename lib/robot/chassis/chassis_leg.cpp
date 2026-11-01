#include "chassis_leg.hpp"
#include "math/angle.hpp"
#include <algorithm>
#include <cmath>

ChassisLeg::ChassisLeg(const Config& config, const Motors& motor)
    : config_(config),
      motor_(motor),
      leg_ref_(config.leg_min),
      leg_length_pid_{Pid(config.leg_length_pid), Pid(config.leg_length_pid)},
      leg_angle_pid_{Pid(config.leg_angle_pid), Pid(config.leg_angle_pid)},
      roll_pid_(config.roll_pid) {}

void ChassisLeg::set_enable(const bool is_enable) {
    is_enable_ = is_enable;
    motor_.left_a.set_enable(is_enable);
    motor_.left_e.set_enable(is_enable);
    motor_.right_a.set_enable(is_enable);
    motor_.right_e.set_enable(is_enable);
    motor_.left_wheel.set_enable(is_enable);
    motor_.right_wheel.set_enable(is_enable);
}

void ChassisLeg::set_speed(const float d_x, const float d_yaw) {
    d_x_ref_ = d_x;
    d_yaw_ref_ = d_yaw;
}

void ChassisLeg::set_leg_length(const float length) {
    leg_ref_ = std::clamp(length, config_.leg_min, config_.leg_max);
}

void ChassisLeg::set_rescue(const bool can_rescue) {
    can_rescue_ = can_rescue;
}

void ChassisLeg::set_jump(const bool should_jump) {
    should_jump_ = should_jump;
}

void ChassisLeg::update(const Quaternion& q, const Vector3& gyro) {
    const float dt = timer_.restart();

    // 电机在线检测；任一电机掉线或关节未处于使能态（含报错）时本周期全部输出零
    motor_.left_a.update();
    motor_.left_e.update();
    motor_.right_a.update();
    motor_.right_e.update();
    motor_.left_wheel.update();
    motor_.right_wheel.update();
    is_motor_ok = motor_.left_a.is_connect && motor_.left_a.err_code == DM8009::ENABLED &&
                  motor_.left_e.is_connect && motor_.left_e.err_code == DM8009::ENABLED &&
                  motor_.right_a.is_connect && motor_.right_a.err_code == DM8009::ENABLED &&
                  motor_.right_e.is_connect && motor_.right_e.err_code == DM8009::ENABLED &&
                  motor_.left_wheel.is_connect && motor_.right_wheel.is_connect;

    // 车体姿态：标准 Z-Y-X 欧拉角，pitch 低头为正、roll 右侧低为正；角速度取机体系 gyro
    const EulerAngles euler = quaternion_to_euler(q);
    body.pitch = euler.pitch;
    body.d_pitch = gyro.y();
    body.roll = euler.roll;
    body.d_roll = gyro.x();
    body.d_yaw = gyro.z();

    // 关节角转杆方位角：电机角逆时针为正、杆方位角顺时针为正，零位对应 phi1 = π、phi4 = 0
    leg[0].phi1 = pi - motor_.left_a.position;
    leg[0].phi4 = 0 - motor_.left_e.position;
    leg[1].phi1 = pi - motor_.right_a.position;
    leg[1].phi4 = 0 - motor_.right_e.position;
    leg[0].d_phi1 = -motor_.left_a.speed;
    leg[0].d_phi4 = -motor_.left_e.speed;
    leg[1].d_phi1 = -motor_.right_a.speed;
    leg[1].d_phi4 = -motor_.right_e.speed;
    solve_leg(leg[0]);
    solve_leg(leg[1]);

    // 相对腿角：左右腿镜像安装，分别反向、同向换算为车体正下方为零、后摆为正
    leg[0].rel_theta = Angle(-leg[0].phi0 + pi / 2);
    leg[1].rel_theta = Angle(leg[1].phi0 - pi / 2);
    // 腿倾角：叠加车体俯仰，低头时轮轴相对竖直方向后摆
    leg[0].theta = Angle(leg[0].rel_theta + body.pitch);
    leg[1].theta = Angle(leg[1].rel_theta + body.pitch);
    leg[0].d_theta = -leg[0].d_phi0 + body.d_pitch;
    leg[1].d_theta = leg[1].d_phi0 + body.d_pitch;

    for (auto& l : leg) {
        // 重力取非负沿腿分量，气弹簧取等效伸腿力幅值
        l.Fg = std::max(config_.mass * 9.8f / 2 * std::cos(l.theta), 0.0f);
        l.Fs = std::fabs(0.5f * config_.spring_force * std::sin(l.knee) * std::sin(pi - 0.5f * l.spread - l.knee));

        // 腿摆动引起的前向速度补偿
        l.v_swing = l.d_L * std::sin(l.theta) + l.L * l.d_theta * std::cos(l.theta);
    }

    // 轮速：电机安装方向已由 is_invert 修正为前进为正
    leg[0].v_wheel = motor_.left_wheel.speed * config_.wheel_radius;
    leg[1].v_wheel = motor_.right_wheel.speed * config_.wheel_radius;

    // 两侧均值为前向速度，右减左为航向角速度（逆时针正）
    const float v_left = leg[0].v_wheel + leg[0].v_swing;
    const float v_right = leg[1].v_wheel + leg[1].v_swing;
    kin.d_x = (v_left + v_right) / 2;
    kin.d_yaw = (v_right - v_left) / config_.wheel_distance;

    // 位移积分：停机、倒地、跳跃或有速度指令时清零，速度降到 0.1 m/s 内后开始积分
    if (!is_enable_ || rescue.is_fallen || jump.is_active || std::fabs(d_x_ref_) > 0.01f) {
        kin.x = 0;
        is_position_ = false;
    } else {
        if (std::fabs(kin.d_x) < 0.1f)
            is_position_ = true;
        if (is_position_)
            kin.x = std::clamp(kin.x + kin.d_x * dt, -config_.x_max, config_.x_max);
    }

    // 倒地与自救：失能或跳跃时复位
    if (!is_enable_ || jump.is_active) {
        rescue = {};
    } else {
        for (const auto& l : leg)
            if (l.rel_theta < config_.fall_theta_front || l.rel_theta > config_.fall_theta_back)
                rescue.is_fallen = true;
        if (std::fabs(body.roll) > 90 * deg)
            rescue.is_fallen = true;

        // 倒地且空闲时接受启动许可：横滚区分正向与翻倒，翻倒时低头（pitch > 0）为头部着地
        if (rescue.is_fallen && can_rescue_ && !rescue.is_running) {
            rescue.is_running = true;
            if (std::fabs(body.roll) < 90 * deg)
                rescue.fall = Fall::UPRIGHT;
            else if (body.pitch > 0)
                rescue.fall = Fall::HEAD;
            else
                rescue.fall = Fall::TAIL;

            // 以当前腿角启动，避免目标阶跃
            rescue.target_theta[0] = leg[0].rel_theta;
            rescue.target_theta[1] = leg[1].rel_theta;
        }

        if (rescue.is_running) {
            if (rescue.fall == Fall::UPRIGHT)
                rescue_stand(dt);
            else if (rescue.fall == Fall::HEAD)
                rescue_flip(-1, dt);
            else
                rescue_flip(1, dt);
        }
    }

    // 跳跃：上升沿且空闲时启动，动作期间不排队；失能或倒地时复位
    const bool is_jump_rising = should_jump_ && !last_should_jump_;
    last_should_jump_ = should_jump_;
    if (!is_enable_ || rescue.is_fallen) {
        jump = {};
    } else {
        if (is_jump_rising)
            jump.is_active = true;

        if (jump.is_active) {
            if (jump.step == 0) { // 起跳，双腿伸到位后收腿
                jump.force = config_.jump_takeoff_force;
                if (leg[0].L >= config_.jump_takeoff_length && leg[1].L >= config_.jump_takeoff_length) {
                    jump.step = 1;
                    jump.force = config_.jump_retract_force;
                }
            } else if (jump.step == 1) { // 收腿，双腿收到位后保持
                jump.force = config_.jump_retract_force;
                if (leg[0].L <= config_.jump_retract_length && leg[1].L <= config_.jump_retract_length) {
                    jump.step = 2;
                    jump.force = 0;
                    jump.hold_time = 0;
                }
            } else if (jump.step == 2) { // 收腿长度保持
                jump.hold_time += dt;
                if (jump.hold_time >= config_.jump_retract_time) {
                    jump.step = 3;
                    jump.hold_time = 0;
                }
            } else { // 落地缓冲，从进入阶段起计时，不等待触地
                jump.hold_time += dt;
                if (jump.hold_time >= config_.jump_buffer_time)
                    jump = {};
            }
        }
    }

    // 虚拟力：先清零，再按状态计算
    for (auto& l : leg)
        l.Fn = l.Tt = l.Tw = 0;

    if (!is_enable_) {
        // 失能时保持零输出
    } else if (rescue.is_fallen) {
        // 倒地：轮端不输出；自救时摆动力矩跟踪目标相对腿角，仅站起收腿阶段输出轴向力
        if (rescue.is_running) {
            for (int i = 0; i < 2; i++) {
                leg[i].Tt = leg_angle_pid_[i].calculate(Angle(rescue.target_theta[i] - leg[i].rel_theta));
                if (rescue.stand_step[i] == 1)
                    leg[i].Fn = leg_length_pid_[i].calculate(config_.leg_min - leg[i].L) + leg[i].Fg - leg[i].Fs;
            }
        }
    } else {
        // 轴向力
        if (!jump.is_active) {
            balance_legs(leg_ref_);
        } else if (jump.step == 0) { // 起跳：跳跃力叠加重力补偿
            for (auto& l : leg)
                l.Fn = jump.force + l.Fg - l.Fs;
        } else if (jump.step == 1) { // 收腿：离地不补偿重力
            for (auto& l : leg)
                l.Fn = jump.force - l.Fs;
        } else if (jump.step == 2) { // 收腿保持：腿长 PD，不补偿重力
            for (int i = 0; i < 2; i++)
                leg[i].Fn = leg_length_pid_[i].calculate(config_.jump_retract_length - leg[i].L) - leg[i].Fs;
        } else { // 落地缓冲
            balance_legs(config_.jump_buffer_length);
        }

        // LQR 增益：按左右腿长求值二次拟合多项式
        const float Ll = leg[0].L, Lr = leg[1].L;
        const float terms[6] = {1, Ll, Lr, Ll * Ll, Ll * Lr, Lr * Lr};

        // 状态误差，均为“目标减反馈”；航向角不跟随云台，误差恒为零
        const float err[10] = {
            std::clamp((0 - kin.x) * 10, -config_.x_err_max, config_.x_err_max),                         // 前向位移，前进为正
            d_x_ref_ - kin.d_x,                                                                          // 前向速度，前进为正
            0,                                                                                           // 航向角，未启用
            d_yaw_ref_ - body.d_yaw,                                                                     // 航向角速度，逆时针为正
            std::clamp(config_.theta_ref - leg[0].theta, -config_.theta_err_max, config_.theta_err_max), // 左腿倾角，后摆为正
            0 - leg[0].d_theta,                                                                          // 左腿倾角速度
            std::clamp(config_.theta_ref - leg[1].theta, -config_.theta_err_max, config_.theta_err_max), // 右腿倾角，后摆为正
            0 - leg[1].d_theta,                                                                          // 右腿倾角速度
            std::clamp(0 - body.pitch, -config_.pitch_err_max, config_.pitch_err_max),                   // 俯仰，低头为正
            0 - body.d_pitch,                                                                            // 俯仰角速度，低头为正
        };

        // 输出 [0/1] 左/右轮力矩，前进为正；[2/3] 左/右摆动力矩，后摆为正
        float out[4]{};
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 10; j++) {
                float k = 0;
                for (int n = 0; n < 6; n++)
                    k += config_.lqr_fit[i][j][n] * terms[n];
                out[i] += k * err[j];
            }
        }
        leg[0].Tw = out[0];
        leg[1].Tw = out[1];
        leg[0].Tt = out[2];
        leg[1].Tt = out[3];

        // 起跳时取消摆动力矩，把关节力矩留给伸腿；收腿及保持阶段离地，关闭轮端
        if (jump.is_active && jump.step == 0) {
            leg[0].Tt = leg[1].Tt = 0;
        } else if (jump.is_active && (jump.step == 1 || jump.step == 2)) {
            leg[0].Tw = leg[1].Tw = 0;
        }
    }

    // 力映射：右腿镜像安装，后摆力矩在机构中为反向
    map_force(leg[0], leg[0].Tt);
    map_force(leg[1], -leg[1].Tt);

    // 电机异常时全部输出零
    if (!is_motor_ok) {
        for (auto& l : leg)
            l.Tw = l.T_A = l.T_E = 0;
    }

    motor_.left_a.set_torque(leg[0].T_A);
    motor_.left_e.set_torque(leg[0].T_E);
    motor_.right_a.set_torque(leg[1].T_A);
    motor_.right_e.set_torque(leg[1].T_E);
    motor_.left_wheel.set_torque(leg[0].Tw);
    motor_.right_wheel.set_torque(leg[1].Tw);
}

void ChassisLeg::solve_leg(Leg& leg) const {
    const float a = config_.active_link, b = config_.passive_link;

    // 膝点间位移 D - B，m
    const float x_db = a * std::cos(leg.phi4) - a * std::cos(leg.phi1);
    const float y_db = a * std::sin(leg.phi4) - a * std::sin(leg.phi1);

    // 以 B、D 为圆心求轮轴 C；正根对应当前装配分支，不能任意切换
    const float A0 = 2 * b * x_db;
    const float B0 = 2 * b * y_db;
    const float C0 = x_db * x_db + y_db * y_db;
    leg.phi2 = 2 * std::atan2(B0 + std::sqrt(A0 * A0 + B0 * B0 - C0 * C0), A0 + C0);

    const float x_c = a * std::cos(leg.phi1) + b * std::cos(leg.phi2);
    const float y_c = a * std::sin(leg.phi1) + b * std::sin(leg.phi2);

    // 轮轴位置转虚拟腿极坐标
    leg.L = std::sqrt(x_c * x_c + y_c * y_c);
    leg.phi0 = std::atan2(y_c, x_c);
    leg.phi3 = std::atan2(-y_db + b * std::sin(leg.phi2), -x_db + b * std::cos(leg.phi2));

    // 闭链速度约束：v_bc 为从动杆 BC 的有符号切向速度，m/s
    const float v_bc = (a * leg.d_phi1 * std::sin(leg.phi1 - leg.phi3) + a * leg.d_phi4 * std::sin(leg.phi3 - leg.phi4)) / std::sin(leg.phi3 - leg.phi2);
    const float d_x_c = -a * leg.d_phi1 * std::sin(leg.phi1) - v_bc * std::sin(leg.phi2);
    const float d_y_c = a * leg.d_phi1 * std::cos(leg.phi1) + v_bc * std::cos(leg.phi2);

    // 轮轴速度分解为伸缩速度与方位角速度
    leg.d_L = (x_c * d_x_c + y_c * d_y_c) / leg.L;
    leg.d_phi0 = (x_c * d_y_c - y_c * d_x_c) / (leg.L * leg.L);

    // 膝内角为 [0, π]；主动杆角差有符号，仅对超过一周的正角差回绕一次
    leg.knee = std::acos(std::clamp((a * a + b * b - leg.L * leg.L) / (2 * a * b), -1.0f, 1.0f));
    leg.spread = leg.phi1 - leg.phi4;
    if (leg.spread > rev)
        leg.spread -= rev;
}

void ChassisLeg::map_force(Leg& leg, const float Tt) const {
    const float a = config_.active_link, b = config_.passive_link;

    // beta 为轮轴 C 处 CA 与 CB 的夹角
    const float cos_beta = (leg.L * leg.L + b * b - a * a) / (2 * leg.L * b);
    const float tan_beta = std::sqrt(1 - cos_beta * cos_beta) / cos_beta;

    // 轴向力形成反向关节力矩，摆动力矩由两关节同向平分
    const float T1 = 0.5f * leg.Fn * leg.L * tan_beta;
    const float T2 = 0.5f * Tt;
    leg.T_A = T1 + T2;
    leg.T_E = -T1 + T2;
}

void ChassisLeg::balance_legs(const float length) {
    // 横滚补偿：右侧低（roll > 0）时输出为负，左腿缩短、右腿伸长
    // 外部工程 imu_roll 与 roll 反号，其写法为左 L - out、右 L + out；此处左 +、右 - 是同一物理方向
    const float roll_out = roll_pid_.calculate(0 - body.roll, 0 - body.d_roll);
    const float length_ref[2] = {
        std::clamp(length + roll_out, config_.leg_min, config_.leg_max),
        std::clamp(length - roll_out, config_.leg_min, config_.leg_max),
    };

    // 腿长 PD 叠加重力补偿、扣除弹簧力
    for (int i = 0; i < 2; i++)
        leg[i].Fn = leg_length_pid_[i].calculate(length_ref[i] - leg[i].L) + leg[i].Fg - leg[i].Fs;
}

void ChassisLeg::rescue_stand(const float dt) {
    const float step = config_.rescue_stand_speed * dt;

    for (int i = 0; i < 2; i++) {
        float& target = rescue.target_theta[i];
        if (rescue.stand_step[i] == 0) {
            // 向前摆腿进入收腿区间
            if (target < -57 * deg || target > 72 * deg)
                target = Angle(target - step);
            if (leg[i].rel_theta >= -60 * deg && leg[i].rel_theta <= 80 * deg)
                rescue.stand_step[i] = 1;
        } else {
            // 收腿时继续将目标腿角移向零
            if (target > 0)
                target = Angle(target - step);
            else
                target = Angle(target + step);
            if (std::fabs(target) <= 11 * deg)
                target = 0;
        }
    }

    // 双腿均进入收腿阶段且反馈回正，判定站起完成
    if (rescue.stand_step[0] == 1 && rescue.stand_step[1] == 1 &&
        std::fabs(leg[0].rel_theta) < 23 * deg && std::fabs(leg[1].rel_theta) < 23 * deg)
        rescue = {};
}

void ChassisLeg::rescue_flip(const float side, const float dt) {
    // side：尾部着地 +1，腿向后翻转到 [90°, 166°]；头部着地 -1，镜像到 [-166°, -90°]
    // 以下区间均为 side × 角度
    const float step = config_.rescue_flip_speed * dt;

    for (int i = 0; i < 2; i++) {
        float& target = rescue.target_theta[i];
        const float rel = side * leg[i].rel_theta;
        if (rescue.flip_step[i] == 0) {
            // 左腿先动，右腿等左腿进入阶段 1
            if (i == 0 || rescue.flip_step[0] == 1) {
                if (!(side * target >= 90 * deg && side * target <= 149 * deg))
                    target = Angle(target - side * step);
                if (rel >= 90 * deg && rel <= 166 * deg)
                    rescue.flip_step[i] = 1;
            }
        } else if (rescue.flip_step[i] == 1) {
            // 双腿就绪后继续翻转，越过正下方前停在 90°
            if (rescue.flip_step[1 - i] != 0) {
                target = Angle(target - side * step);
                if (side * target >= 0 && side * target <= 90 * deg)
                    target = side * 90 * deg;
                if (rel >= 57 * deg && rel <= 106 * deg)
                    rescue.flip_step[i] = 2;
            }
        }
    }

    if (rescue.flip_step[0] == 2 && rescue.flip_step[1] == 2)
        rescue_stand(dt);
}
