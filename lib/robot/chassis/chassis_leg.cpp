#include "chassis_leg.hpp"
#include "math/angle.hpp"
#include <algorithm>
#include <cmath>

ChassisLeg::ChassisLeg(const Motors& motor, const Config& config)
    : config_(config),
      motor_(motor),
      leg_length_ref_(config.leg_min),
      leg_length_pid_{Pid(config.leg_length_pid), Pid(config.leg_length_pid)},
      leg_angle_pid_{Pid(config.leg_angle_pid), Pid(config.leg_angle_pid)},
      roll_pid_(config.roll_pid) {}

void ChassisLeg::set_enable(const bool is_enable) {
    is_enable_ = is_enable;
    motor_.left_back.set_enable(is_enable);
    motor_.left_front.set_enable(is_enable);
    motor_.right_front.set_enable(is_enable);
    motor_.right_back.set_enable(is_enable);
    motor_.left_wheel.set_enable(is_enable);
    motor_.right_wheel.set_enable(is_enable);
}

void ChassisLeg::set_speed(const float d_x, const float d_yaw) {
    d_x_ref_ = d_x;
    d_yaw_ref_ = d_yaw;
}

void ChassisLeg::set_leg_length(const float length) {
    leg_length_ref_ = std::clamp(length, config_.leg_min, config_.leg_max);
}

void ChassisLeg::set_rescue(const bool can_rescue) {
    allow_rescue_ = can_rescue;
}

void ChassisLeg::set_jump(const bool should_jump) {
    trigger_jump_ = should_jump;
}

void ChassisLeg::update(const Quaternionf& q, const Vector3f& gyro) {
    const float dt = std::clamp(update_timer_.restart(), DT_MIN, DT_MAX);

    update_feedback(q, gyro);
    update_kin(dt);
    update_rescue(dt);
    update_jump(dt);
    calc_force(dt);

    // 力映射：右腿镜像安装，后摆力矩在机构中为反向
    map_force(leg_[0], leg_[0].Tt);
    map_force(leg_[1], -leg_[1].Tt);

    // 电机异常时全部输出零：任一电机掉线或关节未处于使能态（含报错）
    is_motor_ok_ = motor_.left_back.is_connect && motor_.left_back.err_code == DM8009::ENABLED &&
                   motor_.left_front.is_connect && motor_.left_front.err_code == DM8009::ENABLED &&
                   motor_.right_front.is_connect && motor_.right_front.err_code == DM8009::ENABLED &&
                   motor_.right_back.is_connect && motor_.right_back.err_code == DM8009::ENABLED &&
                   motor_.left_wheel.is_connect && motor_.right_wheel.is_connect;
    if (!is_motor_ok_) {
        for (auto& l : leg_)
            l.Tw = l.Ta = l.Te = 0;
    }

    motor_.left_back.set_torque(leg_[0].Ta);
    motor_.left_front.set_torque(leg_[0].Te);
    motor_.right_front.set_torque(leg_[1].Ta);
    motor_.right_back.set_torque(leg_[1].Te);
    motor_.left_wheel.set_torque(leg_[0].Tw);
    motor_.right_wheel.set_torque(leg_[1].Tw);
}

void ChassisLeg::update_feedback(const Quaternionf& q, const Vector3f& gyro) {
    // 电机在线检测
    motor_.left_back.update();
    motor_.left_front.update();
    motor_.right_front.update();
    motor_.right_back.update();
    motor_.left_wheel.update();
    motor_.right_wheel.update();

    // 车体姿态：标准 Z-Y-X 欧拉角，pitch 低头为正、roll 右侧低为正；角速度取机体系 gyro
    const EulerAngles euler = quaternion_to_euler(q);
    body_.pitch = euler.pitch;
    body_.d_pitch = gyro.y();
    body_.roll = euler.roll;
    body_.d_roll = gyro.x();
    body_.d_yaw_gyro = gyro.z();

    // 关节角转杆方位角：电机角逆时针为正、杆方位角顺时针为正，零位对应 phi1 = π、phi4 = 0
    leg_[0].phi1 = pi - motor_.left_back.position;
    leg_[0].phi4 = 0 - motor_.left_front.position;
    leg_[1].phi1 = pi - motor_.right_front.position;
    leg_[1].phi4 = 0 - motor_.right_back.position;
    leg_[0].d_phi1 = -motor_.left_back.speed;
    leg_[0].d_phi4 = -motor_.left_front.speed;
    leg_[1].d_phi1 = -motor_.right_front.speed;
    leg_[1].d_phi4 = -motor_.right_back.speed;
    solve_leg(leg_[0]);
    solve_leg(leg_[1]);

    // 相对腿角：左右腿镜像安装，分别反向、同向换算为车体正下方为零、后摆为正
    leg_[0].rel_theta = Angle(-leg_[0].phi0 + pi / 2);
    leg_[1].rel_theta = Angle(leg_[1].phi0 - pi / 2);
    // 腿倾角：叠加车体俯仰，低头时轮轴相对竖直方向后摆
    leg_[0].theta = Angle(leg_[0].rel_theta + body_.pitch);
    leg_[1].theta = Angle(leg_[1].rel_theta + body_.pitch);
    leg_[0].d_theta = -leg_[0].d_phi0 + body_.d_pitch;
    leg_[1].d_theta = leg_[1].d_phi0 + body_.d_pitch;

    for (auto& l : leg_) {
        // 重力取非负沿腿分量，气弹簧取等效伸腿力幅值
        l.Fg = std::max(config_.mass * 9.8f / 2 * std::cos(l.theta), 0.0f);
        l.Fs = std::fabs(0.5f * config_.spring_force * std::sin(l.phi_obc) * std::sin(pi - 0.5f * l.phi_bod - l.phi_obc));

        // 腿摆动引起的前向速度补偿
        l.v_swing = l.d_L * std::sin(l.theta) + l.L * l.d_theta * std::cos(l.theta);
    }

    // 轮速：电机安装方向已由 is_invert 修正为前进为正
    leg_[0].v_wheel = motor_.left_wheel.speed * config_.wheel_radius;
    leg_[1].v_wheel = motor_.right_wheel.speed * config_.wheel_radius;
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

    // 膝内角 ∠OBC 为 [0, π]；主动杆夹角 ∠BOD 有符号，仅对超过一周的正角差回绕一次
    leg.phi_obc = std::acos(std::clamp((a * a + b * b - leg.L * leg.L) / (2 * a * b), -1.0f, 1.0f));
    leg.phi_bod = leg.phi1 - leg.phi4;
    if (leg.phi_bod > rev)
        leg.phi_bod -= rev;
}

void ChassisLeg::update_kin(const float dt) {
    // 两侧均值为前向速度，右减左为航向角速度（逆时针正）
    const float v_left = leg_[0].v_wheel + leg_[0].v_swing;
    const float v_right = leg_[1].v_wheel + leg_[1].v_swing;
    body_.d_x = (v_left + v_right) / 2;
    body_.d_yaw_wheel = (v_right - v_left) / config_.wheel_distance;

    // 位移积分：停机、倒地、跳跃或有速度指令时清零，速度降到 0.1 m/s 内后开始积分
    if (!is_enable_ || rescue_.is_fallen || jump_.is_active || std::fabs(d_x_ref_) > 0.01f) {
        body_.x = 0;
        is_position_ = false;
    } else {
        if (std::fabs(body_.d_x) < 0.1f)
            is_position_ = true;
        if (is_position_)
            body_.x = std::clamp(body_.x + body_.d_x * dt, -config_.x_max, config_.x_max);
    }
}

void ChassisLeg::update_rescue(const float dt) {
    // 倒地与自救：失能或跳跃时复位
    if (!is_enable_ || jump_.is_active) {
        rescue_ = {};
    } else {
        for (const auto& l : leg_)
            if (l.rel_theta < config_.fall_theta_front || l.rel_theta > config_.fall_theta_back)
                rescue_.is_fallen = true;
        if (std::fabs(body_.roll) > 90 * deg)
            rescue_.is_fallen = true;

        // 倒地且空闲时接受启动许可：横滚区分正向与翻倒，翻倒时低头（pitch > 0）为头部着地
        if (rescue_.is_fallen && allow_rescue_ && !rescue_.is_running) {
            rescue_.is_running = true;
            if (std::fabs(body_.roll) < 90 * deg)
                rescue_.fall = Fall::UPRIGHT;
            else if (body_.pitch > 0)
                rescue_.fall = Fall::HEAD;
            else
                rescue_.fall = Fall::TAIL;

            // 以当前腿角启动，避免目标阶跃
            rescue_.target_theta[0] = leg_[0].rel_theta;
            rescue_.target_theta[1] = leg_[1].rel_theta;
        }

        if (rescue_.is_running) {
            if (rescue_.fall == Fall::UPRIGHT)
                rescue_stand(dt);
            else if (rescue_.fall == Fall::HEAD)
                rescue_flip(-1, dt);
            else
                rescue_flip(1, dt);
        }
    }
}

void ChassisLeg::rescue_stand(const float dt) {
    const float step = config_.rescue_stand_speed * dt;

    for (int i = 0; i < 2; i++) {
        float& target = rescue_.target_theta[i];
        if (rescue_.stand_step[i] == 0) {
            // 向前摆腿进入收腿区间
            if (target < -57 * deg || target > 72 * deg)
                target = Angle(target - step);
            if (leg_[i].rel_theta >= -60 * deg && leg_[i].rel_theta <= 80 * deg)
                rescue_.stand_step[i] = 1;
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
    if (rescue_.stand_step[0] == 1 && rescue_.stand_step[1] == 1 &&
        std::fabs(leg_[0].rel_theta) < 23 * deg && std::fabs(leg_[1].rel_theta) < 23 * deg)
        rescue_ = {};
}

void ChassisLeg::rescue_flip(const float side, const float dt) {
    // side：尾部着地 +1，腿向后翻转到 [90°, 166°]；头部着地 -1，镜像到 [-166°, -90°]
    // 以下区间均为 side × 角度
    const float step = config_.rescue_flip_speed * dt;

    for (int i = 0; i < 2; i++) {
        float& target = rescue_.target_theta[i];
        const float rel = side * leg_[i].rel_theta;
        if (rescue_.flip_step[i] == 0) {
            // 左腿先动，右腿等左腿进入阶段 1
            if (i == 0 || rescue_.flip_step[0] == 1) {
                if (!(side * target >= 90 * deg && side * target <= 149 * deg))
                    target = Angle(target - side * step);
                if (rel >= 90 * deg && rel <= 166 * deg)
                    rescue_.flip_step[i] = 1;
            }
        } else if (rescue_.flip_step[i] == 1) {
            // 双腿就绪后继续翻转，越过正下方前停在 90°
            if (rescue_.flip_step[1 - i] != 0) {
                target = Angle(target - side * step);
                if (side * target >= 0 && side * target <= 90 * deg)
                    target = side * 90 * deg;
                if (rel >= 57 * deg && rel <= 106 * deg)
                    rescue_.flip_step[i] = 2;
            }
        }
    }

    if (rescue_.flip_step[0] == 2 && rescue_.flip_step[1] == 2)
        rescue_stand(dt);
}

void ChassisLeg::update_jump(const float dt) {
    // 跳跃：上升沿且空闲时启动，动作期间不排队；失能或倒地时复位
    const bool is_jump_rising = trigger_jump_ && !last_trigger_jump_;
    last_trigger_jump_ = trigger_jump_;
    if (!is_enable_ || rescue_.is_fallen) {
        jump_ = {};
    } else {
        if (is_jump_rising)
            jump_.is_active = true;

        if (jump_.is_active) {
            if (jump_.step == 0) { // 起跳，双腿伸到位后收腿
                jump_.force = config_.jump_takeoff_force;
                if (leg_[0].L >= config_.leg_max && leg_[1].L >= config_.leg_max) {
                    jump_.step = 1;
                    jump_.force = config_.jump_retract_force;
                }
            } else if (jump_.step == 1) { // 收腿，双腿收到位后保持
                jump_.force = config_.jump_retract_force;
                if (leg_[0].L <= config_.leg_min && leg_[1].L <= config_.leg_min) {
                    jump_.step = 2;
                    jump_.force = 0;
                    jump_.hold_time = 0;
                }
            } else if (jump_.step == 2) { // 收腿长度保持
                jump_.hold_time += dt;
                if (jump_.hold_time >= config_.jump_retract_time) {
                    jump_.step = 3;
                    jump_.hold_time = 0;
                }
            } else { // 落地缓冲，从进入阶段起计时，不等待触地
                jump_.hold_time += dt;
                if (jump_.hold_time >= config_.jump_buffer_time)
                    jump_ = {};
            }
        }
    }
}

void ChassisLeg::calc_force(const float dt) {
    // 虚拟力：先清零，再按状态计算
    for (auto& l : leg_)
        l.Fn = l.Tt = l.Tw = 0;

    // 相对腿角 PID 只用于自救，其余时间清空，下次自救从零开始
    if (!is_enable_ || !rescue_.is_fallen || !rescue_.is_running) {
        for (Pid& pid : leg_angle_pid_)
            pid.clear();
    }

    if (!is_enable_) {
        // 失能时保持零输出
    } else if (rescue_.is_fallen) {
        // 倒地：轮端不输出；自救时摆动力矩跟踪目标相对腿角，仅站起收腿阶段输出轴向力
        if (rescue_.is_running) {
            for (int i = 0; i < 2; i++) {
                leg_[i].Tt = leg_angle_pid_[i].calculate(dt, Angle(rescue_.target_theta[i] - leg_[i].rel_theta));
                if (rescue_.stand_step[i] == 1)
                    leg_[i].Fn = leg_length_pid_[i].calculate(dt, config_.leg_min - leg_[i].L, 0 - leg_[i].d_L) + leg_[i].Fg - leg_[i].Fs;
            }
        }
    } else {
        // 轴向力
        if (!jump_.is_active) {
            balance_legs(dt, leg_length_ref_);
        } else if (jump_.step == 0) { // 起跳：跳跃力叠加重力补偿
            for (auto& l : leg_)
                l.Fn = jump_.force + l.Fg - l.Fs;
        } else if (jump_.step == 1) { // 收腿：离地不补偿重力
            for (auto& l : leg_)
                l.Fn = jump_.force - l.Fs;
        } else if (jump_.step == 2) { // 收腿保持：腿长 PD，不补偿重力
            for (int i = 0; i < 2; i++)
                leg_[i].Fn = leg_length_pid_[i].calculate(dt, config_.leg_min - leg_[i].L, 0 - leg_[i].d_L) - leg_[i].Fs;
        } else { // 落地缓冲
            balance_legs(dt, config_.jump_buffer_length);
        }

        calc_lqr();

        // 起跳时取消摆动力矩，把关节力矩留给伸腿；收腿及保持阶段离地，关闭轮端
        if (jump_.is_active && jump_.step == 0) {
            leg_[0].Tt = leg_[1].Tt = 0;
        } else if (jump_.is_active && (jump_.step == 1 || jump_.step == 2)) {
            leg_[0].Tw = leg_[1].Tw = 0;
        }
    }
}

void ChassisLeg::balance_legs(const float dt, const float length) {
    // 横滚补偿：右侧低（roll > 0）时输出为负，左腿缩短、右腿伸长
    // 外部工程 imu_roll 与 roll 反号，其写法为左 L - out、右 L + out；此处左 +、右 - 是同一物理方向
    const float roll_out = roll_pid_.calculate(dt, 0 - body_.roll, 0 - body_.d_roll);
    const float length_ref[2] = {
        std::clamp(length + roll_out, config_.leg_min, config_.leg_max),
        std::clamp(length - roll_out, config_.leg_min, config_.leg_max),
    };

    // 腿长 PD 叠加重力补偿、扣除弹簧力；微分取伸缩速度，不含目标变化
    for (int i = 0; i < 2; i++)
        leg_[i].Fn = leg_length_pid_[i].calculate(dt, length_ref[i] - leg_[i].L, 0 - leg_[i].d_L) + leg_[i].Fg - leg_[i].Fs;
}

void ChassisLeg::calc_lqr() {
    // LQR 增益：按左右腿长求值二次拟合多项式
    const float Ll = leg_[0].L, Lr = leg_[1].L;
    const float terms[6] = {1, Ll, Lr, Ll * Ll, Ll * Lr, Lr * Lr};

    // 状态误差，均为“目标减反馈”；航向角不跟随云台，误差恒为零
    const float err[10] = {
        std::clamp((0 - body_.x) * 10, -config_.x_err_max, config_.x_err_max),                        // 前向位移，前进为正
        d_x_ref_ - body_.d_x,                                                                         // 前向速度，前进为正
        0,                                                                                            // 航向角，未启用
        d_yaw_ref_ - body_.d_yaw_gyro,                                                                // 航向角速度，逆时针为正
        std::clamp(config_.theta_ref - leg_[0].theta, -config_.theta_err_max, config_.theta_err_max), // 左腿倾角，后摆为正
        0 - leg_[0].d_theta,                                                                          // 左腿倾角速度
        std::clamp(config_.theta_ref - leg_[1].theta, -config_.theta_err_max, config_.theta_err_max), // 右腿倾角，后摆为正
        0 - leg_[1].d_theta,                                                                          // 右腿倾角速度
        std::clamp(0 - body_.pitch, -config_.pitch_err_max, config_.pitch_err_max),                   // 俯仰，低头为正
        0 - body_.d_pitch,                                                                            // 俯仰角速度，低头为正
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
    leg_[0].Tw = out[0];
    leg_[1].Tw = out[1];
    leg_[0].Tt = out[2];
    leg_[1].Tt = out[3];
}

void ChassisLeg::map_force(Leg& leg, const float Tt) const {
    const float a = config_.active_link, b = config_.passive_link;

    // beta 为轮轴 C 处 CO 与 CB 的夹角
    const float cos_beta = (leg.L * leg.L + b * b - a * a) / (2 * leg.L * b);
    const float tan_beta = std::sqrt(1 - cos_beta * cos_beta) / cos_beta;

    // 轴向力形成反向关节力矩，摆动力矩由两关节同向平分
    const float T1 = 0.5f * leg.Fn * leg.L * tan_beta;
    const float T2 = 0.5f * Tt;
    leg.Ta = T1 + T2;
    leg.Te = -T1 + T2;
}
