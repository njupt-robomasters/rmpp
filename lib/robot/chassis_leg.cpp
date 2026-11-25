#include "chassis_leg.hpp"
#include "math/angle.hpp"
#include <algorithm>
#include <cmath>

ChassisLeg::ChassisLeg(const Motors& motor, const Config& config)
    : motor(motor),
      config(config),
      leg_length(config.leg_min),
      leg_length_pid{Pid(config.leg_length_pid), Pid(config.leg_length_pid)},
      leg_angle_pid{Pid(config.leg_angle_pid), Pid(config.leg_angle_pid)},
      roll_pid(config.roll_pid) {}

void ChassisLeg::set_enable(const bool is_enable) {
    this->is_enable = is_enable;
    motor.left_back.set_enable(is_enable);
    motor.left_front.set_enable(is_enable);
    motor.right_front.set_enable(is_enable);
    motor.right_back.set_enable(is_enable);
    motor.left_wheel.set_enable(is_enable);
    motor.right_wheel.set_enable(is_enable);
}

void ChassisLeg::set_allow_rescue(const bool allow_rescue) {
    this->allow_rescue = allow_rescue;
}

void ChassisLeg::set_trigger_jump(const bool trigger_jump) {
    this->trigger_jump = trigger_jump;
}

void ChassisLeg::set_power_ref(const float power_ref) {
    this->power_ref = power_ref;
}

void ChassisLeg::set_speed(const float vx, const float wr) {
    this->vx = vx;
    this->wr = wr;
}

void ChassisLeg::set_gimbal_yaw(const float gimbal_yaw) {
    this->gimbal_yaw = gimbal_yaw;
}

void ChassisLeg::set_leg_length(const float leg_length) {
    this->leg_length = std::clamp(leg_length, config.leg_min, config.leg_max);
}

void ChassisLeg::update(const Quaternionf& q, const Vector3f& gyro) {
    const float dt = std::clamp(update_timer.restart(), 0.0001f, 0.01f);

    update_feedback(q, gyro);

    // 航向角速度目标：小陀螺手动角速度，或云台跟随前馈
    d_yaw_ref = wr;

    // 两侧均值为前向速度，右减左为航向角速度（逆时针正）
    const float v_left = leg[0].v_wheel + leg[0].v_swing;
    const float v_right = leg[1].v_wheel + leg[1].v_swing;
    body.d_x = (v_left + v_right) / 2;
    body.d_yaw_wheel = (v_right - v_left) / config.wheel_distance;

    // 位移积分：停机、倒地、跳跃或有速度指令时清零，速度降到 0.1 m/s 内后开始积分
    if (!is_enable || rescue.state != Rescue::State::UPRIGHT || jump.state != Jump::State::IDLE || std::fabs(vx) > 0.01f) {
        body.x = 0;
        is_position = false;
    } else {
        if (std::fabs(body.d_x) < 0.1f)
            is_position = true;
        if (is_position)
            body.x = std::clamp(body.x + body.d_x * dt, -config.x_max, config.x_max);
    }

    update_rescue(dt);
    update_jump(dt);
    calc_force(dt);

    // VMC 力映射：右腿镜像安装，后摆力矩在机构中为反向
    vmc_map(leg[0], leg[0].Tt);
    vmc_map(leg[1], -leg[1].Tt);

    // 电机异常时全部输出零：任一电机掉线或关节未处于使能态（含报错）
    is_motor_ok = motor.left_back.is_connect && motor.left_back.err_code == DM8009::ENABLED &&
                  motor.left_front.is_connect && motor.left_front.err_code == DM8009::ENABLED &&
                  motor.right_front.is_connect && motor.right_front.err_code == DM8009::ENABLED &&
                  motor.right_back.is_connect && motor.right_back.err_code == DM8009::ENABLED &&
                  motor.left_wheel.is_connect && motor.right_wheel.is_connect;
    if (!is_motor_ok) {
        for (auto& l : leg)
            l.Tw = l.Ta = l.Te = 0;
    }

    motor.left_back.set_torque(leg[0].Ta);
    motor.left_front.set_torque(leg[0].Te);
    motor.right_front.set_torque(leg[1].Ta);
    motor.right_back.set_torque(leg[1].Te);
    motor.left_wheel.set_torque(leg[0].Tw);
    motor.right_wheel.set_torque(leg[1].Tw);
}

void ChassisLeg::update_feedback(const Quaternionf& q, const Vector3f& gyro) {
    // 电机在线检测
    motor.left_back.update();
    motor.left_front.update();
    motor.right_front.update();
    motor.right_back.update();
    motor.left_wheel.update();
    motor.right_wheel.update();

    // 车体姿态：标准 Z-Y-X 欧拉角，pitch 低头为正、roll 右侧低为正；角速度取机体系 gyro
    const EulerAngles euler = quaternion_to_euler(q);
    body.pitch = euler.pitch;
    body.d_pitch = gyro.y();
    body.roll = euler.roll;
    body.d_roll = gyro.x();
    body.d_yaw_gyro = gyro.z();

    // 关节角转杆方位角：左腿 A=left_back/E=left_front，右腿 A=right_front/E=right_back
    // 电机角逆时针为正、杆方位角顺时针为正，零位对应 phi1 = π、phi4 = 0
    auto set_leg_angles = [](Leg& l, const DM8009& joint_a, const DM8009& joint_e) {
        l.phi1 = pi - joint_a.angle;
        l.phi4 = 0 - joint_e.angle;
        l.d_phi1 = -joint_a.speed;
        l.d_phi4 = -joint_e.speed;
    };
    set_leg_angles(leg[0], motor.left_back, motor.left_front);
    set_leg_angles(leg[1], motor.right_front, motor.right_back);
    solve_leg(leg[0]);
    solve_leg(leg[1]);

    // 相对腿角：左右腿镜像安装，分别反向、同向换算为车体正下方为零、后摆为正
    // 腿倾角：叠加车体俯仰，低头时轮轴相对竖直方向后摆
    leg[0].rel_theta = Angle(-leg[0].phi0 + pi / 2);
    leg[1].rel_theta = Angle(leg[1].phi0 - pi / 2);
    leg[0].theta = Angle(leg[0].rel_theta + body.pitch);
    leg[1].theta = Angle(leg[1].rel_theta + body.pitch);
    leg[0].d_theta = -leg[0].d_phi0 + body.d_pitch;
    leg[1].d_theta = leg[1].d_phi0 + body.d_pitch;

    for (auto& l : leg) {
        // 重力取非负沿腿分量，气弹簧取等效伸腿力幅值
        l.Fg = std::max(config.mass * 9.8f / 2 * std::cos(l.theta), 0.0f);
        l.Fs = std::fabs(0.5f * config.spring_force * std::sin(l.phi_obc) * std::sin(pi - 0.5f * l.phi_bod - l.phi_obc));

        // 腿摆动引起的前向速度补偿
        l.v_swing = l.d_L * std::sin(l.theta) + l.L * l.d_theta * std::cos(l.theta);
    }

    // 轮速：电机安装方向已由 is_invert 修正为前进为正
    leg[0].v_wheel = motor.left_wheel.speed * config.wheel_radius;
    leg[1].v_wheel = motor.right_wheel.speed * config.wheel_radius;
}

void ChassisLeg::solve_leg(Leg& leg) const {
    const float a = config.active_link, b = config.passive_link;

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

void ChassisLeg::update_rescue(const float dt) {
    // 倒地与自救：失能或跳跃时复位
    if (!is_enable || jump.state != Jump::State::IDLE) {
        rescue = {};
    } else {
        for (const auto& l : leg)
            if (l.rel_theta < config.fall_theta_front || l.rel_theta > config.fall_theta_back)
                rescue.state = Rescue::State::FALLEN;
        if (std::fabs(body.roll) > 90 * deg)
            rescue.state = Rescue::State::FALLEN;

        // 倒地且空闲时接受启动许可：横滚区分正向与翻倒，翻倒时低头（pitch > 0）为头部着地
        if (rescue.state == Rescue::State::FALLEN && allow_rescue) {
            rescue.state = Rescue::State::RUNNING;
            if (std::fabs(body.roll) < 90 * deg)
                rescue.dir = Rescue::FallDir::NORMAL;
            else if (body.pitch > 0)
                rescue.dir = Rescue::FallDir::HEAD;
            else
                rescue.dir = Rescue::FallDir::TAIL;

            // 以当前腿角启动，避免目标阶跃
            rescue.target_theta[0] = leg[0].rel_theta;
            rescue.target_theta[1] = leg[1].rel_theta;
        }

        if (rescue.state == Rescue::State::RUNNING) {
            if (rescue.dir == Rescue::FallDir::NORMAL)
                rescue_stand(dt);
            else if (rescue.dir == Rescue::FallDir::HEAD)
                rescue_flip(-1, dt);
            else
                rescue_flip(1, dt);
        }
    }
}

void ChassisLeg::rescue_stand(const float dt) {
    const float step = config.rescue_stand_speed * dt;

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
    if (rescue.stand_step[0] == 1 && rescue.stand_step[1] == 1 && std::fabs(leg[0].rel_theta) < 23 * deg && std::fabs(leg[1].rel_theta) < 23 * deg) {
        rescue = {};
    }
}

void ChassisLeg::rescue_flip(const float side, const float dt) {
    // side：尾部着地 +1，腿向后翻转到 [90°, 166°]；头部着地 -1，镜像到 [-166°, -90°]
    // 以下区间均为 side × 角度
    const float step = config.rescue_flip_speed * dt;

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

void ChassisLeg::update_jump(const float dt) {
    // 跳跃：上升沿且空闲时启动，动作期间不排队；失能或倒地时复位
    const bool is_jump_rising = trigger_jump && !last_trigger_jump;
    last_trigger_jump = trigger_jump;
    if (!is_enable || rescue.state != Rescue::State::UPRIGHT) {
        jump = {};
    } else {
        if (is_jump_rising && jump.state == Jump::State::IDLE)
            jump.state = Jump::State::TAKEOFF;

        if (jump.state == Jump::State::TAKEOFF) {
            // 起跳，双腿伸到位后收腿
            jump.force = config.jump_takeoff_force;
            if (leg[0].L >= config.leg_max && leg[1].L >= config.leg_max) {
                jump.state = Jump::State::RETRACT;
                jump.force = config.jump_retract_force;
            }
        } else if (jump.state == Jump::State::RETRACT) {
            // 收腿，双腿收到位后保持
            jump.force = config.jump_retract_force;
            if (leg[0].L <= config.leg_min && leg[1].L <= config.leg_min) {
                jump.state = Jump::State::HOLD;
                jump.force = 0;
                jump.hold_time = 0;
            }
        } else if (jump.state == Jump::State::HOLD) {
            // 收腿长度保持
            jump.hold_time += dt;
            if (jump.hold_time >= config.jump_retract_time) {
                jump.state = Jump::State::BUFFER;
                jump.hold_time = 0;
            }
        } else if (jump.state == Jump::State::BUFFER) {
            // 落地缓冲，从进入阶段起计时，不等待触地
            jump.hold_time += dt;
            if (jump.hold_time >= config.jump_buffer_time)
                jump = {};
        }
    }
}

void ChassisLeg::calc_force(const float dt) {
    // 虚拟力：先清零，再按状态计算
    for (auto& l : leg)
        l.Fn = l.Tt = l.Tw = 0;

    // 相对腿角 PID 只用于自救，其余时间清空，下次自救从零开始
    if (!is_enable || rescue.state == Rescue::State::UPRIGHT) {
        for (Pid& pid : leg_angle_pid)
            pid.clear();
    }

    if (!is_enable) {
        // 失能时保持零输出
    } else if (rescue.state != Rescue::State::UPRIGHT) {
        // 倒地：轮端不输出；自救时摆动力矩跟踪目标相对腿角，仅站起收腿阶段输出轴向力
        if (rescue.state == Rescue::State::RUNNING) {
            for (int i = 0; i < 2; i++) {
                leg[i].Tt = leg_angle_pid[i].calculate(dt, Angle(rescue.target_theta[i] - leg[i].rel_theta));
                if (rescue.stand_step[i] == 1)
                    leg[i].Fn = leg_length_pid[i].calculate(dt, config.leg_min - leg[i].L, 0 - leg[i].d_L) + leg[i].Fg - leg[i].Fs;
            }
        }
    } else {
        // 轴向力
        if (jump.state == Jump::State::IDLE) {
            balance_legs(dt, leg_length);
        } else if (jump.state == Jump::State::TAKEOFF) {
            // 起跳：跳跃力叠加重力补偿
            for (auto& l : leg)
                l.Fn = jump.force + l.Fg - l.Fs;
        } else if (jump.state == Jump::State::RETRACT) {
            // 收腿：离地不补偿重力
            for (auto& l : leg)
                l.Fn = jump.force - l.Fs;
        } else if (jump.state == Jump::State::HOLD) {
            // 收腿保持：腿长 PD，不补偿重力
            for (int i = 0; i < 2; i++)
                leg[i].Fn = leg_length_pid[i].calculate(dt, config.leg_min - leg[i].L, 0 - leg[i].d_L) - leg[i].Fs;
        } else {
            // 落地缓冲
            balance_legs(dt, config.jump_buffer_length);
        }

        calc_lqr();

        // 起跳时取消摆动力矩，把关节力矩留给伸腿；收腿及保持阶段离地，关闭轮端
        if (jump.state == Jump::State::TAKEOFF) {
            leg[0].Tt = leg[1].Tt = 0;
        } else if (jump.state == Jump::State::RETRACT || jump.state == Jump::State::HOLD) {
            leg[0].Tw = leg[1].Tw = 0;
        }
    }
}

void ChassisLeg::balance_legs(const float dt, const float length) {
    // 横滚补偿：右侧低（roll > 0）时输出为负，左腿缩短、右腿伸长
    // 外部工程 imu_roll 与 roll 反号，其写法为左 L - out、右 L + out；此处左 +、右 - 是同一物理方向
    const float roll_out = roll_pid.calculate(dt, 0 - body.roll, 0 - body.d_roll);
    const float length_ref[2] = {
        std::clamp(length + roll_out, config.leg_min, config.leg_max),
        std::clamp(length - roll_out, config.leg_min, config.leg_max),
    };

    // 腿长 PD 叠加重力补偿、扣除弹簧力；微分取伸缩速度，不含目标变化
    for (int i = 0; i < 2; i++)
        leg[i].Fn = leg_length_pid[i].calculate(dt, length_ref[i] - leg[i].L, 0 - leg[i].d_L) + leg[i].Fg - leg[i].Fs;
}

void ChassisLeg::calc_lqr() {
    // LQR 增益：按左右腿长求值二次拟合多项式
    const float Ll = leg[0].L, Lr = leg[1].L;
    const float terms[6] = {1, Ll, Lr, Ll * Ll, Ll * Lr, Lr * Lr};

    // 目标限速：直行目标上限为轮子额定转速乘轮半径，边走边转按较快一侧轮缘速度等比缩小两个目标
    const float travel_speed = config.wheel_work_speed * config.wheel_radius;
    const float rim_target = std::fabs(vx) + std::fabs(wr) * config.wheel_distance / 2;
    const float target_share = rim_target > travel_speed ? travel_speed / rim_target : 1;

    // 状态误差，均为"目标减反馈"；位移误差乘 10 以匹配固件拟合表；小陀螺时上板发 0 即不追踪航向
    const float err[10] = {
        (0 - body.x) * 10,                          // 前向位移，前进为正
        target_share * vx - body.d_x,               // 前向速度，前进为正
        gimbal_yaw,                                 // 航向角，跟随模式为云台相对底盘航向
        target_share * d_yaw_ref - body.d_yaw_gyro, // 航向角速度，逆时针为正
        config.theta_ref - leg[0].theta,            // 左腿倾角，后摆为正
        0 - leg[0].d_theta,                         // 左腿倾角速度
        config.theta_ref - leg[1].theta,            // 右腿倾角，后摆为正
        0 - leg[1].d_theta,                         // 右腿倾角速度
        0 - body.pitch,                             // 俯仰，低头为正
        0 - body.d_pitch,                           // 俯仰角速度，低头为正
    };

    // 贡献分组：运动四项为位移、前进速度、航向、旋转速度，姿态六项为两腿角及角速度、俯仰及角速度
    // 输出 [0/1] 左/右轮力矩，前进为正；[2/3] 左/右摆动力矩，后摆为正
    float k[4][10]{}, base[4]{}, motion[4]{};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 10; j++) {
            for (int n = 0; n < 6; n++)
                k[i][j] += config.lqr_fit[i][j][n] * terms[n];
            if (j < 4)
                motion[i] += k[i][j] * err[j];
            else
                base[i] += k[i][j] * err[j];
        }
    }

    float out[4];
    for (int i = 0; i < 4; i++)
        out[i] = base[i] + motion[i];

    // 跳跃期间关节余量被伸腿力占用，不做分配与功率修正，由电机自身限幅
    if (jump.state == Jump::State::IDLE) {
        // 四路额定区间：轮取电机力矩上限；腿摆由两关节平分，当拍轴向力矩先占用各关节余量
        const float wheel_max = motor.left_wheel.config.max_torque;
        const float joint_max = motor.left_front.config.max_torque;
        float lower[4] = {-wheel_max, -wheel_max, 0, 0};
        float upper[4] = {wheel_max, wheel_max, 0, 0};
        for (int i = 0; i < 2; i++) {
            const float swing_max = 2 * std::max(joint_max - std::fabs(calc_axial_torque(leg[i])), 0.0f);
            lower[2 + i] = -swing_max;
            upper[2 + i] = swing_max;
        }

        // 姿态组优先：运动组按四路都能满足的最大共同比例让位；全不可行时运动归零，再整体缩小姿态组
        const float share = maximum_share(base, motion, lower, upper);
        if (share >= 0) {
            for (int i = 0; i < 4; i++)
                out[i] = base[i] + share * motion[i];
        } else {
            const float zero[4]{};
            const float base_share = std::max(maximum_share(zero, base, lower, upper), 0.0f);
            for (int i = 0; i < 4; i++)
                out[i] = base_share * base[i];
        }

        // 功率：双轮净机械功率 Σ Tw·ω 超过 power_ref 时，把轮力矩投影到功率线上，轮缺口经腿部映射补给两腿
        // 约束作用于已相加的轮力矩，保留运动项与扶正项的抵消；制动（负功率）与不限功率（INFINITY）直接通过
        const Vector2f omega(motor.left_wheel.speed, motor.right_wheel.speed);
        const Vector2f wheel(out[0], out[1]);
        wheel_power = omega.dot(wheel);
        const float excess = wheel_power - power_ref;
        if (excess > 0) {
            // 腿部映射 M = D·C⁻¹，把轮缺口按俯仰角速度、旋转速度两个反馈方向表达
            // C：两轮对俯仰角速度、旋转速度的增益；D 第一列把缺少的俯仰阻尼交给两腿稳姿态增益，第二列撤回腿的旋转速度项
            Matrix2f C, D;
            for (int i = 0; i < 2; i++) {
                float posture = 0;
                for (int n = 0; n < 6; n++)
                    posture += config.lqr_posture_fit[i][n] * terms[n];
                C(i, 0) = k[i][9];
                C(i, 1) = k[i][3];
                D(i, 0) = posture - k[2 + i][9];
                D(i, 1) = -k[2 + i][3];
            }
            const Matrix2f M = D * C.inverse();

            // 轮缺口 d：四路改动 [-d; M·d] 按 R 加权的平方和最小，H = R轮 + Mᵀ·R腿·M，d 沿 H⁻¹·ω 且恰好消去超出功率
            Matrix2f H = M.transpose() * Vector2f(config.lqr_r[2], config.lqr_r[3]).asDiagonal() * M;
            H(0, 0) += config.lqr_r[0];
            H(1, 1) += config.lqr_r[1];
            const Vector2f direction = H.inverse() * omega;
            Vector2f limited = wheel - excess / omega.dot(direction) * direction;

            // 投影越过轮额定区间时，从功率线上同比缩小的可行点 anchor 向投影点截取，最多检查两路
            const Vector2f anchor = wheel * (power_ref / wheel_power);
            const Vector2f delta = limited - anchor;
            float share = 1;
            for (int i = 0; i < 2; i++) {
                if (limited[i] < lower[i])
                    share = std::min(share, (lower[i] - anchor[i]) / delta[i]);
                else if (limited[i] > upper[i])
                    share = std::min(share, (upper[i] - anchor[i]) / delta[i]);
            }
            limited = anchor + share * delta;

            // 两腿：原指令加 M·最终轮缺口，再限到摆动额定区间；腿顶到上限时优先满足力矩与功率硬约束
            const Vector2f swing = Vector2f(out[2], out[3]) + M * (wheel - limited);
            out[0] = limited[0];
            out[1] = limited[1];
            out[2] = std::clamp(swing[0], lower[2], upper[2]);
            out[3] = std::clamp(swing[1], lower[3], upper[3]);
        }
    }

    leg[0].Tw = out[0];
    leg[1].Tw = out[1];
    leg[0].Tt = out[2];
    leg[1].Tt = out[3];
}

float ChassisLeg::calc_axial_torque(const Leg& leg) const {
    const float a = config.active_link, b = config.passive_link;

    // ∠OCB 为轮轴 C 处 CO 与 CB 的夹角
    const float cos_ocb = (leg.L * leg.L + b * b - a * a) / (2 * leg.L * b);
    const float tan_ocb = std::sqrt(1 - cos_ocb * cos_ocb) / cos_ocb;

    // 轴向力形成反向关节力矩
    return 0.5f * leg.Fn * leg.L * tan_ocb;
}

float ChassisLeg::maximum_share(const float base[4], const float change[4], const float lower[4], const float upper[4]) {
    float minimum = 0, maximum = 1;
    for (int i = 0; i < 4; i++) {
        if (change[i] == 0) {
            if (base[i] < lower[i] || base[i] > upper[i])
                return -1;
            continue;
        }

        // 每路的可行区间换算为比例区间，与已有区间求交
        const float first = (lower[i] - base[i]) / change[i];
        const float second = (upper[i] - base[i]) / change[i];
        minimum = std::max(minimum, std::min(first, second));
        maximum = std::min(maximum, std::max(first, second));
    }
    return minimum <= maximum ? maximum : -1;
}

void ChassisLeg::vmc_map(Leg& leg, const float Tt) const {
    // 轴向力形成反向关节力矩，摆动力矩由两关节同向平分
    const float T1 = calc_axial_torque(leg);
    const float T2 = 0.5f * Tt;
    leg.Ta = T1 + T2;
    leg.Te = -T1 + T2;
}
