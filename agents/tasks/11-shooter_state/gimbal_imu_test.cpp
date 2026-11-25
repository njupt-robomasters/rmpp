// 编译真实云台、Motor、PID，仅模拟时钟和 CAN 电机反馈
#include "robot/gimbal_classic.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

static uint64_t now_us = 1000000;
static int checks = 0;

void Timer::delay(float) {}

float Timer::elapsed() const {
    return static_cast<float>(now_us - last_cycles) * 1e-6f;
}

float Timer::restart() {
    dt = elapsed();
    last_cycles = now_us;
    return dt;
}

class TestMotor : public Motor {
  private:
    const bool is_torque;

  public:
    TestMotor(const bool is_torque, const bool is_invert = false)
        : Motor({.can = Can(1), .timeout = 0.005f, .is_invert = is_invert}, 20, 20, 1), is_torque(is_torque) {}

    void set_output(const float output) override {
        if (is_torque)
            set_torque(output);
        else
            set_current(output);
    }

    void feedback(const float angle, const float speed = 0) {
        const float dir = config.is_invert ? -1.0f : 1.0f;
        can_callback({.speed = dir * speed, .angle = dir * angle});
    }
};

struct Fixture {
    Pid::Config speed_pid = {.kp = 1, .max_out = 20};
    Pid::Config angle_pid = {.kp = 1, .max_out = 5};
    TestMotor yaw, pitch;
    GimbalClassic gimbal;

    // angle_loop 为 false 时角度 PID 的比例增益置 0，等效只有速度内环
    Fixture(const bool angle_loop, const bool is_torque = true, const bool is_invert = false,
            const float pitch_min = -0.3f, const float pitch_max = 0.5f)
        : yaw(false),
          pitch(is_torque, is_invert),
          gimbal({yaw, pitch}, {.yaw_speed_pid_config = &speed_pid,
                                .yaw_angle_pid_config = &angle_pid,
                                .pitch_speed_pid_config = &speed_pid,
                                .pitch_angle_pid_config = &angle_pid,
                                .pitch_min = pitch_min,
                                .pitch_max = pitch_max}) {
        angle_pid.kp = angle_loop ? 1.0f : 0.0f;
        gimbal.set_enable(true);
    }

    void step(const Quaternionf& q = Quaternionf::Identity(), const Vector3f& gyro = Vector3f::Zero(),
              const float theta = 0, const float motor_speed = 0, const uint64_t delta_us = 1000) {
        now_us += delta_us;
        yaw.feedback(0.1f, motor_speed);
        pitch.feedback(theta, -motor_speed);
        gimbal.update(q, gyro);
    }
};

static void check(const float actual, const float expected, const char* label, const float tolerance = 2e-5f) {
    if (!std::isfinite(actual) || std::fabs(actual - expected) > tolerance) {
        std::fprintf(stderr, "%s: actual %.9g expected %.9g\n", label, actual, expected);
        std::abort();
    }
    checks++;
}

static void test_speed_feedback() {
    Fixture f(false);
    const float yaw0 = 0.8f;
    f.gimbal.set_speed(0.7f, -0.2f);
    f.gimbal.set_chassis_wr(0.4f);
    for (int i = 0; i < 400; i++) {
        // 底盘水平，姿态由两关节决定；编码器速度噪声与反馈无关
        const float theta = -0.2f + 0.6f * static_cast<float>(i) / 399;
        const Quaternionf q = euler_to_quaternion({.yaw = yaw0, .pitch = theta});
        const Vector3f gyro(-std::sin(theta) * 0.3f, 0.15f, std::cos(theta) * 0.3f);
        f.step(q, gyro, theta, i % 2 == 0 ? 80.0f : -90.0f);
        check(f.gimbal.yaw.speed_measure, 0.3f - 0.4f, "joint yaw relative speed recovery");
        check(f.gimbal.pitch.speed_measure, 0.15f, "joint pitch recovery");
        check(f.gimbal.yaw.speed_ref, 0.7f - 0.4f, "yaw speed target includes chassis feedforward");
        check(f.gimbal.pitch.speed_ref, -0.2f, "pitch speed command on motor axis");
        check(f.yaw.current_ref, 0.7f - 0.3f, "yaw error preserves absolute speed command");
        check(f.pitch.torque_ref, -0.35f, "pitch IMU feedback");
        // 角度 PID 比例增益为 0：外环目标仍积分，修正为零，等效速度内环
        check(f.gimbal.yaw.speed_ref - f.gimbal.yaw.speed_measure, 0.7f - 0.3f, "speed mode yaw error unchanged");
        check(f.gimbal.pitch.speed_ref - f.gimbal.pitch.speed_measure, -0.35f, "speed mode pitch error unchanged");
    }
    f.gimbal.set_angle(1.5f, 1.2f);
    f.step();
    check(f.pitch.torque_ref, 0, "speed mode set_angle clears speed command");

    // 倾斜底盘的角速度沿反向机构链合成传感器输入，投影保留两轴分量
    const float theta = 0.3f;
    f.gimbal.set_chassis_wr(0);
    const Vector3f base_rate(0.8f, -0.4f, 0.6f);
    const Quaternionf yaw_rotation(Eigen::AngleAxisf(0.7f, Vector3f::UnitZ()));
    const Quaternionf pitch_rotation(Eigen::AngleAxisf(theta, Vector3f::UnitY()));
    const Vector3f axis_rate = yaw_rotation.conjugate() * base_rate;
    const Vector3f body_rate = pitch_rotation.conjugate() * (axis_rate + Vector3f(0, 0.2f, -0.3f));
    f.step(Quaternionf::Identity(), body_rate, theta);
    check(f.gimbal.yaw.speed_measure, base_rate.z() - 0.3f, "base motion retained in yaw projection");
    check(f.gimbal.pitch.speed_measure, axis_rate.y() + 0.2f, "base motion retained in pitch projection");
    f.step(Quaternionf::Identity(), Vector3f(1, 0, 0), theta);
    check(f.gimbal.yaw.speed_measure, -std::sin(theta), "barrel spin yaw-axis projection");

    // 倾斜底盘下速度指令直接作为电机轴目标
    f.gimbal.set_speed(0.7f, -0.2f);
    f.step(euler_to_quaternion({.yaw = 0.5f, .pitch = 0.3f, .roll = 0.4f}), Vector3f::Zero(), theta);
    check(f.gimbal.yaw.speed_ref, 0.7f, "tilted yaw speed command stays on motor axis");
    check(f.gimbal.pitch.speed_ref, -0.2f, "tilted pitch speed command stays on motor axis");
}

static void test_angle_loop() {
    Fixture f(true);
    f.step();
    // 枪口世界姿态保持时，底盘旋转对应 yaw 电机反转，闭环无额外误差
    f.gimbal.set_chassis_wr(1.0f);
    f.step();
    check(f.gimbal.yaw.speed_ref, -1.0f, "rotating chassis requires reverse joint target");
    check(f.gimbal.yaw.speed_measure, -1.0f, "stationary barrel has reverse joint feedback");
    check(f.yaw.current_ref, 0, "rotating chassis adds no world aiming error");
    f.gimbal.set_chassis_wr(0);
    f.gimbal.set_speed(0.5f, 0.25f);
    f.step();
    check(f.gimbal.yaw.angle_ref, 0.0005f, "angle mode integrates yaw");
    check(f.gimbal.pitch.angle_ref, 0.00025f, "angle mode integrates pitch");
    check(f.gimbal.yaw.speed_ref, 0.5005f, "yaw target combines rate and angle correction");
    check(f.gimbal.pitch.speed_ref, 0.25025f, "pitch target combines rate and angle correction");
    f.step(Quaternionf::Identity(), Vector3f::Zero(), 0, 0, 50000);
    check(f.gimbal.yaw.angle_ref, 0.0055f, "dt upper clamp");
    f.step(Quaternionf::Identity(), Vector3f::Zero(), 0, 0, 1);
    check(f.gimbal.yaw.angle_ref, 0.00555f, "dt lower clamp");

    // yaw 目标跨 ±pi 取最短误差
    f.gimbal.set_angle(pi - 0.01f, 0);
    f.step(euler_to_quaternion({.yaw = -pi + 0.01f}));
    check(f.gimbal.yaw.speed_ref, -0.02f, "shortest yaw error across wrap");

    // 编码器越限：目标钳回限位，向内回程输出保留
    f.gimbal.set_angle(0, 0.5f);
    f.step(euler_to_quaternion({.pitch = 0.4f}), Vector3f::Zero(), 0.6f);
    check(f.gimbal.pitch.angle_ref, 0.3f, "upper ECD limit mapped to IMU target");
    check(f.pitch.torque_ref, -0.1f, "angle mode retreats beyond ECD limit");
}

// pitch 编码器限位经 IMU 与编码器偏差钳制世界目标
static void test_pitch_target_limit() {
    // 水平底盘持续向外推杆，关节理想跟随：目标停在限位，撤掉向外前馈，累计超过 pi 也不反转
    Fixture f(true, true, false);
    f.angle_pid.kp = 10;
    float theta = 0;
    const auto step = [&]() {
        f.step(euler_to_quaternion({.yaw = 0.1f, .pitch = theta}), Vector3f::Zero(), theta);
        theta += f.gimbal.pitch.speed_ref * 0.001f;
    };
    step();
    f.gimbal.set_speed(0, 1);
    float theta_min = 1;
    for (int i = 0; i < 10000; i++) {
        step();
        if (i >= 5000)
            theta_min = std::min(theta_min, theta);
    }
    check(f.gimbal.pitch.angle_ref, 0.5f, "accumulated target held at upper limit");
    check(theta, 0.5f, "pitch settles at upper limit", 1e-4f);
    check(theta_min, 0.5f, "no reversal after long outward command", 1e-4f);
    check(f.gimbal.pitch.speed_ref, 0, "outward feedforward removed at limit", 1e-3f);
    f.gimbal.set_speed(0, -1);
    step();
    check(f.gimbal.pitch.angle_ref, 0.499f, "inward command leaves limit immediately", 1e-4f);

    // 底盘前倾 0.1 rad：限位平移到世界 [-0.2, 0.6]，set_angle 越界目标同样钳制
    const Quaternionf chassis = euler_to_quaternion({.pitch = 0.1f});
    const auto pose = [&](const float joint) { return chassis * Quaternionf(Eigen::AngleAxisf(joint, Vector3f::UnitY())); };
    f.gimbal.set_angle(0.1f, 1.0f);
    f.step(pose(0.2f), Vector3f::Zero(), 0.2f);
    check(f.gimbal.pitch.angle_ref, 0.6f, "tilted upper limit");
    f.gimbal.set_angle(0.1f, -1.0f);
    f.step(pose(0.2f), Vector3f::Zero(), 0.2f);
    check(f.gimbal.pitch.angle_ref, -0.2f, "tilted lower limit");

    // 界外目标被钳住时，向内的速度指令保留前馈
    f.gimbal.set_angle(0.1f, -1.0f);
    f.gimbal.set_speed(0, 0.2f);
    f.step(pose(0.2f), Vector3f::Zero(), 0.2f);
    check(f.gimbal.pitch.angle_ref, -0.2f, "inward command from clamped target");
    check(f.gimbal.pitch.speed_ref, 0.2f + 10 * (-0.2f - 0.3f), "inward feedforward retained", 1e-4f);
}

// 倾斜底盘、关节理想跟随速度目标，世界 yaw/pitch 收敛到目标且无 roll 偏差
static void test_tilted_convergence() {
    const EulerAngles tilts[] = {{.yaw = 0.3f, .pitch = 0.2f, .roll = 0.25f}, {.yaw = -1.0f, .pitch = -0.15f, .roll = -0.3f}};
    for (const auto& tilt : tilts) {
        Fixture f(true, true, false, -pi, pi);
        f.angle_pid.kp = 10;
        const Quaternionf chassis = euler_to_quaternion(tilt);
        float psi = 0.4f, theta = 0.1f;
        const auto pose = [&]() {
            return chassis * Quaternionf(Eigen::AngleAxisf(psi, Vector3f::UnitZ())) * Quaternionf(Eigen::AngleAxisf(theta, Vector3f::UnitY()));
        };
        f.step(pose(), Vector3f::Zero(), theta);
        check(f.gimbal.yaw.speed_ref, 0, "tilted startup holds measured yaw");
        check(f.gimbal.pitch.speed_ref, 0, "tilted startup holds measured pitch");

        const float yaw_target = f.gimbal.yaw.angle_measure + 0.6f;
        const float pitch_target = f.gimbal.pitch.angle_measure - 0.25f;
        f.gimbal.set_angle(yaw_target, pitch_target);
        for (int i = 0; i < 3000; i++) {
            f.step(pose(), Vector3f::Zero(), theta);
            psi += f.gimbal.yaw.speed_ref * 0.001f;
            theta += f.gimbal.pitch.speed_ref * 0.001f;
        }
        const auto euler = quaternion_to_euler(pose());
        check(euler.yaw, yaw_target, "tilted yaw converges", 1e-4f);
        check(euler.pitch, pitch_target, "tilted pitch converges", 1e-4f);
    }
}

static void test_pitch_protection() {
    for (const bool is_torque : {false, true}) {
        for (const bool is_invert : {false, true}) {
            Fixture f(false, is_torque, is_invert);
            const auto output = [&]() { return is_torque ? f.pitch.torque_ref : f.pitch.current_ref; };
            f.step();
            for (const float angle : {-0.4f, -0.3f, 0.5f, 0.6f}) {
                const float outward = angle < 0 ? -1.0f : 1.0f;
                // 指令向内、惯性运动过快，PID 仍可能产生向外的制动输出
                f.gimbal.set_speed(0, -outward);
                f.step(Quaternionf::Identity(), Vector3f(0, -2 * outward, 0), angle);
                check(output(), 0, "protect final native output at/beyond ECD limits");
                f.step(Quaternionf::Identity(), Vector3f::Zero(), angle);
                check(output(), -outward, "allow retreat at/beyond ECD limits");
            }
            f.gimbal.set_speed(0, 1);
            f.step(Quaternionf::Identity(), Vector3f::Zero(), 0.49f);
            check(output(), 1, "allow outward output inside range");

            f.speed_pid.ki = 10;
            f.speed_pid.max_i = 2;
            for (int i = 0; i < 300; i++)
                f.step(Quaternionf::Identity(), Vector3f::Zero(), 0.6f);
            f.gimbal.set_speed(0, 0);
            f.step();
            check(output(), 0, "protected output tracks integral");
        }
    }
}

static void test_enable_and_connection() {
    Fixture f(true);
    f.step();
    f.gimbal.set_angle(0.4f, 0.3f);
    f.step();
    f.gimbal.set_enable(false);
    f.step();
    check(f.yaw.current_ref, 0, "disable yaw");
    check(f.pitch.torque_ref, 0, "disable pitch");
    check(f.gimbal.yaw.angle_ref, 0, "disable follows IMU");
    f.gimbal.set_enable(true);
    f.step();
    check(f.yaw.current_ref, 0, "enable starts from measured pose");
    now_us += 10000;
    f.yaw.feedback(0);
    f.gimbal.update(Quaternionf::Identity(), Vector3f::Zero());
    check(f.yaw.current_ref, 0, "yaw requires connected pitch geometry");
    check(f.pitch.torque_ref, 0, "pitch disconnected");
    f.step();
    check(f.gimbal.pitch.angle_ref, 0, "reconnect follows IMU");

    Fixture speed(false);
    speed.gimbal.set_speed(0.5f, 0.2f);
    speed.step();
    now_us += 10000;
    speed.pitch.feedback(0);
    speed.gimbal.update(Quaternionf::Identity(), Vector3f::Zero());
    check(speed.yaw.current_ref, 0, "yaw disconnected");
    check(speed.gimbal.yaw.speed_ref, 0, "disconnected yaw clears speed target");
    check(speed.gimbal.pitch.speed_ref, 0.2f, "connected pitch retains speed target");
    check(speed.pitch.torque_ref, 0.2f, "pitch independent of yaw connection");
    speed.gimbal.set_enable(false);
    speed.step(Quaternionf::Identity(), Vector3f(0, 0.3f, 0.4f));
    check(speed.gimbal.yaw.speed_ref, 0, "disabled yaw clears speed target");
    check(speed.gimbal.pitch.speed_ref, 0, "disabled pitch clears speed target");
    check(speed.gimbal.yaw.speed_measure, 0.4f, "disabled yaw still refreshes feedback");
    check(speed.gimbal.pitch.speed_measure, 0.3f, "disabled pitch still refreshes feedback");
    check(speed.yaw.current_ref, 0, "disabled yaw output");
    check(speed.pitch.torque_ref, 0, "disabled pitch output");
}

static void test_angle_accel_limit() {
    // 默认无穷大：等效禁用加速度规划，与纯线性 PID（含 P/I/D 及 kp=0）逐帧一致
    for (const float kp : {20.0f, 0.0f}) {
        Pid::Config speed_cfg = {.kp = 2, .max_out = 30};
        Pid::Config angle_cfg = {.kp = kp, .ki = 3.0f, .kd = 0.2f, .max_i = 5, .max_out = 15};
        TestMotor yaw_motor(true), pitch_motor(true);
        GimbalClassic gimbal({yaw_motor, pitch_motor}, {.yaw_speed_pid_config = &speed_cfg,
                                                        .yaw_angle_pid_config = &angle_cfg,
                                                        .pitch_speed_pid_config = &speed_cfg,
                                                        .pitch_angle_pid_config = &angle_cfg,
                                                        .pitch_min = -1.0f,
                                                        .pitch_max = 1.0f});
        Pid raw_yaw(&angle_cfg), raw_pitch(&angle_cfg);
        gimbal.set_enable(true);
        now_us += 1000;
        yaw_motor.feedback(0);
        pitch_motor.feedback(0);
        gimbal.update(Quaternionf::Identity(), Vector3f::Zero());

        const float errs[] = {2.5f, -1.8f, 0.4f, -0.05f, 0.0f};
        for (const float e : errs) {
            gimbal.set_angle(e, -0.3f * e);
            now_us += 1000;
            yaw_motor.feedback(0);
            pitch_motor.feedback(0);
            gimbal.update(Quaternionf::Identity(), Vector3f::Zero());
            check(gimbal.yaw.speed_ref, raw_yaw.calculate(0.001f, e), "default inf matches raw yaw PID", 0);
            check(gimbal.pitch.speed_ref, raw_pitch.calculate(0.001f, -0.3f * e), "default inf matches raw pitch PID", 0);
        }
    }

    // 分段代数曲线：|e| <= e0 保持 kp*e，|e| > e0 切换为 ±sqrt(2*a_max*(|e| - e0/2))，e0 处连续光滑
    Pid::Config speed_cfg = {.kp = 1, .max_out = 50};
    Pid::Config angle_cfg = {.kp = 20, .max_out = 12};
    const float a_yaw = 40.0f, a_pitch = 20.0f; // e0_yaw = 0.1, e0_pitch = 0.05
    TestMotor yaw_motor(true), pitch_motor(true);
    GimbalClassic gimbal({yaw_motor, pitch_motor}, {.yaw_speed_pid_config = &speed_cfg,
                                                    .yaw_angle_pid_config = &angle_cfg,
                                                    .pitch_speed_pid_config = &speed_cfg,
                                                    .pitch_angle_pid_config = &angle_cfg,
                                                    .pitch_min = -1.5f,
                                                    .pitch_max = 1.5f,
                                                    .yaw_max_accel = a_yaw,
                                                    .pitch_max_accel = a_pitch});
    gimbal.set_enable(true);
    now_us += 1000;
    yaw_motor.feedback(0);
    pitch_motor.feedback(0);
    gimbal.update(Quaternionf::Identity(), Vector3f::Zero());

    const auto step_err = [&](const float ey, const float ep) {
        gimbal.set_angle(ey, ep);
        now_us += 1000;
        yaw_motor.feedback(0);
        pitch_motor.feedback(0);
        gimbal.update(Quaternionf::Identity(), Vector3f::Zero());
    };
    step_err(0.08f, -0.04f);
    check(gimbal.yaw.speed_ref, 20.0f * 0.08f, "yaw linear inside e0");
    check(gimbal.pitch.speed_ref, 20.0f * -0.04f, "pitch linear inside e0");

    step_err(0.1f, -0.05f);
    check(gimbal.yaw.speed_ref, 2.0f, "yaw C0 continuous at e0");
    check(gimbal.pitch.speed_ref, -1.0f, "pitch C0 continuous at e0");

    // 切换点导数连续：e0 + de 处的增量斜率约等于 kp
    const float de = 1e-4f;
    step_err(0.1f + de, 0);
    check((gimbal.yaw.speed_ref - 2.0f) / de, 20.0f, "yaw C1 slope continuous at e0", 2e-2f);

    step_err(0.85f, -0.525f);
    check(gimbal.yaw.speed_ref, std::sqrt(2 * a_yaw * (0.85f - 0.05f)), "yaw sqrt braking curve");
    check(gimbal.pitch.speed_ref, -std::sqrt(2 * a_pitch * (0.525f - 0.025f)), "pitch sqrt braking curve");

    step_err(-2.5f, 0);
    check(gimbal.yaw.speed_ref, -12.0f, "sqrt curve respects PID max_out");

    // max_accel = 0 输出零修正
    GimbalClassic zero_accel({yaw_motor, pitch_motor}, {.yaw_speed_pid_config = &speed_cfg,
                                                        .yaw_angle_pid_config = &angle_cfg,
                                                        .pitch_speed_pid_config = &speed_cfg,
                                                        .pitch_angle_pid_config = &angle_cfg,
                                                        .pitch_min = -1.5f,
                                                        .pitch_max = 1.5f,
                                                        .yaw_max_accel = 0,
                                                        .pitch_max_accel = 0});
    zero_accel.set_enable(true);
    now_us += 1000;
    yaw_motor.feedback(0);
    pitch_motor.feedback(0);
    zero_accel.update(Quaternionf::Identity(), Vector3f::Zero());
    zero_accel.set_angle(1.0f, -0.5f);
    now_us += 1000;
    yaw_motor.feedback(0);
    pitch_motor.feedback(0);
    zero_accel.update(Quaternionf::Identity(), Vector3f::Zero());
    check(zero_accel.yaw.speed_ref, 0, "zero max_accel gives zero yaw correction");
    check(zero_accel.pitch.speed_ref, 0, "zero max_accel gives zero pitch correction");

    // 二阶刚体 + 电机力矩饱和闭环仿真：J = 0.05 kg·m²，tau_max = 3 N·m（物理最大角加速度 60 rad/s²）
    const auto simulate_yaw = [](const float max_accel, const float init_yaw, const float target_yaw, float& max_overshoot) {
        constexpr float inertia = 0.05f, dt = 0.001f;
        Pid::Config spd = {.kp = 5.0f, .max_out = 3.0f};
        Pid::Config ang = {.kp = 30.0f, .max_out = 12.0f};
        TestMotor ym(true), pm(true);
        GimbalClassic g({ym, pm}, {.yaw_speed_pid_config = &spd,
                                   .yaw_angle_pid_config = &ang,
                                   .pitch_speed_pid_config = &spd,
                                   .pitch_angle_pid_config = &ang,
                                   .pitch_min = -1.0f,
                                   .pitch_max = 1.0f,
                                   .yaw_max_accel = max_accel});
        g.set_enable(true);
        float psi = init_yaw, omega = 0;
        now_us += 1000;
        ym.feedback(psi, omega);
        pm.feedback(0, 0);
        g.update(euler_to_quaternion({.yaw = psi}), Vector3f(0, 0, omega));
        g.set_angle(target_yaw, 0);

        const float sign = target_yaw >= init_yaw ? 1.0f : -1.0f;
        max_overshoot = 0;
        for (int i = 0; i < 1500; i++) {
            now_us += 1000;
            ym.feedback(psi, omega);
            pm.feedback(0, 0);
            g.update(euler_to_quaternion({.yaw = psi}), Vector3f(0, 0, omega));
            omega += (ym.torque_ref / inertia) * dt;
            psi += omega * dt;
            max_overshoot = std::max(max_overshoot, sign * (psi - target_yaw));
        }
        return psi;
    };

    // 对照：禁用规划时 1.5 rad 阶跃因力矩饱和产生大幅过冲；启用 48 rad/s²（0.8 倍物理上限）后阶跃与外力掰偏均无过冲收敛
    float overshoot_unplanned = 0, overshoot_step = 0, overshoot_disturb = 0;
    simulate_yaw(INFINITY, 0.0f, 1.5f, overshoot_unplanned);
    if (!(overshoot_unplanned > 0.5f)) {
        std::fprintf(stderr, "expected saturation overshoot without planning, got %.9g\n", overshoot_unplanned);
        std::abort();
    }
    checks++;

    const float final_step = simulate_yaw(48.0f, 0.0f, 1.5f, overshoot_step);
    check(overshoot_step, 0.0f, "planned large step has no overshoot", 1e-4f);
    check(final_step, 1.5f, "planned large step settles to target", 1e-4f);

    const float final_disturb = simulate_yaw(48.0f, 1.5f, 0.0f, overshoot_disturb);
    check(overshoot_disturb, 0.0f, "planned large disturbance recovery has no overshoot", 1e-4f);
    check(final_disturb, 0.0f, "planned large disturbance settles to zero", 1e-4f);

    // 小角度（|e| <= e0 = 48 / 900 ≈ 0.0533 rad）轨迹与禁用规划时一致
    float overshoot_small_inf = 0, overshoot_small_plan = 0;
    const float small_inf = simulate_yaw(INFINITY, 0.0f, 0.03f, overshoot_small_inf);
    const float small_plan = simulate_yaw(48.0f, 0.0f, 0.03f, overshoot_small_plan);
    check(small_plan, small_inf, "small-angle response unaffected by accel planning", 0);
}

int main() {
    test_speed_feedback();
    test_angle_loop();
    test_tilted_convergence();
    test_pitch_target_limit();
    test_pitch_protection();
    test_enable_and_connection();
    test_angle_accel_limit();
    std::printf("GimbalClassic IMU: %d checks passed\n", checks);
}
