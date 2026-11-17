// 编译真实云台、Motor、PID，仅模拟时钟和 CAN 电机反馈
#include "robot/gimbal/gimbal_classic.hpp"
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
        raw_angle = dir * angle;
        raw_speed = dir * speed;
        can_callback();
    }
};

struct Fixture {
    Pid::Config speed_pid = {.kp = 1, .max_out = 20};
    Pid::Config angle_pid = {.kp = 1, .max_out = 5};
    TestMotor yaw, pitch;
    GimbalClassic gimbal;

    Fixture(const bool use_angle_loop, const bool is_torque = true, const bool is_invert = false,
            const float pitch_min = -0.3f, const float pitch_max = 0.5f)
        : yaw(false),
          pitch(is_torque, is_invert),
          gimbal({yaw, pitch}, {.yaw_speed_pid_config = &speed_pid,
                                .yaw_angle_pid_config = &angle_pid,
                                .pitch_speed_pid_config = &speed_pid,
                                .pitch_angle_pid_config = &angle_pid,
                                .pitch_min = pitch_min,
                                .pitch_max = pitch_max,
                                .use_angle_loop = use_angle_loop}) {
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
        check(f.gimbal.yaw.speed_measure, 0.3f, "joint yaw recovery");
        check(f.gimbal.pitch.speed_measure, 0.15f, "joint pitch recovery");
        check(f.gimbal.yaw.speed_ref, 0.7f - 0.4f, "yaw speed target includes chassis feedforward");
        check(f.gimbal.pitch.speed_ref, -0.2f, "pitch speed command on motor axis");
        check(f.yaw.current_ref, 0.7f - 0.3f - 0.4f, "yaw error subtracts chassis wr");
        check(f.pitch.torque_ref, -0.35f, "pitch IMU feedback");
        check(f.gimbal.yaw.angle_ref, f.gimbal.yaw.angle_measure, "speed mode yaw target follows measure");
        check(f.gimbal.pitch.angle_ref, f.gimbal.pitch.angle_measure, "speed mode pitch target follows measure");
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

    // pitch 越限只截断输出，保留 IMU 目标
    f.gimbal.set_angle(0, 1.0f);
    f.step(Quaternionf::Identity(), Vector3f::Zero(), 0.5f);
    check(f.pitch.torque_ref, 0, "angle mode ECD upper protection");
    check(f.gimbal.pitch.angle_ref, 1.0f, "output protection preserves IMU target");
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

int main() {
    test_speed_feedback();
    test_angle_loop();
    test_tilted_convergence();
    test_pitch_protection();
    test_enable_and_connection();
    std::printf("GimbalClassic IMU: %d checks passed\n", checks);
}
