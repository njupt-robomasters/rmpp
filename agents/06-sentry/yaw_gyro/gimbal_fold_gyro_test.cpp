// 编译真实 GimbalFold、Motor、Pid，验证小 yaw 的 IMU 角速度内环，以及大 yaw 和 pitch2 保持原编码器速度内环
#include "robot/gimbal_fold.hpp"
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

void Can::add_rx_callback(RxCallback) const {}

void Can::send(const CanFrame&) const {}

class TestMotor : public Motor {
  private:
    const bool is_torque;

  public:
    explicit TestMotor(const bool is_torque)
        : Motor({.can = Can(1), .timeout = 0.01f}, 20, 20, 1), is_torque(is_torque) {}

    void set_output(const float output) override {
        if (is_torque)
            set_torque(output);
        else
            set_current(output);
    }

    void feedback(const float angle, const float speed = 0) {
        can_callback({.speed = speed, .angle = angle});
    }
};

static void check(const float actual, const float expected, const char* label, const float tolerance = 2e-5f) {
    if (!std::isfinite(actual) || std::fabs(actual - expected) > tolerance) {
        std::fprintf(stderr, "%s: actual %.9g expected %.9g\n", label, actual, expected);
        std::abort();
    }
    checks++;
}

struct Rig {
    Pid::Config yaw1_spd = {.kp = 1, .max_out = 20};
    Pid::Config yaw1_pos = {.kp = 5, .max_out = 12};
    Pid::Config yaw2_spd = {.kp = 1, .max_out = 20};
    Pid::Config yaw2_pos = {.kp = 10, .max_out = 12};
    Pid::Config pitch1_spd = {.kp = 1, .max_out = 20};
    Pid::Config pitch1_pos = {.kp = 10, .max_out = 12};
    Pid::Config pitch2_spd = {.kp = 1, .max_out = 20};
    Pid::Config pitch2_pos = {.kp = 10, .max_out = 12};

    TestMotor yaw1{true}, yaw2{false}, pitch1{true}, pitch2{true};
    GimbalFold gimbal;

    explicit Rig(const GimbalFold::Gravity pitch2_gravity = {.torque = 0, .phase = 0})
        : gimbal({yaw1, yaw2, pitch1, pitch2}, {.yaw_speed_pid_config = &yaw1_spd,
                                                .yaw_position_pid_config = &yaw1_pos,
                                                .pitch_speed_pid_config = &pitch2_spd,
                                                .pitch_position_pid_config = &pitch2_pos,
                                                .pitch_min = -0.3f,
                                                .pitch_max = 0.5f,
                                                .yaw2 = {&yaw2_spd, &yaw2_pos, -50 * deg, 50 * deg},
                                                .pitch1 = {&pitch1_spd, &pitch1_pos, -1.5f, 0},
                                                .pitch1_gravity = {.torque = 2.0f, .phase = 0.3f},
                                                .pitch2_gravity = pitch2_gravity}) {
        gimbal.set_enable(true);
        // 首两拍完成 check_motor 使能同步
        for (int i = 0; i < 2; i++) {
            now_us += 1000;
            yaw1.feedback(0, 0);
            yaw2.feedback(0, 0);
            pitch1.feedback(-0.5f, 0);
            pitch2.feedback(0, 0);
            gimbal.update(Quaternionf::Identity(), Vector3f::Zero());
        }
    }
};

// 验证小 yaw 经 pitch2 关节角旋转读取 IMU 角速度并隔离自身编码器速度噪声，而大 yaw 与 pitch2 保持原有编码器速度内环
static void test_yaw2_gyro_and_pitch2_encoder_preserved() {
    Rig r;
    r.yaw2_pos.kp = 0;
    r.pitch2_pos.kp = 0;
    r.gimbal.set_speed(0.7f, -0.2f);
    r.gimbal.set_chassis_wr(0.4f);

    for (int i = 0; i < 300; i++) {
        now_us += 1000;
        const float theta = -0.2f + 0.6f * static_cast<float>(i) / 299.0f;
        const Quaternionf q = euler_to_quaternion({.yaw = 0.6f, .pitch = theta});
        // 枪口绕 yaw2 轴绝对角速度 0.3 rad/s；故意给 gyro.y 注入大幅波动，验证 pitch2 不受 gyro.y 影响而只读 motor.pitch2.speed
        const float noisy_val = (i % 2 == 0) ? 85.0f : -95.0f;
        const Vector3f gyro(-std::sin(theta) * 0.3f, noisy_val, std::cos(theta) * 0.3f);
        r.yaw1.feedback(0.6f, 0.25f);
        r.yaw2.feedback(0.0f, noisy_val);
        r.pitch1.feedback(-0.4f, noisy_val);
        r.pitch2.feedback(theta, 0.15f);
        r.gimbal.update(q, gyro);

        check(r.yaw2.current_ref, 0.7f - 0.3f, "yaw2 uses projected IMU gyro z and ignores yaw2 encoder noise", 1e-4f);
        check(r.pitch2.torque_ref, -0.2f - 0.15f, "pitch2 preserves original motor.pitch2.speed inner loop", 1e-4f);
        check(r.pitch1.torque_ref, 0.0f, "pitch1 stays zero output");
    }

    // 枪口自旋分量 gyro.x 经 pitch2 关节角投影为 -sin(theta) 到小 yaw，pitch2 保持编码器反馈为零
    now_us += 1000;
    const float theta = 0.25f;
    r.gimbal.set_speed(0, 0);
    r.yaw1.feedback(0.0f, 0.0f);
    r.yaw2.feedback(0.0f, 50.0f);
    r.pitch2.feedback(theta, 0.0f);
    r.gimbal.update(Quaternionf::Identity(), Vector3f(1.0f, 5.0f, 0.0f));
    check(r.yaw2.current_ref, std::sin(theta), "barrel roll projects -sin(theta) into yaw2 speed feedback", 1e-5f);
    check(r.pitch2.torque_ref, 0.0f, "pitch2 ignores IMU gyro completely", 1e-5f);

    // 大 yaw 保持纯线性位置环（无平方根衰减）与纯电机编码器速度环（无小轴反作用力矩前馈）
    Rig big_check;
    big_check.yaw1_pos.max_out = 100.0f;
    big_check.yaw1_spd.max_out = 100.0f;
    big_check.gimbal.set_chassis_wr(0.5f);
    big_check.gimbal.set_angle(2.0f, 0.0f);
    now_us += 1000;
    big_check.yaw1.feedback(0.0f, 0.2f);
    big_check.yaw2.feedback(0.0f, 0.0f);
    big_check.pitch2.feedback(0.0f, 0.0f);
    big_check.gimbal.update(Quaternionf::Identity(), Vector3f(0.0f, 0.0f, -3.0f));
    check(big_check.yaw1.torque_ref, (5.0f * 2.0f - 0.5f) - 0.2f, "yaw1 linear position PID and encoder speed loop unchanged", 1e-4f);
}

// 验证小陀螺与大轴超调扰动下小轴世界系零稳态误差与即时陀螺仪阻尼，以及 ±50° 限位处平滑随动大轴
static void test_spintop_disturbance_and_yaw2_limit() {
    Rig r;
    r.gimbal.set_yaw_window(true);
    r.gimbal.set_angle(20 * deg, 0);

    // 小陀螺 + 大轴存在滞后转速误差时，只要枪口在世界系静止在目标上（gyro = 0），小轴输出严格为 0
    float base = 0;
    for (int i = 0; i < 200; i++) {
        now_us += 1000;
        base = Angle(base + 90 * rpm * 0.001f);
        const float q1 = -base + 5 * deg;
        const float q2 = 20 * deg - 5 * deg;
        r.yaw1.feedback(q1, -40 * rpm);
        r.yaw2.feedback(q2, 40 * rpm - 90 * rpm);
        r.pitch2.feedback(0, 0);
        r.gimbal.set_chassis_wr(90 * rpm);
        r.gimbal.update(euler_to_quaternion({.yaw = 20 * deg}), Vector3f::Zero());
        check(r.yaw2.current_ref, 0.0f, "yaw2 zero steady-state error when head is stationary in world despite yaw1 lag", 1e-5f);
    }

    // 大轴超调带动枪口产生 +2.5 rad/s 世界角速度瞬间，小轴当拍直接输出 -2.5 A 反向阻尼
    now_us += 1000;
    r.yaw1.feedback(0, 2.5f);
    r.yaw2.feedback(20 * deg, 0);
    r.pitch2.feedback(0, 0);
    r.gimbal.set_chassis_wr(0);
    r.gimbal.update(euler_to_quaternion({.yaw = 20 * deg}), Vector3f(0, 0, 2.5f));
    check(r.yaw2.current_ref, -2.5f, "yaw2 immediately damps world velocity disturbance from yaw1 overshoot", 1e-5f);

    // 掉头大角度越过小轴 ±50° 限位时，小轴贴在 +50° 随大轴（chassis_wr + yaw1.speed）同速运动，不反向拖拽大轴
    Rig slew;
    slew.gimbal.set_angle(120 * deg, 0);
    slew.gimbal.set_chassis_wr(1.0f);
    now_us += 1000;
    slew.yaw1.feedback(10 * deg, 4.0f);
    slew.yaw2.feedback(50 * deg, 0.0f);
    slew.pitch2.feedback(0, 0);
    slew.gimbal.update(euler_to_quaternion({.yaw = 60 * deg}), Vector3f(0, 0, 5.0f));
    check(slew.gimbal.yaw2.ref, 50 * deg, "yaw2 clamped at +50 deg");
    check(slew.yaw2.current_ref, 0.0f, "clamped yaw2 rides yaw1 smoothly with zero fighting current", 1e-5f);
}

// 验证倾斜底盘下小 yaw 世界航向解耦收敛，以及 pitch2 限位前馈/重力补偿/编码器闭环保持原样和掉线隔离
static void test_tilted_yaw2_and_offline_isolation() {
    const EulerAngles tilts[] = {{.yaw = 0.3f, .pitch = 0.15f, .roll = 0.25f},
                                 {.yaw = -0.8f, .pitch = -0.12f, .roll = -0.28f}};
    for (const auto& tilt : tilts) {
        Rig r;
        r.gimbal.set_yaw_window(true);
        const Quaternionf base_pose = euler_to_quaternion(tilt);
        float q2 = 0.05f;
        const float theta2 = 0.1f;
        const auto head_pose = [&]() {
            return base_pose * Quaternionf(Eigen::AngleAxisf(q2, Vector3f::UnitZ())) *
                   Quaternionf(Eigen::AngleAxisf(theta2, Vector3f::UnitY()));
        };

        now_us += 1000;
        r.yaw1.feedback(0, 0);
        r.yaw2.feedback(q2, 0);
        r.pitch2.feedback(theta2, 0);
        r.gimbal.update(head_pose(), Vector3f::Zero());

        const auto init_euler = quaternion_to_euler(head_pose());
        const float target_yaw = Angle(init_euler.yaw + 0.35f);
        r.gimbal.set_angle(target_yaw, init_euler.pitch);

        for (int i = 0; i < 2500; i++) {
            now_us += 1000;
            r.yaw1.feedback(0, 0);
            r.yaw2.feedback(q2, 0);
            r.pitch2.feedback(theta2, 0);
            r.gimbal.update(head_pose(), Vector3f::Zero());
            q2 += r.yaw2.current_ref * 0.001f;
        }
        const auto final_euler = quaternion_to_euler(head_pose());
        check(Angle(final_euler.yaw - target_yaw), 0.0f, "tilted yaw2 converges in world frame", 1e-4f);
    }

    // pitch2 越上界被钳位时 is_pitch2_limit 取消速度前馈（不论正负），只保留编码器速度闭环与重力补偿
    Rig g_rig({.torque = 1.5f, .phase = 0.0f});
    g_rig.gimbal.set_angle(0, 1.0f);
    g_rig.gimbal.set_speed(0, -0.8f);
    now_us += 1000;
    g_rig.yaw1.feedback(0, 0);
    g_rig.yaw2.feedback(0, 0);
    g_rig.pitch2.feedback(0.5f, 0.2f);
    g_rig.gimbal.update(euler_to_quaternion({.pitch = 0.5f}), Vector3f(0, 95.0f, 0));
    check(g_rig.gimbal.pitch.ecd.ref, 0.5f, "pitch2 clamped at max");
    check(g_rig.pitch2.torque_ref, -0.2f + 1.5f * std::cos(0.5f), "pitch2 uses motor.pitch2.speed, cancels ff on limit, ignores gyro.y", 1e-4f);

    Rig iso;
    iso.gimbal.set_angle(0.2f, 0.1f);
    for (int i = 0; i < 15; i++) {
        now_us += 1000;
        iso.yaw1.feedback(0);
        iso.yaw2.feedback(0);
        iso.gimbal.update(Quaternionf::Identity(), Vector3f::Zero());
    }
    check(iso.pitch2.torque_ref, 0.0f, "offline pitch2 outputs zero");
    check(iso.yaw2.current_ref, 2.0f, "yaw2 remains active when pitch2 is offline", 1e-4f);
}

int main() {
    test_yaw2_gyro_and_pitch2_encoder_preserved();
    test_spintop_disturbance_and_yaw2_limit();
    test_tilted_yaw2_and_offline_isolation();
    std::printf("GimbalFold yaw2 IMU gyro inner loop: %d checks passed\n", checks);
}
