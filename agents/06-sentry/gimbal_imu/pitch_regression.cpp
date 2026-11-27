// 与合并前真实 GimbalFold 逐拍对照 pitch2，覆盖前馈、限位、掉线、恢复和周期抖动
#include "baseline_gimbal_fold.hpp"
#include "robot/gimbal_fold.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>

static uint64_t now_us = 1000000;
static int checks = 0;

float Timer::elapsed() const {
    return static_cast<float>(now_us - last_cycles) * 1e-6f;
}

float Timer::restart() {
    dt = elapsed();
    last_cycles = now_us;
    return dt;
}

static void near(const float actual, const float expected, const int step, const char* label) {
    if (!std::isfinite(actual) || std::fabs(actual - expected) > 2e-5f) {
        std::fprintf(stderr, "pitch regression step %d %s: %.9g != %.9g\n", step, label, actual, expected);
        std::abort();
    }
    checks++;
}

class TestMotor : public Motor {
  public:
    TestMotor() : Motor({.can = Can(1), .timeout = 0.01f}, 7.5f, 7, 1) {}

    void set_output(const float output) override {
        set_torque(output);
    }

    void feedback(const float angle, const float speed) {
        can_callback({.speed = speed, .angle = angle});
    }
};

int main() {
    const Pid::Config speed = {.kp = 0.50558f, .ki = 2, .kd = 0.02f, .ff = -1.55f, .max_i = 0.4f, .max_out = 5, .err_fc = 10};
    const Pid::Config angle = {.kp = 20, .max_out = 720 * deg_s};
    const Pid::Config zero = {};
    TestMotor a1, a2, a3, a4, b1, b2, b3, b4;
    GimbalFold current({a1, a2, a3, a4}, {
                                             .yaw1_angle_pid_config = &zero,
                                             .yaw1_speed_pid_config = &zero,
                                             .yaw2_angle_pid_config = &zero,
                                             .yaw2_speed_pid_config = &zero,
                                             .yaw2_min = -50 * deg,
                                             .yaw2_max = 50 * deg,
                                             .pitch_angle_pid_config = &angle,
                                             .pitch_speed_pid_config = &speed,
                                             .pitch_min = -0.1f,
                                             .pitch_max = 0.314f,
                                             .pitch1_position_pid_config = &zero,
                                             .pitch1_speed_pid_config = &zero,
                                             .pitch2_gravity = {.torque = 0.8f, .phase = 0.2f},
                                         });
    BaselineGimbalFold baseline({b1, b2, b3, b4}, {
                                                      .yaw_speed_pid_config = &zero,
                                                      .yaw_position_pid_config = &zero,
                                                      .pitch_speed_pid_config = &speed,
                                                      .pitch_position_pid_config = &angle,
                                                      .pitch_min = -0.1f,
                                                      .pitch_max = 0.314f,
                                                      .yaw2 = {&zero, &zero, -50 * deg, 50 * deg},
                                                      .pitch1 = {&zero, &zero, 0, 0},
                                                      .pitch2_gravity = {.torque = 0.8f, .phase = 0.2f},
                                                  });
    const uint32_t intervals[] = {1000, 1300, 700, 50, 13000};
    for (int i = 0; i < 12000; i++) {
        now_us += intervals[i % 5];
        const bool enabled = i % 137 > 8;
        current.set_enable(enabled);
        baseline.set_enable(enabled);
        const float ecd = 0.32f * std::sin(i * 0.013f) + 0.1f;
        const float world_pitch = ecd + 0.12f * std::sin(i * 0.003f);
        const float rate = 1.2f * std::cos(i * 0.007f);
        for (TestMotor* motor : {&a1, &a2, &a3, &b1, &b2, &b3})
            motor->feedback(0, 0);
        if (i % 233 > 20) {
            a4.feedback(ecd, rate);
            b4.feedback(ecd, rate);
        }
        if (i % 17 == 0) {
            const float target = 0.65f * std::sin(i * 0.02f);
            current.set_angle(0, target);
            baseline.set_angle(0, target);
        }
        if (i % 3 == 0) {
            const float target_rate = 0.8f * std::cos(i * 0.011f);
            current.set_speed(0, target_rate);
            baseline.set_speed(0, target_rate);
        }
        const Quaternionf q = euler_to_quaternion({.yaw = 0.4f, .pitch = world_pitch, .roll = 0.1f});
        const Vector3f gyro(5, i % 2 ? -90 : 90, 4);
        current.update(q, gyro);
        baseline.update(q, gyro);
        near(a4.torque_ref, b4.torque_ref, i, "torque");
        near(a4.current_ref, b4.current_ref, i, "current");
        near(current.pitch.ref, baseline.pitch.imu.ref, i, "world reference");
        near(current.pitch.ecd_ref, baseline.pitch.ecd.ref, i, "encoder reference");
        near(current.pitch.ecd_measure, baseline.pitch.ecd.measure, i, "encoder feedback");
        near(a3.torque_ref, 0, i, "pitch1 disabled");
    }
    std::printf("Pitch2 baseline regression: %d checks passed\n", checks);
}
