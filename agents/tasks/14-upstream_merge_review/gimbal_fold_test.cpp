// 真实控制代码差分回归，驱动反馈与计时由主机模拟
#include "baseline/gimbal_fold.hpp"
#include "robot/gimbal/gimbal_fold.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>

static uint64_t now_us = 1000000;
static int comparisons = 0;

float Timer::elapsed() const {
    return static_cast<float>(now_us - last_cycles) * 1e-6f;
}

float Timer::restart() {
    const float dt = elapsed();
    last_cycles = now_us;
    return dt;
}

class TestMotor : public Motor {
  private:
    const bool is_torque;

  public:
    explicit TestMotor(const bool is_torque)
        : Motor({.can = Can(1), .timeout = 0.005f}, 20, 6, 1), is_torque(is_torque) {}

    void set_output(const float output) override {
        if (is_torque)
            set_torque(output);
        else
            set_current(output);
    }

    void feedback(const float position, const float speed) {
        can_callback({.speed = speed, .angle = position});
    }
};

static void compare(const float actual, const float expected) {
    if (!std::isfinite(actual) || !std::isfinite(expected) || std::fabs(actual - expected) > 2e-5f) {
        std::fprintf(stderr, "comparison %d: actual %.9g expected %.9g\n", comparisons, actual, expected);
        std::abort();
    }
    comparisons++;
}

static void compare_axis(const auto& actual, const auto& baseline) {
    compare(actual.ecd.ref, baseline.ecd.ref);
    compare(actual.ecd.measure, baseline.ecd.measure);
    compare(actual.imu.ref, baseline.imu.ref);
    compare(actual.imu.measure, baseline.imu.measure);
    compare(actual.imu_minus_ecd, baseline.imu_minus_ecd);
}

int main() {
    const Pid::Config speed = {.kp = 2, .ki = 0.4f, .kd = 0.002f, .max_i = 1, .max_out = 5, .err_fc = 30};
    const Pid::Config position = {.kp = 6, .ki = 0.2f, .max_i = 0.4f, .max_out = 8};
    TestMotor motors[4] = {TestMotor(true), TestMotor(false), TestMotor(true), TestMotor(true)};
    TestMotor old_motors[4] = {TestMotor(true), TestMotor(false), TestMotor(true), TestMotor(true)};
    Angle last_angle[4];
    GimbalFold fold({motors[0], motors[1], motors[2], motors[3]}, {
                                                                      .yaw_speed_pid_config = &speed,
                                                                      .yaw_position_pid_config = &position,
                                                                      .pitch_speed_pid_config = &speed,
                                                                      .pitch_position_pid_config = &position,
                                                                      .pitch_min = -0.3f,
                                                                      .pitch_max = 0.5f,
                                                                      .yaw2 = {&speed, &position, -0.7f, 0.7f},
                                                                      .pitch1 = {&speed, &position, -1.6f, 0},
                                                                      .pitch1_gravity = {.torque = 2, .phase = 0.5f},
                                                                      .pitch2_gravity = {.torque = 1, .phase = -0.2f},
                                                                  });
    BaselineGimbalFold baseline({old_motors[0], old_motors[1], old_motors[2], old_motors[3]}, {
                                                                                                  .gimbal = {&speed, &position, &speed, &position, -0.3f, 0.5f},
                                                                                                  .yaw2 = {&speed, &position, -0.7f, 0.7f},
                                                                                                  .pitch1 = {&speed, &position, -1.6f, 0},
                                                                                                  .pitch1_gravity = {.torque = 2, .phase = 0.5f},
                                                                                                  .pitch2_gravity = {.torque = 1, .phase = -0.2f},
                                                                                              });

    for (int i = 0; i < 6000; i++) {
        const bool is_enable = i % 701 > 35;
        const bool is_ecd = i / 83 % 2 == 0;
        const float chassis_wr = std::sin(i * 0.02f);
        fold.set_enable(is_enable);
        baseline.set_enable(is_enable);
        fold.set_mode(is_ecd ? GimbalFold::ECD_MODE : GimbalFold::IMU_MODE);
        baseline.set_mode(is_ecd ? BaselineGimbalFold::ECD_MODE : BaselineGimbalFold::IMU_MODE);
        fold.set_chassis_wr(chassis_wr);
        baseline.set_chassis_wr(chassis_wr);
        if (i % 17 == 0) {
            fold.set_speed(4 * chassis_wr, -2 * chassis_wr);
            baseline.set_speed(4 * chassis_wr, -2 * chassis_wr);
        }
        if (i % 37 == 0) {
            fold.set_angle(3.5f * chassis_wr, 0.9f * chassis_wr);
            baseline.set_angle(3.5f * chassis_wr, 0.9f * chassis_wr);
            fold.set_pitch1_angle(-0.8f - chassis_wr);
            baseline.set_pitch1_angle(-0.8f - chassis_wr);
        }
        if (i % 131 == 0) {
            fold.turn_yaw(pi);
            baseline.yaw.ecd.ref += pi;
            baseline.yaw.imu.ref += pi;
        }

        now_us += i % 101 == 0 ? 50000 : (i % 97 == 0 ? 50 : 1000);
        for (int axis = 0; axis < 4; axis++) {
            if ((i + axis * 49) % 245 >= 25) {
                const float angle = 3.3f * std::sin(i * 0.003f + axis);
                const float rate = 5 * std::cos(i * 0.03f + axis);
                motors[axis].feedback(angle, rate);
                old_motors[axis].feedback(angle, rate);
                last_angle[axis] = angle;
            }
        }
        const Angle imu_yaw = 3.4f * std::sin(i * 0.012f);
        const Angle imu_pitch = 1.1f * std::cos(i * 0.033f);
        fold.update(imu_yaw, imu_pitch);
        baseline.update(imu_yaw, imu_pitch);

        compare(fold.yaw1.measure, last_angle[0]);
        compare(fold.yaw2.measure, last_angle[1]);
        compare(fold.pitch1.measure, last_angle[2]);
        compare(fold.pitch.ecd.measure, last_angle[3]);

        compare_axis(fold.yaw, baseline.yaw);
        compare_axis(fold.pitch, baseline.pitch);
        compare(fold.yaw1.ref, baseline.yaw1.ref);
        compare(fold.yaw1.measure, baseline.yaw1.measure);
        compare(fold.yaw2.ref, baseline.yaw2.ref);
        compare(fold.yaw2.measure, baseline.yaw2.measure);
        compare(fold.pitch1.ref, baseline.pitch1.ref);
        compare(fold.pitch1.measure, baseline.pitch1.measure);
        for (int axis = 0; axis < 4; axis++) {
            compare(motors[axis].current_ref, old_motors[axis].current_ref);
            compare(motors[axis].torque_ref, old_motors[axis].torque_ref);
        }
    }
    std::printf("PASS: 6000 periods, %d comparisons\n", comparisons);
}
