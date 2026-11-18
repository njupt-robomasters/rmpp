// 主机差分回归：独立云台与重构前实现在相同反馈、指令和时钟下逐周期比较
#include "baseline/gimbal_classic.hpp"
#include "baseline/gimbal_dual_yaw.hpp"
#include "robot/gimbal/gimbal_classic.hpp"
#include "robot/gimbal/gimbal_dual_yaw.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <type_traits>

static uint64_t now_us = 1000000;
static int comparisons = 0;

void Timer::delay(float) {}

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
    explicit TestMotor(const bool is_torque = false)
        : Motor({.can = Can(1), .timeout = 0.005f}, 20, 6, 1), is_torque(is_torque) {}

    void set_output(const float output) override {
        if (is_torque)
            set_torque(output);
        else
            set_current(output);
    }

    void feedback(const float position, const float speed) {
        raw_position = position;
        raw_speed = speed;
        can_callback();
    }
};

static void compare(const float actual, const float expected) {
    if (!std::isfinite(actual) || !std::isfinite(expected) || std::fabs(actual - expected) > 2e-5f) {
        std::fprintf(stderr, "comparison %d: actual %.9g expected %.9g\n", comparisons, actual, expected);
        std::abort();
    }
    comparisons++;
}

static void compare_axis(const auto& actual, const auto& expected) {
    compare(actual.ecd.ref, expected.ecd.ref);
    compare(actual.ecd.measure, expected.ecd.measure);
    compare(actual.imu.ref, expected.imu.ref);
    compare(actual.imu.measure, expected.imu.measure);
    compare(actual.imu_minus_ecd, expected.imu_minus_ecd);
}

template <class GimbalType, class BaselineType, bool IS_DUAL>
static void run_scenario() {
    const Pid::Config speed_pid = {.kp = 1.7f, .ki = 0.4f, .kd = 0.002f, .max_i = 1, .max_out = 5, .err_fc = 30, .d_fc = 20};
    const Pid::Config position_pid = {.kp = 3, .ki = 0.2f, .kd = 0.003f, .max_i = 0.4f, .max_out = 4};
    TestMotor yaw(IS_DUAL), yaw2, pitch;
    TestMotor old_yaw(IS_DUAL), old_yaw2, old_pitch;
    const auto motor = [&]() {
        if constexpr (IS_DUAL)
            return GimbalType({yaw, yaw2, pitch}, {&speed_pid, &position_pid, &speed_pid, &position_pid, -0.3f, 0.5f});
        else
            return GimbalType({yaw, pitch}, {&speed_pid, &position_pid, &speed_pid, &position_pid, -0.3f, 0.5f});
    };
    const auto old_motor = [&]() {
        if constexpr (IS_DUAL)
            return BaselineType({old_yaw, old_yaw2, old_pitch}, {&speed_pid, &position_pid, &speed_pid, &position_pid, -0.3f, 0.5f});
        else
            return BaselineType({old_yaw, old_pitch}, {&speed_pid, &position_pid, &speed_pid, &position_pid, -0.3f, 0.5f});
    };
    auto gimbal = motor();
    auto baseline = old_motor();
    for (int i = 0; i < 6000; i++) {
        const int phase = i % 1200;
        const bool is_enable = phase >= 70 && (phase < 970 || phase >= 1050);
        const bool is_ecd = (i / 83) % 2 == 0;
        gimbal.set_enable(is_enable);
        baseline.set_enable(is_enable);
        gimbal.set_mode(is_ecd ? GimbalType::ECD_MODE : GimbalType::IMU_MODE);
        baseline.set_mode(is_ecd ? BaselineType::ECD_MODE : BaselineType::IMU_MODE);
        gimbal.set_chassis_wr(0.2f * std::sin(i * 0.03f));
        baseline.set_chassis_wr(0.2f * std::sin(i * 0.03f));
        if (i % 19 == 0) {
            const float yaw_speed = 3 * std::sin(i * 0.1f);
            const float pitch_speed = std::cos(i * 0.1f);
            gimbal.set_speed(yaw_speed, pitch_speed);
            baseline.set_speed(yaw_speed, pitch_speed);
        }
        if (i % 31 == 0) {
            const Angle yaw_ref = 3.4f * std::sin(i * 0.02f);
            const Angle pitch_ref = std::cos(i * 0.05f);
            gimbal.set_angle(yaw_ref, pitch_ref);
            baseline.set_angle(yaw_ref, pitch_ref);
        }
        // 长间隔触发 dt 上限和超时，短间隔触发 dt 下限；分别中断两侧 yaw 与 pitch 反馈
        now_us += i % 101 == 0 ? 50000 : (i % 97 == 0 ? 0 : (i % 89 == 0 ? 50 : 1000));
        const float position = 3.3f * std::sin(i * 0.003f);
        const float speed = std::cos(i * 0.03f);
        if (phase < 380 || phase >= 560) {
            yaw.feedback(position, speed);
            old_yaw.feedback(position, speed);
        }
        if (phase < 500 || phase >= 640) {
            yaw2.feedback(-position * 0.3f, -speed * 0.4f);
            old_yaw2.feedback(-position * 0.3f, -speed * 0.4f);
        }
        if (phase < 720 || phase >= 840) {
            pitch.feedback(0.2f * speed, speed * 0.1f);
            old_pitch.feedback(0.2f * speed, speed * 0.1f);
        }
        const Angle imu_yaw = position + 0.4f;
        const Angle imu_pitch = 0.2f * speed - 0.1f;
        gimbal.update(imu_yaw, imu_pitch);
        baseline.update(imu_yaw, imu_pitch);
        compare_axis(gimbal.yaw, baseline.yaw);
        compare_axis(gimbal.pitch, baseline.pitch);
        compare(yaw.current_ref, old_yaw.current_ref);
        compare(yaw.torque_ref, old_yaw.torque_ref);
        compare(pitch.current_ref, old_pitch.current_ref);
        assert(float(gimbal.pitch.ecd.ref) >= -0.3f && float(gimbal.pitch.ecd.ref) <= 0.5f);
        assert((is_enable && yaw.is_connect) || yaw.current_ref == 0);
        assert((is_enable && pitch.is_connect) || pitch.current_ref == 0);
        if constexpr (IS_DUAL) {
            compare(yaw2.current_ref, old_yaw2.current_ref);
            compare(gimbal.yaw1.ref, baseline.yaw1.ref);
            compare(gimbal.yaw1.measure, baseline.yaw1.measure);
            compare(gimbal.yaw2.ref, baseline.yaw2.ref);
            compare(gimbal.yaw2.measure, baseline.yaw2.measure);
            assert((is_enable && yaw2.is_connect) || yaw2.current_ref == 0);
        }
    }
}

int main() {
    static_assert(!std::is_polymorphic_v<GimbalClassic>);
    static_assert(!std::is_polymorphic_v<GimbalDualYaw>);
    run_scenario<GimbalClassic, BaselineGimbalClassic, false>();
    run_scenario<GimbalDualYaw, BaselineGimbalDualYaw, true>();
    std::printf("PASS: 12000 cycles, %d comparisons\n", comparisons);
}
