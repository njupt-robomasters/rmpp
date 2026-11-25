// 实际 Motor、底盘和 PID 的离线解算与输出回归，仅替换时钟
#include "robot/chassis_matrix.hpp"
#include "robot/chassis_steer.hpp"
#include <bit>
#include <cassert>
#include <cstdio>

static uint64_t now_cycles = 1000000;
static int checks = 0;

float Timer::elapsed() const {
    return static_cast<float>(now_cycles - last_cycles) * 1e-6f;
}

float Timer::restart() {
    dt = elapsed();
    last_cycles = now_cycles;
    return dt;
}

class TestMotor : public Motor {
  public:
    TestMotor() : Motor({.can = Can(1)}, 10, 2, 1) {}

    void feed(const float speed = 0, const float angle = 0) {
        can_callback({.speed = speed, .angle = angle});
    }
};

class MatrixProbe : public ChassisMatrix {
  public:
    MatrixProbe(const std::array<Motor*, 4>& wheel, const Config& config) : ChassisMatrix(wheel, config) {}

    Vector3f measure() const {
        return {vx.measure, vy.measure, vz.measure};
    }

  private:
    void control(float) override {}
};

class SteerProbe : public ChassisSteer {
  public:
    SteerProbe(const std::array<Motor*, 4>& wheel, const std::array<Motor*, 4>& steer, const Config& config) : ChassisSteer(wheel, steer, config) {}

    Vector3f measure() const {
        return {vx.measure, vy.measure, vz.measure};
    }

  private:
    void control(float) override {}
};

static void check(const bool result) {
    assert(result);
    checks++;
}

static void check_matrix(const float (*kinematics)[3]) {
    TestMotor w[4];
    MatrixProbe chassis({&w[0], &w[1], &w[2], &w[3]}, {.kinematics = kinematics, .chassis_radius = 0.4f, .wheel_radius = 0.1f});
    for (const Vector3f& expected : {Vector3f(1, 0, 0), Vector3f(0, 1, 0), Vector3f(0, 0, 1), Vector3f(0.7f, -0.4f, 0.8f)}) {
        const Vector4f v = chassis.backward_speed(expected);
        for (unsigned mask = 0; mask < 16; mask++) {
            now_cycles += 200000;
            for (int i = 0; i < 4; i++)
                if (mask & (1u << i))
                    w[i].feed(v[i] / 0.1f);
            chassis.update();
            if (std::popcount(mask) >= 3)
                check((chassis.measure() - expected).norm() < 1e-5f);
            else
                check(chassis.measure().array().isNaN().all());
        }
    }
}

static void check_steer() {
    TestMotor w[4], s[4];
    SteerProbe chassis({&w[0], &w[1], &w[2], &w[3]}, {&s[0], &s[1], &s[2], &s[3]}, {.chassis_radius = 0.4f, .wheel_radius = 0.1f});
    const float tangent[] = {135 * deg, -135 * deg, -45 * deg, 45 * deg};
    for (const Vector3f& expected : {Vector3f(1, 0, 0), Vector3f(0, 1, 0), Vector3f(0, 0, 1), Vector3f(0.7f, -0.4f, 0.8f)}) {
        for (unsigned wheel_mask = 0; wheel_mask < 16; wheel_mask++) {
            for (unsigned steer_mask = 0; steer_mask < 16; steer_mask++) {
                now_cycles += 200000;
                for (int i = 0; i < 4; i++) {
                    const float x = expected.x() + expected.z() * std::cos(tangent[i]);
                    const float y = expected.y() + expected.z() * std::sin(tangent[i]);
                    if (wheel_mask & (1u << i))
                        w[i].feed(std::hypot(x, y) / 0.1f);
                    if (steer_mask & (1u << i))
                        s[i].feed(0, std::atan2(y, x));
                }
                chassis.update();
                if (std::popcount(wheel_mask & steer_mask) >= 2)
                    check((chassis.measure() - expected).norm() < 1e-5f);
                else
                    check(chassis.measure().array().isNaN().all());
            }
        }
    }
}

static const Pid::Config pid{.kp = 1, .ki = 1, .max_i = 2, .max_out = 10};

template <class Chassis>
static void check_steer_control(const bool is_torque) {
    TestMotor w[4], s[4];
    Chassis chassis({&w[0], &w[1], &w[2], &w[3]}, {&s[0], &s[1], &s[2], &s[3]},
                    {.chassis_radius = 0.4f, .wheel_radius = 0.1f, .speed_pid_config = &pid, .steer_pid_config = &pid});
    chassis.set_enable(true);
    chassis.set_power_ref(1000);
    chassis.set_speed(1, 0.5f, 0);
    // 先全在线建立输出，再覆盖全部轮/舵在线组合，最后全在线恢复
    for (unsigned state = 0; state < 258; state++) {
        const unsigned mask = state == 0 || state == 257 ? 255 : state - 1;
        now_cycles += 200000;
        for (int i = 0; i < 4; i++) {
            if (mask & (1u << i))
                w[i].feed();
            if (mask & (1u << (i + 4)))
                s[i].feed();
        }
        chassis.update();
        const unsigned valid = mask & (mask >> 4) & 15;
        for (int i = 0; i < 4; i++) {
            const bool can_drive = (valid & (1u << i)) && (!is_torque || std::popcount(valid) >= 2);
            check(std::isfinite(w[i].current_ref));
            check(can_drive ? std::fabs(w[i].current_ref) > 0 : w[i].current_ref == 0);
            check(std::isfinite(s[i].current_ref));
            check((mask & (1u << (i + 4))) ? std::fabs(s[i].current_ref) > 0 : s[i].current_ref == 0);
        }
    }
}

template <class Chassis>
static void check_matrix_control(const bool is_torque) {
    TestMotor w[4];
    Chassis chassis({&w[0], &w[1], &w[2], &w[3]},
                    {.kinematics = ChassisMatrix::OMNI, .chassis_radius = 0.4f, .wheel_radius = 0.1f, .speed_pid_config = &pid});
    chassis.set_enable(true);
    chassis.set_power_ref(1000);
    chassis.set_speed(1, 0.5f, 0);
    for (unsigned state = 0; state < 18; state++) {
        const unsigned mask = state == 0 || state == 17 ? 15 : state - 1;
        now_cycles += 200000;
        for (int i = 0; i < 4; i++)
            if (mask & (1u << i))
                w[i].feed();
        chassis.update();
        for (int i = 0; i < 4; i++) {
            const bool can_drive = (mask & (1u << i)) && (!is_torque || std::popcount(mask) >= 3);
            check(std::isfinite(w[i].current_ref));
            check(can_drive ? std::fabs(w[i].current_ref) > 0 : w[i].current_ref == 0);
        }
    }
}

int main() {
    check_matrix(ChassisMatrix::OMNI);
    const float mecanum[4][3] = {{-1, 1, std::sqrt(2.0f)}, {-1, -1, std::sqrt(2.0f)}, {1, -1, std::sqrt(2.0f)}, {1, 1, std::sqrt(2.0f)}};
    check_matrix(mecanum);
    check_steer();
    check_steer_control<ChassisSteerSpeed>(false);
    check_steer_control<ChassisSteerTorque>(true);
    check_matrix_control<ChassisMatrixSpeed>(false);
    check_matrix_control<ChassisMatrixTorque>(true);
    std::printf("PASS: %d chassis offline checks\n", checks);
}
