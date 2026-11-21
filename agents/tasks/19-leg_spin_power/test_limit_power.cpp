// 轮腿轮端功率限幅回归：按日志帧注入电机反馈，检查实际下发力矩的功率不超过目标、未设置目标时保留平衡力矩
#include "lqr_fit.hpp"
#include "motor/dm8009.hpp"
#include "motor/m3508.hpp"
#include "robot/chassis/chassis_leg.hpp"
#include <cmath>
#include <cstdio>

static uint64_t now_cycles = 1000000;

void Timer::delay(const float) {}

float Timer::elapsed() const {
    return static_cast<float>(now_cycles - last_cycles) * 1e-6f;
}

float Timer::restart() {
    dt = elapsed();
    last_cycles = now_cycles;
    return dt;
}

static constexpr float WHEEL_KT = 0.2545043441f;
static constexpr float WHEEL_MAX_TORQUE = M3508::MAX_CURRENT * WHEEL_KT;
static constexpr float WHEEL_RADIUS = 0.06f;
static constexpr float WHEEL_REDUCTION = 16.875f;
static constexpr float WHEEL_RESISTANCE = 0.194f;

// 直接注入输出轴反馈，跳过 CAN 帧解析与标定偏移
class TestDm : public DM8009 {
  public:
    TestDm() : DM8009({.can = Can(1), .tx_id = 0x01, .rx_id = 0x11}) {}

    void feed(const float angle, const float speed) {
        Motor::can_callback({.speed = speed, .angle = angle});
        err_code = ENABLED;
    }
};

class TestWheel : public M3508 {
  public:
    explicit TestWheel(const uint32_t rx_id)
        : M3508({.can = Can(1), .rx_id = rx_id, .max_torque = M3508::MAX_CURRENT * WHEEL_KT, .reduction = WHEEL_REDUCTION}) {}

    void feed(const float speed) {
        Motor::can_callback({.current = 0.0f, .speed = speed});
    }
};

// 日志帧：杆方位角由关节角反推，速度与姿态直接取 SWD 记录
struct Frame {
    const char* name;
    float phi1[2], d_phi1[2], phi4[2], d_phi4[2];
    float v_wheel[2];
    float pitch, d_pitch, d_yaw_gyro;
    float d_yaw_ref, power_ref;
};

struct Result {
    float torque[2], speed[2];
};

static Pid::Config leg_length_pid = {.kp = 1750, .kd = 100, .max_out = 180};
static Pid::Config leg_angle_pid = {.kp = 140, .kd = 7, .max_out = 80};
static Pid::Config roll_pid_config = {.kp = 1.3f, .kd = 0.004f, .max_out = 0.4f};
static Pid::Config follow_pid_config = {.kp = 6.0f, .max_out = 180 * deg_s};

static Result run(const Frame& frame, const float power_ref) {
    TestDm left_front, left_back, right_back, right_front;
    TestWheel left_wheel(0x201), right_wheel(0x202);

    ChassisLeg chassis({left_front, left_back, right_back, right_front, left_wheel, right_wheel},
                       {
                           .mass = 22.75f,
                           .spring_force = 450.0f,
                           .wheel_radius = WHEEL_RADIUS,
                           .wheel_distance = 0.488f,
                           .active_link = 0.210f,
                           .passive_link = 0.248f,
                           .leg_min = 0.14f,
                           .leg_max = 0.33f,
                           .leg_length_pid = &leg_length_pid,
                           .leg_angle_pid = &leg_angle_pid,
                           .roll_pid = &roll_pid_config,
                           .follow_pid = &follow_pid_config,
                           .lqr_fit = LQR_FIT,
                           .wheel_resistance = WHEEL_RESISTANCE,
                       });

    // 关节角：chassis 内换算 phi1 = π - angle、phi4 = -angle，角速度取反
    left_back.feed(pi - frame.phi1[0], 0 - frame.d_phi1[0]);
    left_front.feed(0 - frame.phi4[0], 0 - frame.d_phi4[0]);
    right_front.feed(pi - frame.phi1[1], 0 - frame.d_phi1[1]);
    right_back.feed(0 - frame.phi4[1], 0 - frame.d_phi4[1]);
    left_wheel.feed(frame.v_wheel[0] / WHEEL_RADIUS * WHEEL_REDUCTION);
    right_wheel.feed(frame.v_wheel[1] / WHEEL_RADIUS * WHEEL_REDUCTION);

    chassis.set_enable(true);
    chassis.set_speed(0, frame.d_yaw_ref);
    chassis.set_leg_length(0.14f);
    chassis.set_power_ref(power_ref);
    now_cycles += 2000;
    chassis.update(Quaternionf(Eigen::AngleAxisf(frame.pitch, Vector3f::UnitY())), Vector3f(0, frame.d_pitch, frame.d_yaw_gyro));

    return {{left_wheel.torque_ref, right_wheel.torque_ref}, {left_wheel.speed, right_wheel.speed}};
}

// 实际下发力矩对应的轮端功率，W
static float power_of(const Result& result) {
    const float k = 0.75f * WHEEL_RESISTANCE / (WHEEL_KT * WHEEL_KT);
    float power = 0;
    for (int i = 0; i < 2; i++)
        power += result.torque[i] * result.speed[i] + k * result.torque[i] * result.torque[i];
    return power;
}

static int checks = 0;

static void expect(const bool condition, const char* name, const char* what) {
    checks++;
    if (!condition) {
        std::printf("FAIL %s: %s\n", name, what);
        std::exit(1);
    }
}

int main() {
    const Frame frames[] = {
        {"41.655", {3.1172f, 3.2353f}, {0.0110f, 1.3517f}, {0.0032f, 0.2621f}, {0.1429f, 1.2198f}, {-3.335f, 3.254f}, -0.0266f, 0.315f, 14.43f, 17.82f, 403.5f},
        {"41.784", {3.1999f, 3.3670f}, {0.4725f, 0.3187f}, {0.1279f, 0.3947f}, {0.8242f, 0.5385f}, {-3.329f, 3.315f}, 0.0030f, 0.606f, 15.11f, 17.86f, 399.2f},
        {"42.090", {3.2146f, 3.4175f}, {0.0989f, 0.8022f}, {0.2889f, 0.6321f}, {0.9341f, 1.4615f}, {-3.016f, 3.156f}, 0.0185f, 1.145f, 14.47f, 18.16f, 402.0f},
        {"42.299", {3.3283f, 3.7209f}, {1.6813f, 1.2637f}, {0.4855f, 1.3866f}, {1.7692f, 11.0659f}, {-2.707f, 3.245f}, 0.0371f, 1.428f, 13.54f, 19.95f, 387.5f},
    };

    for (const Frame& frame : frames) {
        const Result limited = run(frame, frame.power_ref);
        std::printf("%s ref=%.1f: torque=(%.2f, %.2f) power=%.0fW\n", frame.name, frame.power_ref, limited.torque[0], limited.torque[1], power_of(limited));
        for (int i = 0; i < 2; i++)
            expect(std::fabs(limited.torque[i]) <= WHEEL_MAX_TORQUE + 1e-4f, frame.name, "力矩超过电机上限");
        expect(power_of(limited) <= frame.power_ref + 2.0f, frame.name, "实际功率超过目标");

        // 未设置目标（0 W）：保持平衡力矩，不因驱动衰减而清零
        const Result unset = run(frame, 0);
        expect(std::fabs(unset.torque[0]) + std::fabs(unset.torque[1]) > 0.5f, frame.name, "未设置目标时平衡力矩被清零");
    }

    // 平衡力矩单项超过目标时衰减平衡部分：力矩小于未限幅值，实际功率不超目标
    const Result balance_limited = run(frames[2], 250);
    const Result balance_full = run(frames[2], 0);
    std::printf("42.090 ref=250: torque=(%.2f, %.2f) power=%.0fW；ref=0: torque=(%.2f, %.2f) power=%.0fW\n",
                balance_limited.torque[0], balance_limited.torque[1], power_of(balance_limited),
                balance_full.torque[0], balance_full.torque[1], power_of(balance_full));
    expect(power_of(balance_limited) <= 252.0f, "42.090/250W", "实际功率超过目标");
    expect(std::fabs(balance_limited.torque[1]) < std::fabs(balance_full.torque[1]) - 0.3f, "42.090/250W", "平衡力矩未衰减");

    // 大幅旋转指令：驱动被限幅到电机上限后仍参与分配，右轮出力与左轮不对称
    const Frame high_yaw = {"41.537/32rad", {3.1067f, 3.2330f}, {0.0300f, 1.0000f}, {0.0300f, 0.2500f}, {0.1000f, 1.0000f}, {-3.320f, 3.350f}, -0.0228f, 0.153f, 14.31f, 32.0f, 250.0f};
    const Result split = run(high_yaw, high_yaw.power_ref);
    expect(power_of(split) <= 252.0f, high_yaw.name, "实际功率超过目标");
    expect(std::fabs(split.torque[1]) > 1.0f, high_yaw.name, "驱动被过度衰减");

    std::printf("PASS: %d checks\n", checks);
}
