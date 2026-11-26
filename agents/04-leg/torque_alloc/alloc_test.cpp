// 轮腿力矩分配回归：超限时姿态组优先、运动组共同退让，四路不超过电机与关节上限；未超限时原样下发
#include "robot/chassis_leg_lqr.hpp"
#include "motor/dm8009.hpp"
#include "motor/m3508.hpp"
#include "robot/chassis_leg.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>

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
static constexpr float JOINT_MAX_TORQUE = 40.0f;
static constexpr float WHEEL_RADIUS = 0.06f;
static constexpr float WHEEL_REDUCTION = 16.875f;
static constexpr float WHEEL_DISTANCE = 0.488f;
static constexpr float ACTIVE_LINK = 0.210f;
static constexpr float PASSIVE_LINK = 0.248f;
static constexpr float LEG_LENGTH = 0.18f;
static constexpr float WHEEL_WORK_SPEED = 40.0f;
static constexpr float MASS = 22.75f;
static constexpr float SPRING_FORCE = 450.0f;
static constexpr float THETA_REF = 0.07f;

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
        : M3508({.can = Can(1), .rx_id = rx_id, .max_torque = WHEEL_MAX_TORQUE, .reduction = WHEEL_REDUCTION}) {}

    void feed(const float speed) {
        Motor::can_callback({.current = 0.0f, .speed = speed * WHEEL_REDUCTION});
    }
};

static Pid::Config leg_length_pid = {.kp = 1750, .kd = 100, .max_out = 180};
static Pid::Config leg_angle_pid = {.kp = 140, .kd = 7, .max_out = 80};
static Pid::Config roll_pid_config = {.kp = 1.3f, .kd = 0.004f, .max_out = 0.4f};

// 单个工况的下发力矩，N·m；关节按 left_back、left_front、right_back、right_front
struct Result {
    float wheel[2], joint_a[2], joint_e[2];
};

// 腿直立：phi0 由相对腿角换算，phi1、phi4 取正装配分支
static Result run(const float rel_theta, const float pitch, const float vx, const float wr) {
    TestDm left_front, left_back, right_back, right_front;
    TestWheel left_wheel(0x201), right_wheel(0x202);

    ChassisLeg chassis({left_front, left_back, right_back, right_front, left_wheel, right_wheel},
                       {
                           .mass = MASS,
                           .spring_force = SPRING_FORCE,
                           .wheel_radius = WHEEL_RADIUS,
                           .wheel_distance = WHEEL_DISTANCE,
                           .active_link = ACTIVE_LINK,
                           .passive_link = PASSIVE_LINK,
                           .leg_min = 0.14f,
                           .leg_max = 0.33f,
                           .leg_length_pid = &leg_length_pid,
                           .leg_angle_pid = &leg_angle_pid,
                           .roll_pid = &roll_pid_config,
                           .lqr_fit = LQR_FIT,
                           .lqr_posture_fit = LQR_POSTURE_FIT,
                           .lqr_r = LQR_R,
                           .wheel_work_speed = WHEEL_WORK_SPEED,
                       });

    chassis.set_enable(true);
    chassis.set_allow_rescue(true);
    chassis.update(Quaternionf::Identity(), Vector3f::Zero());
    chassis.set_allow_rescue(false);

    // 五连杆反解：三角形 O-B-C 中 OB = a、BC = b、OC = 腿长，phi1 = phi0 + beta、phi4 = phi0 - beta
    const float beta = std::acos((ACTIVE_LINK * ACTIVE_LINK + LEG_LENGTH * LEG_LENGTH - PASSIVE_LINK * PASSIVE_LINK) / (2 * ACTIVE_LINK * LEG_LENGTH));

    // 关节角：chassis 内换算 phi1 = π - angle、phi4 = -angle，角速度取反
    const float phi0_left = pi / 2 - rel_theta, phi0_right = pi / 2 + rel_theta;
    left_back.feed(pi - (phi0_left + beta), 0);
    left_front.feed(0 - (phi0_left - beta), 0);
    right_front.feed(pi - (phi0_right + beta), 0);
    right_back.feed(0 - (phi0_right - beta), 0);
    left_wheel.feed(0);
    right_wheel.feed(0);

    chassis.set_speed(vx, wr);
    chassis.set_leg_length(LEG_LENGTH);
    now_cycles += 2000;
    chassis.update(Quaternionf(Eigen::AngleAxisf(pitch, Vector3f::UnitY())), Vector3f::Zero());

    return {{left_wheel.torque_ref, right_wheel.torque_ref},
            {left_back.torque_ref, right_front.torque_ref},
            {left_front.torque_ref, right_back.torque_ref}};
}

// 拟合表在双腿等长时的增益，N·m 每单位误差
static float gain(const int output, const int state) {
    const float terms[6] = {1, LEG_LENGTH, LEG_LENGTH, LEG_LENGTH * LEG_LENGTH, LEG_LENGTH * LEG_LENGTH, LEG_LENGTH * LEG_LENGTH};
    float k = 0;
    for (int n = 0; n < 6; n++)
        k += LQR_FIT[output][state][n] * terms[n];
    return k;
}

static float target_share(const float vx, const float wr) {
    const float rim = std::fabs(vx) + std::fabs(wr) * WHEEL_DISTANCE / 2;
    const float travel = WHEEL_WORK_SPEED * WHEEL_RADIUS;
    return rim > travel ? travel / rim : 1;
}

// 轴向力在两关节上形成的反向力矩幅值，与 ChassisLeg::calc_axial_torque() 相同
static float axial_torque(const float theta) {
    const float cos_ocb = (LEG_LENGTH * LEG_LENGTH + PASSIVE_LINK * PASSIVE_LINK - ACTIVE_LINK * ACTIVE_LINK) / (2 * LEG_LENGTH * PASSIVE_LINK);
    const float tan_ocb = std::sqrt(1 - cos_ocb * cos_ocb) / cos_ocb;

    // 平衡腿长误差为零时，腿长 PD 输出为零，只剩重力、弹簧与关节角补偿
    const float phi_obc = std::acos((ACTIVE_LINK * ACTIVE_LINK + PASSIVE_LINK * PASSIVE_LINK - LEG_LENGTH * LEG_LENGTH) / (2 * ACTIVE_LINK * PASSIVE_LINK));
    const float phi_bod = 2 * std::acos((ACTIVE_LINK * ACTIVE_LINK + LEG_LENGTH * LEG_LENGTH - PASSIVE_LINK * PASSIVE_LINK) / (2 * ACTIVE_LINK * LEG_LENGTH));
    const float fg = std::max(MASS * 9.8f / 2 * std::cos(theta), 0.0f);
    const float fs = std::fabs(0.5f * SPRING_FORCE * std::sin(phi_obc) * std::sin(pi - 0.5f * phi_bod - phi_obc));
    const float fn = fg - fs;
    return 0.5f * fn * LEG_LENGTH * tan_ocb;
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
    // 未超限：小旋转目标，四路原样下发
    {
        const float wr = 0.5f;
        const Result result = run(THETA_REF, 0, 0, wr);
        for (int i = 0; i < 2; i++) {
            const float wheel = gain(i, 3) * wr;
            const float t1 = axial_torque(THETA_REF);
            const float t2 = (i == 0 ? 1.0f : -1.0f) * 0.5f * gain(2 + i, 3) * wr;
            std::printf("pass wheel[%d]=%.3f raw=%.3f joint=(%.3f, %.3f) expect=(%.3f, %.3f)\n",
                        i, result.wheel[i], wheel, result.joint_a[i], result.joint_e[i], t1 + t2, -t1 + t2);
            expect(std::fabs(result.wheel[i] - wheel) < 1e-3f, "小旋转", "轮端未原样下发");
            expect(std::fabs(result.joint_a[i] - (t1 + t2)) < 1e-3f, "小旋转", "关节 A 未原样下发");
            expect(std::fabs(result.joint_e[i] - (-t1 + t2)) < 1e-3f, "小旋转", "关节 E 未原样下发");
        }
    }

    // 目标限速：直行 5 m/s 先按 2.4 m/s 缩放，分配前误差对应 2.4 m/s
    {
        const float vx = 5;
        const Result result = run(THETA_REF, 0, vx, 0);
        const float wheel = gain(0, 1) * WHEEL_WORK_SPEED * WHEEL_RADIUS;
        std::printf("target wheel=%.3f expected=%.3f raw5=%.3f\n", result.wheel[0], wheel, gain(0, 1) * vx);
        expect(std::fabs(result.wheel[0] - wheel) < 1e-3f, "目标限速", "目标未按轮子额定转速缩放");
        expect(std::fabs(result.wheel[0]) < std::fabs(gain(0, 1) * vx) - 0.5f, "目标限速", "缩放后仍按原目标出力");
    }

    // 超限：原地高速旋转，轮端到限，腿摆同比让位，两关节不超过 40 N·m
    {
        const float wr = 32;
        const Result result = run(THETA_REF, 0, 0, wr);
        const float error = target_share(0, wr) * wr;
        const float raw_wheel = gain(0, 3) * error;
        const float raw_swing = gain(2, 3) * error;
        const float share = result.wheel[0] / raw_wheel;
        const float t1 = axial_torque(THETA_REF);
        std::printf("spin wheel=%.3f raw=%.3f swing=%.3f raw=%.3f share=%.4f\n",
                    result.wheel[0], raw_wheel, gain(2, 3) * error, raw_swing, share);

        for (int i = 0; i < 2; i++) {
            const float t2 = (i == 0 ? 1.0f : -1.0f) * 0.5f * share * gain(2 + i, 3) * error;
            std::printf("spin joint[%d]=(%.3f, %.3f) expect=(%.3f, %.3f)\n", i, result.joint_a[i], result.joint_e[i], t1 + t2, -t1 + t2);
            expect(std::fabs(result.wheel[i]) <= WHEEL_MAX_TORQUE + 1e-3f, "高速旋转", "轮端超过电机上限");
            expect(std::fabs(result.wheel[i]) >= 4.0f, "高速旋转", "运动出力被过度削减");
            expect(share < 0.9f, "高速旋转", "运动组未退让");
            expect(std::fabs(result.joint_a[i] - (t1 + t2)) < 1e-3f, "高速旋转", "关节 A 与分配比例不符");
            expect(std::fabs(result.joint_e[i] - (-t1 + t2)) < 1e-3f, "高速旋转", "关节 E 与分配比例不符");
            expect(std::fabs(result.joint_a[i]) <= JOINT_MAX_TORQUE + 1e-3f, "高速旋转", "关节 A 超过力矩上限");
            expect(std::fabs(result.joint_e[i]) <= JOINT_MAX_TORQUE + 1e-3f, "高速旋转", "关节 E 超过力矩上限");
        }
    }

    std::printf("PASS: %d checks\n", checks);
}
