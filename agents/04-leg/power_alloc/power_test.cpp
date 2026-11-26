// 轮腿功率分配：同一状态分别以不限功率和给定预算运行真实 ChassisLeg，输出四路力矩供 check_power.py 与仿真算法对照
// 主机自检：功率未超时原样下发；超限后双轮净机械功率不超过预算、轮与关节不超过额定力矩
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
static constexpr float WHEEL_REDUCTION = 16.875f;
static constexpr float ACTIVE_LINK = 0.210f;
static constexpr float PASSIVE_LINK = 0.248f;
static constexpr float LEG_LENGTH = 0.18f;
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

// 下发力矩：轮前进为正；摆动力矩后摆为正，由两关节差值还原；axial 为轴向力在关节上的力矩
struct Result {
    float wheel[2], swing[2], axial[2];
};

struct Case {
    const char* name;
    float omega[2]; // 轮轴速度，rad/s，前进为正
    float pitch;    // 车身俯仰，rad，低头为正；腿倾角保持 theta_ref
    float vx, wr;   // 速度目标，m/s、rad/s
    float budget;   // 功率预算，W
};

static Result run(const Case& c, const float power_ref) {
    TestDm left_front, left_back, right_back, right_front;
    TestWheel left_wheel(0x201), right_wheel(0x202);
    ChassisLeg chassis({left_front, left_back, right_back, right_front, left_wheel, right_wheel},
                       {
                           .leg_length_pid = &leg_length_pid,
                           .leg_angle_pid = &leg_angle_pid,
                           .roll_pid = &roll_pid_config,
                           .lqr_fit = LQR_FIT,
                           .lqr_posture_fit = LQR_POSTURE_FIT,
                           .lqr_r = LQR_R,
                       });

    // 五连杆反解：phi1 = phi0 + beta、phi4 = phi0 - beta
    const float beta = std::acos((ACTIVE_LINK * ACTIVE_LINK + LEG_LENGTH * LEG_LENGTH - PASSIVE_LINK * PASSIVE_LINK) / (2 * ACTIVE_LINK * LEG_LENGTH));
    const float rel_theta = THETA_REF - c.pitch;
    const float phi0_left = pi / 2 - rel_theta, phi0_right = pi / 2 + rel_theta;
    left_back.feed(pi - (phi0_left + beta), 0);
    left_front.feed(0 - (phi0_left - beta), 0);
    right_front.feed(pi - (phi0_right + beta), 0);
    right_back.feed(0 - (phi0_right - beta), 0);
    left_wheel.feed(c.omega[0]);
    right_wheel.feed(c.omega[1]);

    chassis.set_enable(true);
    chassis.set_speed(c.vx, c.wr);
    chassis.set_leg_length(LEG_LENGTH);
    chassis.set_power_ref(power_ref);
    now_cycles += 2000;
    chassis.update(Quaternionf(Eigen::AngleAxisf(c.pitch, Vector3f::UnitY())), Vector3f::Zero());

    // 左腿 Ta = T1 + Tt/2、Te = -T1 + Tt/2；右腿镜像，Tt 取反
    const float a[2] = {left_back.torque_ref, right_front.torque_ref};
    const float e[2] = {left_front.torque_ref, right_back.torque_ref};
    return {{left_wheel.torque_ref, right_wheel.torque_ref},
            {a[0] + e[0], -(a[1] + e[1])},
            {(a[0] - e[0]) / 2, (a[1] - e[1]) / 2}};
}

static int checks = 0;

static void expect(const bool condition, const char* name, const char* what) {
    checks++;
    if (!condition) {
        std::fprintf(stderr, "FAIL %s: %s\n", name, what);
        std::exit(1);
    }
}

int main() {
    const Case cases[] = {
        {"直行巡航", {40, 40}, 0.2f, 3, 0, 80},
        {"边走边转", {30, 45}, 0.15f, 2, 2, 60},
        {"单轮反转", {45, -10}, 0.25f, 1, 3, 40},
        {"预算为零", {40, 40}, 0.2f, 3, 0, 0},
        {"轮端顶边", {60, 2}, 0.6f, 3, 0, 20},
        {"制动负功", {40, 40}, -0.2f, 3, 0, 80},
        {"预算充足", {40, 40}, 0.2f, 3, 0, 1000},
    };

    for (const Case& c : cases) {
        const Result raw = run(c, INFINITY);
        const Result limited = run(c, c.budget);
        const float raw_power = raw.wheel[0] * c.omega[0] + raw.wheel[1] * c.omega[1];
        const float power = limited.wheel[0] * c.omega[0] + limited.wheel[1] * c.omega[1];

        std::printf("%s omega %.9g %.9g budget %.9g raw %.9g %.9g %.9g %.9g limited %.9g %.9g %.9g %.9g axial %.9g %.9g\n",
                    c.name, c.omega[0], c.omega[1], c.budget,
                    raw.wheel[0], raw.wheel[1], raw.swing[0], raw.swing[1],
                    limited.wheel[0], limited.wheel[1], limited.swing[0], limited.swing[1],
                    limited.axial[0], limited.axial[1]);

        expect(power <= c.budget + 1e-3f || power <= 0, c.name, "双轮净机械功率超过预算");
        if (raw_power <= c.budget) {
            for (int i = 0; i < 2; i++) {
                expect(limited.wheel[i] == raw.wheel[i], c.name, "未超功率时轮端被修改");
                expect(limited.swing[i] == raw.swing[i], c.name, "未超功率时腿摆被修改");
            }
        }
        for (int i = 0; i < 2; i++) {
            expect(std::fabs(limited.wheel[i]) <= WHEEL_MAX_TORQUE + 1e-4f, c.name, "轮端超过额定力矩");
            expect(std::fabs(limited.axial[i]) + std::fabs(limited.swing[i]) / 2 <= JOINT_MAX_TORQUE + 1e-3f, c.name, "关节超过额定力矩");
        }
    }
    std::fprintf(stderr, "PASS: %d checks\n", checks);
}
