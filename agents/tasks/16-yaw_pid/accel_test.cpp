#include "../13-minipc_nav/support.hpp"
#include "gimbal_fold.hpp"

class TestMotor : public Motor {
  public:
    TestMotor() : Motor({.can = Can(1)}, 100, 100, 1) {}
    void feedback(const float angle, const float speed = 0) {
        can_callback({.speed = speed, .angle = angle});
    }
};

static const Pid::Config SPEED{.kp = 1, .max_out = 100};
static const Pid::Config POSITION{.kp = 4, .max_out = 720 * deg_s};
static constexpr float ACCEL = 720 * deg_ss;

struct Rig {
    TestMotor big, small, fold, pitch;
    GimbalFold g{{big, small, fold, pitch}, {
        .yaw_speed_pid_config = &SPEED,
        .yaw_position_pid_config = &POSITION,
        .pitch_speed_pid_config = &SPEED,
        .pitch_position_pid_config = &POSITION,
        .pitch_min = -1,
        .pitch_max = 1,
        .yaw1_max_accel = ACCEL,
        .yaw2 = {&SPEED, &POSITION, -40 * deg, 40 * deg},
        .pitch1 = {&SPEED, &POSITION, -2, 0},
    }};
    float angle = 0, speed = 0;

    Rig() {
        g.set_enable(true);
        tick(0);
        tick(0);
    }

    void tick(const float target, const uint32_t step_us = 1000, const float wr = 0) {
        const float dt = step_us * 1e-6f;
        now_us += step_us;
        big.feedback(angle, speed);
        small.feedback(g.yaw2.ref);
        fold.feedback(0);
        pitch.feedback(0);
        g.set_angle(target, 0);
        g.set_chassis_wr(wr);
        g.update(Angle(angle + small.angle), 0);
        const float command = speed + big.torque_ref;
        CHECK(std::fabs(command - speed) <= ACCEL * dt + 2e-5f);
        speed = command;
        angle += speed * dt;
        CHECK(std::fabs(float(g.yaw2.ref)) <= 40 * deg + 1e-6f);
        near(fold.torque_ref, 0);
    }
};

int main() {
    // 正反阶跃、动态换向与抖动周期：速度增量有界，目标角最终收敛
    Rig r;
    for (const float target : {170 * deg, -70 * deg, 20 * deg}) {
        for (int i = 0; i < 6000; i++) {
            r.tick(target, i % 2 == 0 ? 500 : 1500);
            near(r.g.yaw.imu.ref, Angle(target));
        }
        CHECK(std::fabs(float(Angle(r.angle - target))) < 0.001f);
        CHECK(std::fabs(r.speed) < 0.001f);
    }
    for (int i = 0; i < 200; i++)
        r.tick(170 * deg);
    for (int i = 0; i < 6000; i++)
        r.tick(-100 * deg);
    CHECK(std::fabs(float(Angle(r.angle + 100 * deg))) < 0.001f);

    // 匀加速区间的角位移与 t² 成比例；世界目标与小轴分配保留
    Rig quadratic;
    for (int i = 0; i < 100; i++)
        quadratic.tick(170 * deg);
    const float first = quadratic.angle;
    for (int i = 0; i < 100; i++)
        quadratic.tick(170 * deg);
    CHECK(quadratic.angle / first > 3.95f && quadratic.angle / first < 4.05f);

    // 目标速度前馈撤销和底盘前馈突变同样经过加减速约束
    Rig feedforward;
    for (int i = 0; i < 300; i++) {
        now_us += 1000;
        feedforward.big.feedback(feedforward.angle, feedforward.speed);
        feedforward.small.feedback(0);
        feedforward.g.set_speed(i < 150 ? 360 * deg_s : 0, 0);
        feedforward.g.set_chassis_wr(i < 200 ? 0 : 2);
        feedforward.g.update(feedforward.angle, 0);
        const float command = feedforward.speed + feedforward.big.torque_ref;
        CHECK(std::fabs(command - feedforward.speed) <= ACCEL * 0.001f + 2e-5f);
        feedforward.speed = command;
        feedforward.angle += command * 0.001f;
    }

    // 失能立即清输出，重启从实测速度开始；离线清规划，恢复首周期重新初始化
    r.g.set_enable(false);
    r.tick(r.angle);
    near(r.big.torque_ref, 0);
    r.g.set_enable(true);
    r.tick(r.angle);
    near(r.big.torque_ref, 0);
    now_us += 200000;
    r.g.update(0, 0);
    CHECK(!r.big.is_connect);
    near(r.big.torque_ref, 0);
    r.speed = 1.2f;
    r.tick(r.angle);
    near(r.big.torque_ref, 0);
    r.tick(r.angle + 0.2f);

    // 跨 ±π 走短路径，底盘运动时保持有限输出
    Rig wrap;
    wrap.g.set_enable(false);
    wrap.angle = 179 * deg;
    wrap.tick(wrap.angle);
    wrap.g.set_enable(true);
    wrap.tick(wrap.angle);
    for (int i = 0; i < 3000; i++)
        wrap.tick(-179 * deg);
    CHECK(std::fabs(float(Angle(wrap.angle + 179 * deg))) < 0.001f);
    std::printf("yaw acceleration checks: %d\n", checks);
}
