#include "../15-minipc_nav/support.hpp"
#include "gimbal_fold.hpp"

class TestMotor : public Motor {
  public:
    TestMotor() : Motor({.can = Can(1), .timeout = 0.01f}, 20, 6, 1) {}
    void feedback(const float angle, const float speed = 0) { can_callback({.speed = speed, .angle = angle}); }
};

static const Pid::Config SPEED{.kp = 1, .ki = 0.1f, .max_i = 1, .max_out = 10};
static const Pid::Config POSITION{.kp = 5, .max_out = 10};

struct Rig {
    TestMotor big, small, fold, pitch;
    GimbalFold g{{big, small, fold, pitch}, {
        .yaw_speed_pid_config = &SPEED, .yaw_position_pid_config = &POSITION,
        .pitch_speed_pid_config = &SPEED, .pitch_position_pid_config = &POSITION,
        .pitch_min = -1, .pitch_max = 1,
        .yaw2 = {&SPEED, &POSITION, -45 * deg, 45 * deg},
        .pitch1 = {&SPEED, &POSITION, -2, 0},
    }};
    float base = 0, q1 = 0, q2 = 0;
    bool big_online = true, small_online = true;
    Rig() { g.set_enable(true); tick(0); tick(0); }
    void tick(const float target, const float wr = 0) {
        now_us += 1000;
        if (big_online) big.feedback(q1, -wr);
        if (small_online) small.feedback(q2);
        fold.feedback(-0.3f);
        pitch.feedback(0);
        g.set_angle(target, 0);
        g.set_chassis_wr(wr);
        g.update(Angle(base + q1 + q2), 0);
        near(fold.current_ref, 0);
        near(fold.torque_ref, 0);
    }
    void plant() { q1 = g.yaw1.ref; q2 = g.yaw2.ref; }
};

int main() {
    // 窗口内仅小轴改变瞄准目标，头部总目标保持外部命令
    Rig a;
    for (const float target : {20 * deg, -30 * deg, 44.9f * deg, -44.9f * deg}) {
        for (int i = 0; i < 20; i++) { a.tick(target); a.plant(); }
        near(a.g.yaw1.ref, 0);
        near(a.g.yaw2.ref, target);
        near(a.g.yaw.imu.ref, target);
    }

    // 大目标首周期直接分配到内侧目标，小轴始终限于45度
    Rig b;
    for (int i = 0; i < 3000; i++) {
        b.tick(70 * deg);
        const float world = Angle(float(b.g.yaw1.ref) + b.base);
        near(world, 35 * deg);
        CHECK(std::fabs(float(b.g.yaw2.ref)) <= 45 * deg + 1e-6f);
        near(b.g.yaw.imu.ref, 70 * deg);
        if (i == 0)
            CHECK(b.big.current_ref > 3);
        b.plant();
    }
    CHECK(b.q1 > 34.5f * deg && b.q1 < 35.5f * deg);
    CHECK(b.q2 > 34.5f * deg && b.q2 < 35.5f * deg);

    // 回到内侧后保持新的大轴世界朝向，不再把小轴强制回中
    b.tick(65 * deg); b.plant();
    for (int i = 0; i < 1000; i++) { b.tick(65 * deg); b.plant(); }
    const float held = b.q1;
    for (int i = 0; i < 100; i++) { b.tick(held + 40 * deg); b.plant(); }
    near(b.q1, held);

    // 反向大目标首周期直接换向
    b.tick(-70 * deg);
    near(b.g.yaw1.ref, -35 * deg);
    for (int i = 0; i < 3500; i++) { b.tick(-70 * deg); b.plant(); }
    CHECK(b.q1 < -34.5f * deg && b.q1 > -35.5f * deg);
    near(b.g.yaw.imu.ref, -70 * deg);

    // 实测越过停止阈值时保持当下朝向，回差区继续保持
    Rig stop;
    stop.tick(70 * deg);
    stop.q1 = 40 * deg;
    stop.q2 = 30 * deg;
    stop.tick(70 * deg);
    near(stop.g.yaw1.ref, 40 * deg);
    CHECK(std::fabs(stop.big.torque_ref) < 0.01f);
    stop.tick(80 * deg);
    near(stop.g.yaw1.ref, 40 * deg);

    // 小陀螺：大轴反向随底盘转，世界朝向保持，小轴只承担20度瞄准
    Rig spin;
    spin.q2 = 20 * deg;
    for (int i = 0; i < 2000; i++) {
        spin.base = Angle(spin.base + 120 * rpm * 0.001f);
        spin.q1 = -spin.base;
        spin.tick(20 * deg, 120 * rpm);
        near(Angle(float(spin.g.yaw1.ref) + spin.base), 0);
        near(spin.g.yaw2.ref, 20 * deg);
        near(spin.small.current_ref, 0);
        near(spin.big.current_ref, 0);
    }

    // 世界系跨±180度时取最短角，20度转动由小轴完成
    Rig wrap;
    wrap.g.set_enable(false); wrap.base = 179 * deg; wrap.tick(179 * deg);
    wrap.g.set_enable(true); wrap.tick(179 * deg); wrap.tick(-161 * deg);
    near(wrap.g.yaw1.ref, 0);
    near(wrap.g.yaw2.ref, 20 * deg);
    near(wrap.g.yaw.imu.ref, -161 * deg);

    // 离线、重新使能、ECD/IMU模式切换保持有限输出和pitch1失能
    wrap.small_online = false;
    for (int i = 0; i < 20; i++) wrap.tick(-150 * deg);
    near(wrap.small.current_ref, 0);
    wrap.small_online = true; wrap.tick(-150 * deg); wrap.tick(-150 * deg);
    CHECK(std::isfinite(float(wrap.g.yaw1.ref)));
    wrap.g.set_mode(GimbalFold::ECD_MODE); wrap.tick(0.1f);
    near(wrap.g.yaw1.ref, 0.1f);
    wrap.g.set_mode(GimbalFold::IMU_MODE); wrap.tick(-160 * deg);
    CHECK(std::isfinite(wrap.big.current_ref));
    wrap.g.set_enable(false); wrap.tick(0);
    near(wrap.big.current_ref, 0); near(wrap.small.current_ref, 0);
    std::printf("Yaw window: 9 scenario groups passed, %d assertions\n", checks);
}
