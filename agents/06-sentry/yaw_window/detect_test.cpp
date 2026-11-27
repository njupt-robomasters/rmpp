#include "../../05-comm/nav/support.hpp"
#include "robot/gimbal_fold.hpp"

static void detect_near(const float actual, const float expected, const int line) {
    if (!std::isfinite(actual) || std::fabs(actual - expected) >= 2e-5f)
        std::fprintf(stderr, "line %d: actual=%g expected=%g\n", line, actual, expected);
    near(actual, expected);
}

#define near(actual, expected) detect_near(actual, expected, __LINE__)

class DetectMotor : public Motor {
  public:
    DetectMotor() : Motor({.can = Can(1), .timeout = 0.01f}, 20, 6, 1) {}

    void feedback(const float angle, const float speed = 0) {
        can_callback({.speed = speed, .angle = angle});
    }
};

static const Pid::Config SPEED{.kp = 1, .max_out = 20};
static const Pid::Config POSITION{.kp = 5, .max_out = 10};

struct DetectRig {
    DetectMotor big, small, fold, pitch;
    GimbalFold g{{big, small, fold, pitch}, {
                                                .yaw_speed_pid_config = &SPEED,
                                                .yaw_position_pid_config = &POSITION,
                                                .pitch_speed_pid_config = &SPEED,
                                                .pitch_position_pid_config = &POSITION,
                                                .pitch_min = -1,
                                                .pitch_max = 1,
                                                .yaw2 = {&SPEED, &POSITION, -40 * deg, 40 * deg},
                                                .pitch1 = {&SPEED, &POSITION, -2, 0},
                                            }};
    float base = 0, q1 = 0, q2 = 0;
    bool detected = false, big_online = true, small_online = true;

    DetectRig() {
        g.set_enable(true);
        tick(0);
        tick(0);
    }

    void tick(const float target, const float wr = 0) {
        now_us += 1000;
        if (big_online)
            big.feedback(q1, -wr);
        if (small_online)
            small.feedback(q2);
        fold.feedback(-0.3f);
        pitch.feedback(0);
        g.set_yaw_window(detected);
        g.set_angle(target, 0);
        g.set_chassis_wr(wr);
        g.update(euler_to_quaternion({.yaw = Angle(base + q1 + q2)}), Vector3f::Zero());
        near(fold.current_ref, 0);
        near(fold.torque_ref, 0);
        near(g.yaw.imu.ref, target);
        CHECK(std::fabs(float(g.yaw2.ref)) <= 40 * deg + 1e-6f);
    }
};

int main() {
    // 未识别时大轴追随总目标，小轴补偿尚未完成的转角
    DetectRig r;
    r.tick(30 * deg);
    near(r.g.yaw1.ref, 30 * deg);
    near(r.g.yaw2.ref, 30 * deg);
    r.q1 = 10 * deg;
    r.tick(30 * deg);
    near(r.g.yaw1.ref, 30 * deg);
    near(r.g.yaw2.ref, 20 * deg);

    // 识别上升沿从实测大轴朝向保持，窗口内仅小轴追踪
    r.detected = true;
    for (const float target : {30 * deg, -20 * deg, 50 * deg, -30 * deg}) {
        r.tick(target);
        near(r.g.yaw1.ref, 10 * deg);
        near(r.g.yaw2.ref, target - 10 * deg);
    }

    // 正反越界只补足超出角度，到位后继续保持新的大轴朝向
    r.tick(70 * deg);
    near(r.g.yaw1.ref, 30 * deg);
    r.q1 = 30 * deg;
    r.tick(60 * deg);
    near(r.g.yaw1.ref, 30 * deg);
    near(r.g.yaw2.ref, 30 * deg);
    r.tick(-70 * deg);
    near(r.g.yaw1.ref, -30 * deg);
    r.q1 = -30 * deg;
    r.tick(-70 * deg);
    near(r.g.yaw2.ref, -40 * deg);

    // 检测下降沿立即恢复大轴追随、小轴误差补偿
    r.detected = false;
    r.tick(-10 * deg);
    near(r.g.yaw1.ref, -10 * deg);
    near(r.g.yaw2.ref, 20 * deg);

    // 小陀螺时大轴保持世界朝向，底盘旋转前馈抵消相对转速
    DetectRig spin;
    spin.detected = true;
    spin.tick(20 * deg);
    spin.q2 = 20 * deg;
    for (int i = 0; i < 1500; i++) {
        spin.base = Angle(spin.base + 120 * rpm * 0.001f);
        spin.q1 = -spin.base;
        spin.tick(20 * deg, 120 * rpm);
        near(Angle(float(spin.g.yaw1.ref) + spin.base), 0);
        near(spin.g.yaw2.ref, 20 * deg);
        near(spin.big.current_ref, 0);
        near(spin.small.current_ref, 0);
    }

    // 跨越世界航向 ±180° 时采用最短角
    DetectRig wrap;
    wrap.base = 179 * deg;
    wrap.detected = true;
    wrap.tick(-161 * deg);
    near(wrap.g.yaw1.ref, 0);
    near(wrap.g.yaw2.ref, 20 * deg);

    // 掉线期间回退普通分配，恢复后从实测朝向重新建立窗口
    wrap.small_online = false;
    for (int i = 0; i < 20; i++)
        wrap.tick(-151 * deg);
    near(wrap.small.current_ref, 0);
    near(wrap.g.yaw1.ref, 30 * deg);
    wrap.small_online = true;
    wrap.tick(-151 * deg);
    wrap.tick(-151 * deg);
    near(wrap.g.yaw1.ref, 0);
    wrap.g.set_enable(false);
    now_us += 1000;
    wrap.g.update(euler_to_quaternion({.yaw = 179 * deg}), Vector3f::Zero());
    near(wrap.big.current_ref, 0);
    near(wrap.small.current_ref, 0);
    wrap.g.set_enable(true);
    wrap.tick(179 * deg);
    wrap.tick(-161 * deg);
    near(wrap.g.yaw1.ref, 0);
    near(wrap.g.yaw2.ref, 20 * deg);
    wrap.g.set_mode(GimbalFold::ECD_MODE);
    wrap.g.set_angle(0.1f, 0);
    now_us += 1000;
    wrap.g.update(euler_to_quaternion({.yaw = 179 * deg}), Vector3f::Zero());
    near(wrap.g.yaw1.ref, 0.1f);
    std::printf("Detect yaw: passed %d assertions\n", checks);
}
