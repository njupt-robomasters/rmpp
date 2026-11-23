#include "../../../app/jelly_sentry_debug/yaw_pid/axis.hpp"
#include "../13-minipc_nav/support.hpp"

class TestMotor : public Motor {
  public:
    TestMotor() : Motor({.can = Can(1), .timeout = 0.01f}, 2, 1.5f, 1) {}

    void feedback(const float angle, const float speed = 0) {
        can_callback({.speed = speed, .angle = angle});
    }
};

static const Pid::Config SPEED{.kp = 1, .max_out = 2};
static const Pid::Config POSITION{.kp = 5, .max_out = 60 * deg_s};

int main() {
    TestMotor big, small;
    YawTestAxis big_axis(big, {&SPEED, &POSITION});
    YawTestAxis small_axis(small, {&SPEED, &POSITION, -30 * deg, 30 * deg});
    big.feedback(25 * deg);
    small.feedback(0);
    big_axis.set_enable(true);
    small_axis.set_enable(true);
    big_axis.update(0.001f);
    small_axis.update(0.001f);

    // 大轴阶跃，小轴保持编码器零位；大轴实际运动不进入小轴误差
    for (const float measured : {25 * deg, 35 * deg, 45 * deg}) {
        big.feedback(measured, 0.2f);
        small.feedback(0);
        big_axis.set_angle(45 * deg);
        small_axis.set_angle(0);
        big_axis.update(0.001f);
        small_axis.update(0.001f);
        near(small_axis.ref, 0);
        near(small.current_ref, 0);
    }

    // 小轴阶跃，大轴保持请求时编码器角；小轴转动不驱动大轴补偿
    for (const float measured : {-20 * deg, 0.0f, 20 * deg}) {
        big.feedback(25 * deg);
        small.feedback(measured, 0.2f);
        big_axis.set_angle(25 * deg);
        small_axis.set_angle(20 * deg);
        big_axis.update(0.001f);
        small_axis.update(0.001f);
        near(big_axis.ref, 25 * deg);
        near(big.current_ref, 0);
    }

    // 固定轴受到扰动时有恢复输出，参考角继续保持
    big.feedback(27 * deg);
    big_axis.update(0.001f);
    CHECK(big.current_ref < 0);
    near(big_axis.ref, 25 * deg);
    small_axis.set_angle(40 * deg);
    near(small_axis.ref, 30 * deg);
    small_axis.set_angle(-40 * deg);
    near(small_axis.ref, -30 * deg);

    // 失能清输出，恢复首周期从实测初始化；离线也清零
    big_axis.set_enable(false);
    big_axis.update(0.001f);
    near(big.current_ref, 0);
    big_axis.set_enable(true);
    big_axis.set_angle(-20 * deg);
    big_axis.update(0.001f);
    near(big_axis.ref, 27 * deg);
    near(big.current_ref, 0);
    big.is_connect = false;
    big_axis.update(0.001f);
    near(big.current_ref, 0);
}
