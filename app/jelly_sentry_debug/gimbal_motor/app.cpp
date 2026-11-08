#include <algorithm>
#include "../../jelly_sentry/gimbal.hpp"
#include "bsp/timer.hpp"
#include "rc/fsi6x.hpp"
#include "rc/vt13.hpp"

// 单个云台电机串级闭环测试，电机与 PID 参数直接使用整车 app/jelly_sentry/gimbal.hpp，调好后无需搬运
// 遥控器右手水平摇杆给目标转速，对其积分得到目标角度（编码器），并作为速度前馈
// 使能：VT13 N、S 挡，或 FSi6X SWC 中、下挡；失能或离线时目标跟随实测，使能后从原地开始
// 默认测试 pitch2（俯仰，DM4310，力矩输出）；换电机时修改：
// - motor 的类型与对象、speed_pid、position_pid 改为对应电机与参数（yaw1 / pitch1 为 DM4310，yaw2 为 RMDL7025）
// - IS_TORQUE_OUTPUT：yaw2（7025）为电流输出，其余为力矩输出
// - 测试 yaw2 时把循环周期改为 2 ms（7025 应答上限 500 Hz）
// 调试器观察 position_ref、motor.position、motor.speed、motor.torque_ref / current_ref

static constexpr float MAX_SPEED = 360 * deg_s; // 摇杆满打时的目标转速，rad/s
static constexpr bool IS_TORQUE_OUTPUT = true;  // true 力矩输出（DM4310），false 电流输出（7025）

static DM4310& motor = pitch2;
static Pid speed_pid(&pitch2_speed_pid);
static Pid position_pid(&pitch2_position_pid);

static VT13 vt13({.uart = Uart(1)});
static FSi6X fsi6x({.uart = Uart(5)});

// 跨周期状态
static Angle position_ref;
static bool is_ready = false;

// 遥控器使能：VT13 N、S 挡，或 FSi6X SWC 中、下挡；断联时不使能
static bool rc_enable() {
    return (vt13.is_connect && (vt13.mode == VT13::N || vt13.mode == VT13::S)) ||
           (fsi6x.is_connect && (fsi6x.swc == FSi6X::MID || fsi6x.swc == FSi6X::DOWN));
}

extern "C" void app_main() {
    Timer loop_timer;
    while (true) {
        if (loop_timer.elapsed() < 1 * ms)
            continue;
        const float dt = loop_timer.restart();

        vt13.update();
        fsi6x.update();
        motor.update();

        const bool is_enable = rc_enable();
        motor.set_enable(is_enable);

        // 失能或离线：目标跟随实测、清空 PID 并输出零；恢复后第一个周期仍输出零
        const bool can_control = is_enable && motor.is_connect;
        if (!can_control || !is_ready) {
            position_ref = motor.position;
            speed_pid.clear();
            position_pid.clear();
            motor.set_current(0);
            is_ready = can_control;
        } else {
            const float speed = std::clamp(vt13.x + fsi6x.x, -1.0f, 1.0f) * MAX_SPEED;
            position_ref += speed * dt;
            const float speed_ref = position_pid.calculate(Angle(position_ref - motor.position)) + speed;
            const float output = speed_pid.calculate(speed_ref - motor.speed);
            if (IS_TORQUE_OUTPUT) {
                motor.set_torque(output);
                speed_pid.track(motor.torque_ref);
            } else {
                motor.set_current(output);
                speed_pid.track(motor.current_ref);
            }
        }

        motor.send();
    }
}
