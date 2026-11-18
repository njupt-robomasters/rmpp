#include "chassis.hpp"
#include "../leg_protocol.hpp"
#include "app.hpp"

// 上板指令，在 CAN 中断内解码
static LegUpFrame up;
static bool up_is_connect = false;
static Timer callback_timer;

static void control_by_up() {
    chassis.set_enable(up.is_enable);
    chassis.set_allow_rescue(up.allow_rescue);
    chassis.set_trigger_jump(up.trigger_jump);
    chassis.set_speed(up.vx, up.wr);
    chassis.set_gimbal_yaw(up.yaw);
    chassis.set_leg_length(up.leg_length);
    chassis.set_power_ref(up.power_ref);
}

static void control_by_fsi6x() {
    chassis.set_enable(fsi6x.is_connect && fsi6x.swa == FSi6X::DOWN);
    chassis.set_allow_rescue(fsi6x.swb == FSi6X::DOWN);
    chassis.set_trigger_jump(fsi6x.swd == FSi6X::DOWN);

    chassis.set_speed(fsi6x.x * 3.0f, fsi6x.y * 360 * deg_s);
    chassis.set_gimbal_yaw(0);

    if (fsi6x.swc == FSi6X::UP)
        chassis.set_leg_length(0.14f);
    else if (fsi6x.swc == FSi6X::MID)
        chassis.set_leg_length(0.22f);
    else if (fsi6x.swc == FSi6X::DOWN)
        chassis.set_leg_length(0.33f);

    chassis.set_power_ref(INFINITY);
}

void chassis_loop() {
    // 上板在线时由上板控制，断联时回退 FSi6X
    if (callback_timer.elapsed() > LEG_TIMEOUT)
        up_is_connect = false;

    if (up_is_connect) {
        control_by_up();
    } else {
        control_by_fsi6x();
    }

    // 姿态来自本板 IMU；换用虚拟 IMU 时传入换算后的底盘 q 与 gyro
    chassis.update(attitude.q, imu.gyro);
}

void chassis_write_can_frame(CanFrame& frame) {
    const LegDownFrame down{
        .is_fallen = chassis.rescue.is_fallen,
        .wr = chassis.body.d_yaw_gyro,
    };
    down.pack(frame);
}

void chassis_can_callback(const CanFrame& frame) {
    if (!up.unpack(frame))
        return;

    callback_timer.restart();
    up_is_connect = true;
}
