#include "chassis.hpp"
#include "app.hpp"

static constexpr float MOTOR_OVERHEAT = 80; // 电机温度报警阈值，℃

using enum LegDownFrame::MotorState;

static LegDownFrame::MotorState joint_state(const DM8009& motor);
static LegDownFrame::MotorState wheel_state(const M3508& motor);

// 上板指令：在 CAN 中断内解码
static void control_by_up() {
    chassis.set_enable(leg_protocol.received.is_enable);
    chassis.set_allow_rescue(true);
    chassis.set_trigger_jump(leg_protocol.received.trigger_jump);
    chassis.set_speed(leg_protocol.received.vx, leg_protocol.received.wr);
    chassis.set_gimbal_yaw(leg_protocol.received.yaw);
    chassis.set_leg_length(leg_protocol.received.leg_length);
    chassis.set_power_ref(leg_protocol.received.power_ref);
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
    // 上板断联清空指令，使底盘失能
    leg_protocol.update();

    // FSi6X 在线时优先接管，避免上板协议错误导致误动作；否则按上板指令
    if (fsi6x.is_connect)
        control_by_fsi6x();
    else
        control_by_up();

    // 姿态来自本板 IMU；换用虚拟 IMU 时传入换算后的底盘 q 与 gyro
    chassis.update(attitude.q, imu.gyro);

    leg_protocol.is_fallen = chassis.rescue.state != ChassisLeg::Rescue::State::UPRIGHT;
    leg_protocol.is_enable = chassis.is_enable;
    leg_protocol.wr = chassis.body.d_yaw_gyro;
    leg_protocol.motor[0] = joint_state(left_front);
    leg_protocol.motor[1] = joint_state(left_back);
    leg_protocol.motor[2] = joint_state(right_back);
    leg_protocol.motor[3] = joint_state(right_front);
    leg_protocol.motor[4] = wheel_state(left_wheel);
    leg_protocol.motor[5] = wheel_state(right_wheel);
}

// 关节状态：反馈超时优先，其次达妙错误码与过热
static LegDownFrame::MotorState joint_state(const DM8009& motor) {
    if (!motor.is_connect)
        return OFFLINE;
    if (motor.err_code != DM8009::NONE && motor.err_code != DM8009::ENABLED)
        return ERROR;
    if (motor.temperature >= MOTOR_OVERHEAT)
        return OVERHEAT;
    return NORMAL;
}

// 轮状态：反馈超时与过热
static LegDownFrame::MotorState wheel_state(const M3508& motor) {
    if (!motor.is_connect)
        return OFFLINE;
    if (motor.temperature >= MOTOR_OVERHEAT)
        return OVERHEAT;
    return NORMAL;
}
