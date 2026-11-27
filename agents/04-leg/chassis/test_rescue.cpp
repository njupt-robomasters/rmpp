#include "robot/chassis_leg_lqr.hpp"
#include <cassert>
#include <cmath>

float Timer::elapsed() const {
    return 0.002f;
}

float Timer::restart() {
    return elapsed();
}

int main() {
    using State = ChassisLeg::Rescue::State;
    using Maneuver = ChassisLeg::Rescue::Maneuver;
    const Motor::Config motor_config{.can = Can(1)};
    DM8009 left_front(motor_config), left_back(motor_config), right_back(motor_config), right_front(motor_config);
    M3508 left_wheel(motor_config), right_wheel(motor_config);
    const Pid::Config pid_config{.kp = 140, .max_out = 80};
    ChassisLeg chassis({left_front, left_back, right_back, right_front, left_wheel, right_wheel},
                       {.leg_length_pid = &pid_config, .leg_angle_pid = &pid_config, .roll_pid = &pid_config, .lqr_fit = LQR_FIT, .lqr_posture_fit = LQR_POSTURE_FIT, .lqr_r = LQR_R});
    for (auto* joint : {&left_front, &left_back, &right_back, &right_front}) {
        joint->is_connect = true;
        joint->err_code = DM8009::ENABLED;
    }
    left_wheel.is_connect = right_wheel.is_connect = true;

    const auto tick = [&](const float leg_angle, const float roll = 0, const float pitch = 0) {
        left_front.angle = left_back.angle = leg_angle;
        right_front.angle = right_back.angle = -leg_angle;
        chassis.update(Eigen::AngleAxisf(pitch, Vector3f::UnitY()) * Eigen::AngleAxisf(roll, Vector3f::UnitX()), Vector3f::Zero());
    };
    static_assert(static_cast<int>(State::FALLEN) == 0);
    static_assert(static_cast<int>(State::RECOVERING) == 1);
    static_assert(static_cast<int>(State::BALANCING) == 2);
    assert(chassis.rescue.state == State::FALLEN);
    chassis.set_enable(true);
    tick(0);
    assert(chassis.rescue.state == State::FALLEN);
    chassis.set_allow_rescue(true);
    left_front.err_code = DM8009::NONE;
    tick(0);
    assert(chassis.rescue.state == State::FALLEN && chassis.is_enable);
    left_front.err_code = DM8009::ENABLED;
    tick(0);
    assert(chassis.rescue.state == State::BALANCING);
    chassis.set_allow_rescue(false);
    tick(85 * deg);
    assert(chassis.rescue.state == State::FALLEN);
    chassis.set_allow_rescue(true);
    for (int cycle = 1; cycle <= 6; cycle++) {
        tick(85 * deg);
        assert(chassis.rescue.state == State::RECOVERING && chassis.rescue.maneuver == Maneuver::STAND_UP);
        assert(std::fabs(chassis.rescue.target_theta[0] - (85 * deg - cycle * 0.008f)) < 1e-5f);
        assert(left_front.torque_ref + left_back.torque_ref < 0);
        chassis.set_allow_rescue(false);
    }
    tick(0);
    assert(chassis.rescue.state == State::BALANCING);

    tick(0);
    assert(chassis.body.is_position);
    left_front.is_connect = false;
    tick(0);
    assert(!chassis.body.is_position && chassis.body.x == 0 && chassis.rescue.state == State::FALLEN);
    left_front.is_connect = true;
    chassis.set_allow_rescue(true);
    tick(0);
    chassis.set_allow_rescue(false);

    for (const float pitch : {15 * deg, 0.0f, -15 * deg}) {
        const Maneuver maneuver = pitch > 0 ? Maneuver::FLIP_FROM_HEAD : Maneuver::FLIP_FROM_TAIL;
        const float roll = pitch > 0 ? -110 * deg : 110 * deg;
        tick(0, roll, pitch);
        assert(chassis.rescue.state == State::FALLEN);
        chassis.set_allow_rescue(true);
        tick(0, roll, pitch);
        assert(chassis.rescue.state == State::RECOVERING && chassis.rescue.maneuver == maneuver);
        assert(maneuver == Maneuver::FLIP_FROM_HEAD ? chassis.rescue.target_theta[0] > 0 : chassis.rescue.target_theta[0] < 0);
        chassis.set_allow_rescue(false);
        tick(0);
        assert(chassis.rescue.state == State::RECOVERING && chassis.rescue.maneuver == maneuver);
        chassis.set_enable(false);
        tick(0);
        assert(chassis.rescue.state == State::FALLEN && chassis.rescue.target_theta[0] == 0);
        for (const auto* joint : {&left_front, &left_back, &right_back, &right_front})
            assert(joint->torque_ref == 0);
        chassis.set_enable(true);
        tick(0);
        assert(chassis.rescue.state == State::FALLEN);
        chassis.set_allow_rescue(true);
        tick(0);
        assert(chassis.rescue.state == State::BALANCING);
        chassis.set_allow_rescue(false);
    }
    chassis.set_trigger_jump(true);
    tick(0);
    chassis.set_trigger_jump(false);
    tick(85 * deg);
    assert(chassis.rescue.state == State::BALANCING);
    right_wheel.is_connect = false;
    chassis.set_trigger_jump(true);
    tick(0);
    assert(chassis.rescue.state == State::FALLEN && chassis.is_enable);
    for (const auto* joint : {&left_front, &left_back, &right_back, &right_front})
        assert(joint->torque_ref == 0);
    assert(left_wheel.torque_ref == 0 && right_wheel.torque_ref == 0);
    right_wheel.is_connect = true;
    tick(0);
    assert(chassis.rescue.state == State::FALLEN);
    chassis.set_allow_rescue(true);
    tick(0);
    assert(chassis.rescue.state == State::BALANCING);
    chassis.set_allow_rescue(false);
    tick(85 * deg);
    assert(chassis.rescue.state == State::FALLEN);
    chassis.set_allow_rescue(true);
    tick(85 * deg);
    assert(chassis.rescue.state == State::RECOVERING);
    right_back.is_connect = false;
    tick(85 * deg);
    assert(chassis.rescue.state == State::FALLEN && chassis.rescue.target_theta[0] == 0);
}
