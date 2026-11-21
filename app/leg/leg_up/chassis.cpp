#include "chassis.hpp"
#include "app.hpp"
#include "gimbal.hpp"
#include "referee.hpp"

void chassis_loop() {
    super_cap.set_mode(vt13.is_enable ? SuperCap::BOOST : SuperCap::OFF);

    // 裁判功率上限与缓冲能量：离线沿用最后值，启动默认 120 W / 60 J
    super_cap.set_referee_limit(referee.robot_status.chassis_power_limit);
    super_cap.set_referee_energy(referee.power_heat_data.buffer_energy);
    super_cap.update();

    // 下板断联检测
    leg_protocol.update();

    // 使能后首次按下 pause 才下发底盘使能，防止误触；之后 pause 切换腿长，失能或下板断联复位
    static bool last_pause = false;
    static bool is_leg_length_rising = true;
    if (vt13.is_enable && leg_protocol.is_connect) {
        if (vt13.rc.pause && !last_pause) {
            if (!leg_protocol.is_enable) {
                leg_protocol.is_enable = true;
            } else if (is_leg_length_rising) {
                if (leg_protocol.leg_length < LEG_LENGTH_MID) {
                    leg_protocol.leg_length = LEG_LENGTH_MID;
                } else {
                    leg_protocol.leg_length = LEG_LENGTH_HIGH;
                    is_leg_length_rising = false;
                }
            } else {
                if (leg_protocol.leg_length > LEG_LENGTH_MID) {
                    leg_protocol.leg_length = LEG_LENGTH_MID;
                } else {
                    leg_protocol.leg_length = LEG_LENGTH_LOW;
                    is_leg_length_rising = true;
                }
            }
        }
    } else {
        leg_protocol.is_enable = false;
        leg_protocol.leg_length = LEG_LENGTH_LOW;
        is_leg_length_rising = true;
    }
    last_pause = vt13.rc.pause;

    leg_protocol.trigger_jump = vt13.rc.fn;
    leg_protocol.vx = vt13.vx * std::cos(yaw.angle);
    leg_protocol.wr = vt13.wr;
    leg_protocol.yaw = yaw.angle;
}
