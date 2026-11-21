#include "chassis.hpp"
#include "app.hpp"
#include "gimbal.hpp"
#include "referee.hpp"

void chassis_loop() {
    // 下板断联检测
    leg_protocol.update();
    leg_protocol.trigger_jump = vt13.key.e || vt13.rc.fn;
    leg_protocol.vx = vt13.vx * std::cos(yaw.angle);
    leg_protocol.wr = vt13.wr;
    leg_protocol.yaw = yaw.angle;

    // 使能后首次按下 pause 才下发底盘使能，防止误触；之后 pause 切换腿长，失能或下板断联复位
    static bool last_pause = false;
    static bool last_z = false, last_x = false, last_c = false;
    static bool is_leg_length_rising = true;
    if (vt13.is_enable && leg_protocol.is_connect) { // vt13使能，且下板连接
        if (vt13.rc.pause && !last_pause) {
            if (!leg_protocol.is_enable) {
                leg_protocol.is_enable = true;
            } else if (is_leg_length_rising) { // LOW -> MID -> HIGH
                if (leg_protocol.leg_length < LEG_LENGTH_MID) {
                    leg_protocol.leg_length = LEG_LENGTH_MID;
                } else {
                    leg_protocol.leg_length = LEG_LENGTH_HIGH;
                    is_leg_length_rising = false;
                }
            } else { // HIGH -> MID -> LOW
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

    // 腿长直选：Z/X/C 按下沿分别选择低/中/高档
    if (leg_protocol.is_enable) {
        if (vt13.key.z && !last_z) {
            leg_protocol.leg_length = LEG_LENGTH_LOW;
            is_leg_length_rising = true;
        } else if (vt13.key.x && !last_x) {
            leg_protocol.leg_length = LEG_LENGTH_MID;
        } else if (vt13.key.c && !last_c) {
            leg_protocol.leg_length = LEG_LENGTH_HIGH;
            is_leg_length_rising = false;
        }
    }
    last_z = vt13.key.z;
    last_x = vt13.key.x;
    last_c = vt13.key.c;

    // 超级电容
    super_cap.set_mode(vt13.is_enable ? SuperCap::BOOST : SuperCap::OFF);
    super_cap.set_referee_limit(referee.robot_status.chassis_power_limit);
    super_cap.set_referee_energy(referee.power_heat_data.buffer_energy);
    super_cap.update();
}
