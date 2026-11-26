#include "chassis.hpp"
#include "app.hpp"
#include "gimbal.hpp"
#include "minipc.hpp"
#include "referee.hpp"

void chassis_loop() {
    // 下板断联检测
    leg_protocol.update();

    // 航向：小陀螺发手动角速度并禁用航向追踪；随头发相对航向，自瞄跟踪时清零摇杆前馈
    leg_protocol.trigger_jump = vt13.key.e || vt13.rc.fn;
    leg_protocol.vx = vt13.vx * std::cos(yaw.angle);
    const bool is_spin = vt13.wr != 0;
    const float yaw_feedforward = (vt13.is_aim && aim.is_detect) ? 0.0f : vt13.yaw_speed;
    leg_protocol.wr = is_spin ? vt13.wr : yaw_feedforward;
    leg_protocol.yaw = is_spin ? 0.0f : yaw.angle;

    // 按键下沿：本拍在上一拍状态上取得，供复位、自救和腿长三段共用
    static bool last_pause = false, last_v = false;
    static bool last_z = false, last_x = false, last_c = false;
    static bool is_leg_length_rising = true;
    const bool is_pause_down = vt13.rc.pause && !last_pause;
    const bool is_v_down = vt13.key.v && !last_v;
    const bool is_z_down = vt13.key.z && !last_z;
    const bool is_x_down = vt13.key.x && !last_x;
    const bool is_c_down = vt13.key.c && !last_c;
    last_pause = vt13.rc.pause;
    last_v = vt13.key.v;
    last_z = vt13.key.z;
    last_x = vt13.key.x;
    last_c = vt13.key.c;

    // 使能：直接透传遥控器挡位，下板断联时失能
    leg_protocol.is_enable = vt13.is_enable && leg_protocol.is_connect;

    if (!leg_protocol.is_enable) {
        // 复位：失能或断联时清空自救批准和腿长，重新使能后先经 allow_rescue 才进入站立控制
        leg_protocol.allow_rescue = false;
        leg_protocol.leg_length = LEG_LENGTH_LOW;
        is_leg_length_rising = true;
    } else if (leg_protocol.received.is_fallen) {
        // 自救：倒地时 pause 或 v 下沿锁存批准，站起后解锁，下次倒地重新批准
        if (is_pause_down || is_v_down)
            leg_protocol.allow_rescue = true;
    } else {
        // 腿长：使能后 pause 沿当前方向切一档，Z/X/C 直选低/中/高档
        leg_protocol.allow_rescue = false;
        if (is_pause_down) {
            // 腿长循环：LOW -> MID -> HIGH -> MID -> LOW
            if (is_leg_length_rising) {
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
        } else if (is_z_down) {
            // 腿长直选：Z/X/C 分别选低/中/高档，并同步 pause 循环的升降方向
            leg_protocol.leg_length = LEG_LENGTH_LOW;
            is_leg_length_rising = true;
        } else if (is_x_down) {
            leg_protocol.leg_length = LEG_LENGTH_MID;
        } else if (is_c_down) {
            leg_protocol.leg_length = LEG_LENGTH_HIGH;
            is_leg_length_rising = false;
        }
    }

    // 超级电容：底盘使能时开 DCDC 并叠加放电补偿
    super_cap.set_enable(vt13.is_enable);
    super_cap.set_boost_enable(vt13.is_enable);
    super_cap.set_referee_limit(referee.robot_status.chassis_power_limit);
    super_cap.set_referee_energy(referee.power_heat_data.buffer_energy);
    super_cap.update();
    leg_protocol.power_ref = super_cap.power_chassis_ref;
}
