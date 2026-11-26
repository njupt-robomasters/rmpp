#include "chassis.hpp"
#include "app.hpp"
#include "gimbal.hpp"
#include "minipc.hpp"
#include "referee.hpp"

static float follow_offset = 0; // 车头相对云台的目标偏角，rad；左正右负

void chassis_loop() {
    // 使能：直接透传遥控器挡位，下板断联时失能
    leg_protocol.update();
    leg_protocol.is_enable = vt13.is_enable && leg_protocol.is_connect;

    // 随头：横向占优时首次锁存左/右 90°，横向反打保留车头；纵向占优回正，等幅或松手保持
    const bool is_spin = vt13.wr != 0;
    if (!leg_protocol.is_enable) {
        follow_offset = 0;
    } else if (!is_spin && yaw.is_connect) {
        if (std::fabs(vt13.vx) > std::fabs(vt13.vy))
            follow_offset = 0;
        else if (follow_offset == 0 && std::fabs(vt13.vy) > std::fabs(vt13.vx))
            follow_offset = vt13.vy > 0 ? 90 * deg : -90 * deg;
    }

    // 平移：按实测云台相对角把两轴速度投影到车头；跟随偏角仅进入下板航向误差
    leg_protocol.vx = vt13.vx * std::cos(yaw.angle) - vt13.vy * std::sin(yaw.angle);
    leg_protocol.yaw = is_spin || !yaw.is_connect ? 0.0f : static_cast<float>(Angle(yaw.angle + follow_offset));
    const float yaw_feedforward = (vt13.is_aim && aim.is_detect) ? 0.0f : vt13.yaw_speed;
    leg_protocol.wr = is_spin ? vt13.wr : yaw_feedforward;
    leg_protocol.trigger_jump = vt13.key.e || vt13.rc.fn;

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
