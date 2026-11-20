#include "chassis.hpp"
#include "app.hpp"
#include "gimbal.hpp"
#include "referee.hpp"

// 下发指令，chassis_loop() 写入、chassis_write_can_frame() 组帧
static LegUpFrame up;
bool down_is_connect = false;
static Timer callback_timer;

void chassis_loop() {
    // 超级电容：保持 OFF，按裁判功率上限与缓冲能量折算底盘功率目标；裁判离线时沿用最后值，启动默认 120 W / 60 J
    super_cap.set_mode(SuperCap::OFF);
    super_cap.set_referee_limit(referee.robot_status.chassis_power_limit);
    super_cap.set_referee_energy(referee.power_heat_data.buffer_energy);
    super_cap.update();

    // 下板断联检测
    if (callback_timer.elapsed() > LEG_TIMEOUT) {
        down_is_connect = false;
        down = {};
    }

    up.is_enable = vt13.is_enable && down_is_connect;
    up.trigger_jump = vt13.rc.fn;
    up.vx = vt13.vx * std::cos(yaw.angle);
    up.wr = vt13.wr;
    up.yaw = yaw.angle;
    up.power_ref = super_cap.power_chassis_ref;

    static bool last_pause = false;
    if (!up.is_enable) {
        up.allow_rescue = false;
        up.leg_length = LEG_LENGTH_LOW;
    } else {
        // pause：倒地时授权自救并保持，站立时切换腿长
        if (vt13.rc.pause && !last_pause) {
            up.allow_rescue = true;
            if (up.leg_length < LEG_LENGTH_MID)
                up.leg_length = LEG_LENGTH_MID;
            else if (up.leg_length < LEG_LENGTH_HIGH)
                up.leg_length = LEG_LENGTH_HIGH;
            else
                up.leg_length = LEG_LENGTH_LOW;
        }
    }
    last_pause = vt13.rc.pause;
}

void chassis_write_can_frame(CanFrame& frame) {
    up.pack(frame);
}

void chassis_can_callback(const CanFrame& frame) {
    if (!down.unpack(frame))
        return;

    callback_timer.restart();
    down_is_connect = true;
}
