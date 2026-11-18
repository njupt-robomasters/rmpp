#include "chassis.hpp"
#include "app.hpp"
#include "gimbal.hpp"

// 下发指令，chassis_loop() 写入、chassis_write_can_frame() 组帧
static LegUpFrame up;
static bool down_is_connect = false;
static Timer callback_timer;

void chassis_loop() {
    // 超级电容
    super_cap.set_mode(SuperCap::OFF);
    super_cap.update();

    // 下板断联检测
    if (callback_timer.elapsed() > LEG_TIMEOUT) {
        down_is_connect = false;
        down = {};
    }

    up.is_enable = vt13.is_connect && (vt13.mode == VT13::N || vt13.mode == VT13::S) && down_is_connect;
    up.trigger_jump = vt13.fn;
    up.vx = vt13.x * std::cos(yaw.angle) * VX_MAX;
    up.wr = vt13.wheel * WR_MAX;
    up.yaw = yaw.angle;
    up.power_ref = super_cap.power_chassis_ref;

    static bool last_pause = false;
    if (!up.is_enable) {
        up.allow_rescue = false;
        up.leg_length = 0.14f;
    } else {
        // pause：倒地时授权自救并保持，站立时切换腿长
        if (vt13.pause && !last_pause) {
            if (down.is_fallen) {
                up.allow_rescue = true;
            } else {
                if (up.leg_length < 0.22f)
                    up.leg_length = 0.22f;
                else if (up.leg_length < 0.33f)
                    up.leg_length = 0.33f;
                else
                    up.leg_length = 0.14f;
            }
        }
    }
    last_pause = vt13.pause;
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
