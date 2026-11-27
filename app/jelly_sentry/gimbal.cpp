#include "gimbal.hpp"
#include "app.hpp"
#include "melody/music/g5g5g5.hpp"
#include "minipc.hpp"
#include "sentry.hpp"

static bool last_calib = false, last_r = false;

void gimbal_loop() {
    attitude.update();
    gimbal.set_enable(vt13.is_enable);

    // yaw 零点校准：C 挡按住 fn 和 photo，按下沿触发；更新 offset 后写入 KvStore
    const bool calib = vt13.rc.mode == VT13::C && vt13.rc.fn && vt13.rc.photo;
    if (calib && !last_calib) {
        if (yaw1.is_connect)
            yaw1.config.offset += (yaw1.config.is_invert ? -yaw1.angle_multi : yaw1.angle_multi) * yaw1.config.reduction;
        kv_store.write("yaw_offset", yaw1.config.offset);
        melody.play(G5G5G5);
    }
    last_calib = calib;

    if (is_sentry_active()) {
        sentry_gimbal();
    } else {
        // VT13：R 键掉头；photo 或右键自瞄且识别到目标时跟随视觉角度，否则摇杆与鼠标速度已是限幅后的操控量
        if (vt13.key.r && !last_r)
            gimbal.turn_yaw(180 * deg);
        last_r = vt13.key.r;

        if (vt13.is_aim && aim.is_detect)
            gimbal.set_angle(aim.yaw, aim.pitch);
        else
            gimbal.set_speed(vt13.yaw_speed, vt13.pitch_speed);

        // 识别到装甲板时小 yaw 优先
        gimbal.set_yaw_window(aim.is_detect);
    }

    gimbal.update(attitude.q, imu.gyro);
}
