#include "gimbal.hpp"
#include "app.hpp"
#include "melody/music/g5g5g5.hpp"
#include "minipc.hpp"
#include "sentry.hpp"
#include <algorithm>

static bool last_calib = false, last_r = false;

void gimbal_loop() {
    // aim 在 CDC 接收中断内解析，逐字段读取
    const bool is_detect = aim.is_detect;
    const Angle target_yaw = aim.yaw, target_pitch = aim.pitch;
    const float distance = aim.distance;

    gimbal.set_enable(sentry_enable());

    // yaw 零点校准：C 挡按住 fn 和 photo，按下沿触发；更新 offset 后写入 KvStore
    const bool calib = vt13.rc.mode == VT13::C && vt13.rc.fn && vt13.rc.photo;
    if (calib && !last_calib) {
        if (yaw1.is_connect)
            yaw1.config.offset += (yaw1.config.is_invert ? -yaw1.angle_multi : yaw1.angle_multi) * yaw1.config.reduction;
        kv_store.write("yaw_offset", yaw1.config.offset);
        melody.play(G5G5G5);
    }
    last_calib = calib;

    if (sentry_active()) {
        sentry_gimbal(is_detect, target_yaw, target_pitch, distance);
    } else {
        // VT13：R 键掉头；photo 或鼠标右键自瞄且识别到目标时跟随视觉角度
        // 否则取 -pitch_speed，保持摇杆上推目标增大的方向；裁判自检阶段 yaw 叠加匀速旋转
        if (vt13.key.r && !last_r)
            gimbal.turn_yaw(180 * deg);
        last_r = vt13.key.r;

        if (vt13.is_aim && is_detect) {
            gimbal.set_angle(target_yaw, target_pitch);
        } else {
            float yaw_speed = vt13.yaw_speed + sentry_self_check_yaw_speed();
            const float yaw_speed_max = vt13.config.yaw_speed_max;
            gimbal.set_speed(std::clamp(yaw_speed, -yaw_speed_max, yaw_speed_max), -vt13.pitch_speed);
        }
    }

    gimbal.update(attitude.q, imu.gyro);
}
