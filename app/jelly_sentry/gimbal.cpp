#include "gimbal.hpp"
#include "app.hpp"
#include "bsp/irq_lock.hpp"
#include "melody/music/g5g5g5.hpp"
#include "minipc.hpp"
#include "sentry.hpp"
#include <algorithm>

static bool last_calib = false, last_r = false;

void gimbal_loop() {
    // aim 在 CDC 中断内更新，整体读取保证一致性
    const auto [is_detect, target_yaw, target_pitch, distance] = [] {
        const IrqLock lock;
        return std::tuple{aim.is_detect, aim.yaw, aim.pitch, aim.distance};
    }();

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

    const bool is_auto = sentry_official() || sentry_active();
    if (is_auto) {
        sentry_gimbal(is_detect, target_yaw, target_pitch, distance);
    } else {
        // VT13：R 键掉头；photo 或右键自瞄且识别到目标时跟随视觉角度
        if (vt13.key.r && !last_r)
            gimbal.turn_yaw(180 * deg);
        last_r = vt13.key.r;

        if (vt13.is_aim && is_detect) {
            gimbal.set_angle(target_yaw, target_pitch);
        } else {
            const float yaw_speed = vt13.yaw_speed + sentry_self_check_yaw_speed();
            const float yaw_speed_max = vt13.config.yaw_speed_max;
            gimbal.set_speed(std::clamp(yaw_speed, -yaw_speed_max, yaw_speed_max), -vt13.pitch_speed);
        }
    }

    // 允许交战且识别到装甲板时小 yaw 优先；过洞断联保持原分配模式
    if (!is_auto || nav.is_connect || !is_nav_through_hole)
        gimbal.set_yaw_window((!is_auto || sentry_can_engage()) && is_detect);
    gimbal.update(attitude.q, imu.gyro);
}
