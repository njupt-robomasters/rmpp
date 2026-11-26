#include "gimbal.hpp"
#include "app.hpp"
#include "chassis.hpp"
#include "melody/music/g5g5g5.hpp"
#include <algorithm>
#include <cmath>

// 自主扫描：yaw 连续旋转，pitch 目标在编码器软限位间往返，rad/s
static constexpr float YAW_SPEED = 180 * deg_s, PITCH_SPEED = 60 * deg_s;

// 自主锁定：自瞄有效距离，m；脱锁时间，s；不高于此血量放弃锁定
static constexpr float LOCK_DISTANCE = 5, LOCK_TIMEOUT = 500 * ms;
static constexpr int GO_HOME_HP = 120;

// 自主状态，退出自主时复位为向下扫描
static bool is_lock = false, scan_up = false;
static Angle lock_yaw, lock_pitch; // 锁定目标，IMU 系
static Timer lock_lost_timer;      // 识别到有效目标时重新开始

static bool last_calib = false, last_r = false;

// VT13：摇杆叠加鼠标；pitch 取 -pitch_speed，保持摇杆上推目标增大的方向；R 键掉头
// photo 或鼠标右键自瞄，识别到目标时跟随视觉角度；裁判自检阶段 yaw 叠加匀速旋转
static void control_by_vt13(const bool is_detect, const Angle target_yaw, const Angle target_pitch) {
    if (vt13.key.r && !last_r)
        gimbal.turn_yaw(180 * deg);
    last_r = vt13.key.r;

    if (vt13.is_aim && is_detect) {
        gimbal.set_angle(target_yaw, target_pitch);
    } else {
        float yaw_speed = vt13.yaw_speed;
        if (referee.game_status.game_progress == Referee::REFEREE_SELF_CHECK)
            yaw_speed += YAW_SPEED;
        const float yaw_speed_max = vt13.config.yaw_speed_max;
        gimbal.set_speed(std::clamp(yaw_speed, -yaw_speed_max, yaw_speed_max), -vt13.pitch_speed);
    }
}

// 自主：摇杆或鼠标转动云台时人工优先；松开后有效目标锁定，脱锁超时或低血量时过洞停扫、其余扫描
static void control_by_auto(const bool is_detect, const Angle target_yaw, const Angle target_pitch, const float distance) {
    if (vt13.yaw_speed != 0 || vt13.pitch_speed != 0) {
        is_lock = false;
        gimbal.set_speed(vt13.yaw_speed, -vt13.pitch_speed);
        return;
    }

    const bool has_target = is_detect && distance <= LOCK_DISTANCE;
    const bool is_low_hp = referee.robot_status.current_HP <= GO_HOME_HP;
    if (has_target && !is_low_hp) {
        is_lock = true;
        lock_yaw = target_yaw;
        lock_pitch = target_pitch;
        lock_lost_timer.restart();
    } else if (is_low_hp || lock_lost_timer.elapsed() > LOCK_TIMEOUT) {
        is_lock = false;
    }

    if (is_lock) {
        gimbal.set_angle(lock_yaw, lock_pitch);
    } else if (nav.is_through_hole) {
        gimbal.set_speed(0, 0);
    } else {
        if (gimbal.pitch.ecd.ref >= PITCH_MAX)
            scan_up = false;
        if (gimbal.pitch.ecd.ref <= PITCH_MIN)
            scan_up = true;
        gimbal.set_speed(YAW_SPEED, scan_up ? PITCH_SPEED : -PITCH_SPEED);
    }
}

void gimbal_loop(const bool is_detect, const Angle target_yaw, const Angle target_pitch, const float distance) {
    gimbal.set_enable(sentry_enable());

    // yaw 零点校准：VT13 C 挡按住 fn 和 photo，按下时触发一次
    // 按大 yaw 多圈角更新 offset，使单圈与多圈反馈同时归零，写入 KvStore，蜂鸣器响 G5G5G5
    const bool calib = vt13.rc.mode == VT13::C && vt13.rc.fn && vt13.rc.photo;
    if (calib && !last_calib) {
        if (yaw1.is_connect)
            yaw1.config.offset += (yaw1.config.is_invert ? -yaw1.angle_multi : yaw1.angle_multi) * yaw1.config.reduction;
        kv_store.write("yaw_offset", yaw1.config.offset);
        melody.play(G5G5G5);
    }
    last_calib = calib;

    if (sentry_active()) {
        control_by_auto(is_detect, target_yaw, target_pitch, distance);
    } else {
        is_lock = scan_up = false;
        control_by_vt13(is_detect, target_yaw, target_pitch);
    }

    // 识别到装甲板时小 yaw 优先追踪（窗口模式）
    gimbal.set_yaw_window(is_detect);
    // 小陀螺前馈；底盘转速未知时关闭
    gimbal.set_chassis_wr(std::isfinite(chassis.wr.measure) ? chassis.wr.measure : 0);
    gimbal.update(attitude.yaw, attitude.pitch);
}
