#include "sentry.hpp"
#include "app.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "minipc.hpp"
#include "shooter.hpp"
#include <random>

// 回家途中，sentry_chassis() 维护
static bool is_go_home = false;

void sentry_chassis() {
    // 战术点位：按血量锁存回家，回满 maximum_HP 再出击；未收到最大血量时保持锁存
    const uint16_t max_hp = referee.robot_status.maximum_HP;
    const uint16_t hp = referee.robot_status.current_HP;
    if (max_hp != 0) {
        if (hp < GO_HOME_HP)
            is_go_home = true;
        else if (is_go_home && hp >= max_hp)
            is_go_home = false;
    }

    // 导航目标：回家途中去回家点，否则去中心增益点
    if (is_go_home)
        nav.set_target(HOME_X, HOME_Y);
    else
        nav.set_target(CENTER_X, CENTER_Y);

    // 平移：雷达系按负小 yaw 角转到云台头部系，叠加以头部系为参考的遥控平移，不额外限幅
    const auto [nav_vx, nav_vy] = rotate(nav.vx, nav.vy, -gimbal.yaw2.measure);

    // 转速：受击后短时间内提高，按最后一次弹丸受击刷新
    chassis.set_speed(nav_vx, nav_vy, time_since_hit() < SPIN_HIT_TIME ? SPIN_SPEED_HIT : SPIN_SPEED);
}

// pitch 扫描速度倍率：标准库 LCG 取 0.5～1，只放慢不超过额定值；换值让 pitch 与 yaw 的周期不成固定比例，避免每圈同一朝向的死区
static float next_pitch_scale() {
    static std::minstd_rand rng;
    return std::uniform_real_distribution<float>(0.5f, 1.0f)(rng);
}

void sentry_gimbal() {
    // 自瞄状态：锁定标志、锁定角、脱锁计时、扫描方向与速度倍率
    static bool is_lock = false, scan_up = false;
    static Angle lock_yaw, lock_pitch;
    static Timer lock_lost_timer;
    static float scan_scale = 1;

    // 自瞄：允许交战且 5 m 内识别到目标时更新锁定，丢识别后保持最多 500 ms
    if (aim.is_detect && aim.distance <= LOCK_DISTANCE && !is_go_home) {
        is_lock = true;
        lock_yaw = aim.yaw;
        lock_pitch = aim.pitch;
        lock_lost_timer.restart();
    } else if (lock_lost_timer.elapsed() > LOCK_TIMEOUT || is_go_home) {
        is_lock = false;
    }

    // 锁定：小 yaw 优先跟随，大 yaw 保持世界朝向；解锁扫描由大 yaw 承担转速，此处写回每周期状态
    gimbal.set_yaw_window(is_lock);
    if (is_lock) {
        gimbal.set_angle(lock_yaw, lock_pitch);
        return;
    }

    // 扫描：yaw 匀速，pitch 在编码器软限位间往返，每到端点换一次速度倍率
    if (gimbal.pitch.ecd_ref >= PITCH_MAX) {
        scan_up = false;
        scan_scale = next_pitch_scale();
    } else if (gimbal.pitch.ecd_ref <= PITCH_MIN) {
        scan_up = true;
        scan_scale = next_pitch_scale();
    }
    if (scan_up)
        gimbal.set_speed(SCAN_YAW_SPEED, SCAN_PITCH_SPEED * scan_scale);
    else
        gimbal.set_speed(SCAN_YAW_SPEED, -SCAN_PITCH_SPEED * scan_scale);
}

void sentry_shooter() {
    // 场下测试：S 挡手动开摩擦轮，S 挡同时作为开火批准
    if (is_sentry_test()) {
        shooter.set_bullet_speed(vt13.bullet_speed);
        if (aim.is_fire)
            shooter.set_bullet_freq(vt13.bullet_freq);
        else
            shooter.set_bullet_freq(0);
    } else {
        // 比赛：自动开摩擦轮，未在回家途中且视觉许可时拨弹
        shooter.set_bullet_speed(vt13.config.bullet_speed);
        if (!is_go_home && aim.is_fire)
            shooter.set_bullet_freq(vt13.config.bullet_freq);
        else
            shooter.set_bullet_freq(0);
    }
}
