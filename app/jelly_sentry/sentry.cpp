#include "sentry.hpp"
#include "app.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "math/math.hpp"
#include "minipc.hpp"
#include "shooter.hpp"
#include <algorithm>
#include <cmath>

// 导航目标点，上位机坐标系，m；中心增益点与补给区回家点
static constexpr float CENTER_X = 6.0f, CENTER_Y = -3.8f;
static constexpr float HOME_X = 0.15f, HOME_Y = -0.15f;

// 返航血量：低于 100 HP 回家，回满 maximum_HP 后再出击
static constexpr uint16_t GO_HOME_HP = 100;

// 2026 通信协议 V1.2.0 0x0209 bit23：RMUL 中心增益点 RFID
static constexpr uint32_t RFID_CENTER_RMUL = 1u << 23u;

// 小陀螺：合成平移速度从零到满量程时，由 90 降到 60 rpm
static constexpr float WR_MIN = 60 * rpm, WR_MAX = 90 * rpm;

// 扫描：yaw 连续旋转，pitch 目标在编码器软限位间往返，rad/s
static constexpr float SCAN_YAW_SPEED = 180 * deg_s, SCAN_PITCH_SPEED = 60 * deg_s;

// 锁定：自瞄有效距离，m；脱锁时间，s
static constexpr float LOCK_DISTANCE = 5, LOCK_TIMEOUT = 500 * ms;

// 自主状态，退出自主及赛场扫描时复位为向下扫描
static bool is_lock = false, scan_up = false;
static Angle lock_yaw, lock_pitch;
static Timer lock_lost_timer;

void sentry_update() {
    // 战术点位：正赛或 fn 测试时按血量锁存回家，RFID 判定到达中心
    if (!sentry_active()) {
        nav_state = NavState::IDLE;
        nav.set_target(0, 0);
    } else {
        if (nav_state == NavState::IDLE)
            nav_state = NavState::GO_CENTER;

        const uint16_t max_hp = referee.robot_status.maximum_HP;
        const uint16_t hp = referee.robot_status.current_HP;
        if (max_hp != 0) {
            if (hp < GO_HOME_HP)
                nav_state = NavState::GO_HOME;
            else if (nav_state == NavState::GO_HOME && hp >= max_hp)
                nav_state = NavState::GO_CENTER;
        }

        if (nav_state != NavState::GO_HOME) {
            const bool is_at_center = referee.is_connect && (referee.rfid_status.rfid_status & RFID_CENTER_RMUL) != 0u;
            nav_state = is_at_center ? NavState::AT_CENTER : NavState::GO_CENTER;
        }

        if (nav_state == NavState::GO_HOME)
            nav.set_target(HOME_X, HOME_Y);
        else
            nav.set_target(CENTER_X, CENTER_Y);
    }

    if (!sentry_official() && !sentry_active())
        is_lock = scan_up = false;
}

void sentry_chassis() {
    // 方向反馈：任一 yaw 丢失时停车
    if (!yaw1.is_connect || !yaw2.is_connect) {
        chassis.set_mode(ChassisMatrix::DETACH_MODE);
        chassis.set_speed(0, 0, 0);
        return;
    }

    // 导航断联：平移归零；最近为过洞状态则停车，其余原地 90 rpm
    if (!nav.is_connect) {
        chassis.set_mode(ChassisMatrix::DETACH_MODE);
        chassis.set_speed(0, 0, is_nav_through_hole ? 0 : WR_MAX);
        return;
    }

    // 平移：雷达系按负小 yaw 角转到云台头部系，始终叠加遥控，不额外限幅
    const auto [nav_vx, nav_vy] = rotate(nav.vx, nav.vy, -gimbal.yaw2.measure);
    const float vx = nav_vx + vt13.vx;
    const float vy = nav_vy + vt13.vy;
    if (is_nav_through_hole) {
        chassis.set_mode(ChassisMatrix::FOLLOW_MODE);
        chassis.set_speed(vx, vy, 0);
    } else {
        chassis.set_mode(ChassisMatrix::DETACH_MODE);
        const float speed = std::clamp(std::hypot(vx, vy), 0.0f, vt13.config.vxy_max);
        chassis.set_speed(vx, vy, map(speed, 0.0f, vt13.config.vxy_max, WR_MAX, WR_MIN));
    }
}

void sentry_gimbal(const bool is_detect, const Angle target_yaw, const Angle target_pitch, const float distance) {
    // 过洞断联：清除旧锁定并保持姿态，恢复有效导航前暂停人工转动和自瞄
    if (!nav.is_connect && is_nav_through_hole) {
        is_lock = false;
        gimbal.set_speed(0, 0);
        return;
    }

    // 场下自主测试：摇杆或鼠标有输入时退出锁定，直接人工接管
    if (!sentry_official() && (vt13.yaw_speed != 0 || vt13.pitch_speed != 0)) {
        is_lock = false;
        gimbal.set_speed(vt13.yaw_speed, -vt13.pitch_speed);
        return;
    }

    // 自瞄：允许交战且 5 m 内识别到目标时更新锁定，丢识别后保持最多 500 ms
    const bool has_target = is_detect && distance <= LOCK_DISTANCE;
    const bool can_engage = sentry_can_engage();
    if (can_engage && has_target) {
        is_lock = true;
        lock_yaw = target_yaw;
        lock_pitch = target_pitch;
        lock_lost_timer.restart();
    } else if (!can_engage || lock_lost_timer.elapsed() > LOCK_TIMEOUT) {
        is_lock = false;
    }

    if (is_lock) {
        gimbal.set_angle(lock_yaw, lock_pitch);
    } else if (is_nav_through_hole) {
        gimbal.set_speed(0, 0);
    } else {
        if (gimbal.pitch.ecd_ref >= PITCH_MAX)
            scan_up = false;
        if (gimbal.pitch.ecd_ref <= PITCH_MIN)
            scan_up = true;
        gimbal.set_speed(SCAN_YAW_SPEED, scan_up ? SCAN_PITCH_SPEED : -SCAN_PITCH_SPEED);
    }
}

float sentry_self_check_yaw_speed() {
    return referee.game_status.game_progress == Referee::REFEREE_SELF_CHECK ? SCAN_YAW_SPEED : 0;
}

void sentry_shooter() {
    // 自主：摩擦轮常开；回家与赛前禁止拨弹，其余按视觉开火许可
    shooter.set_bullet_speed(vt13.config.bullet_speed);
    shooter.set_bullet_freq(sentry_can_engage() && aim.is_fire ? vt13.config.bullet_freq : 0);
}
