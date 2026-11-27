#include "sentry.hpp"
#include "app.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "minipc.hpp"
#include "shooter.hpp"

// 导航目标点，上位机坐标系，m；中心点即增益点，回家点为补血点
static constexpr float CENTER_X = 2.0f, CENTER_Y = 1.0f; // 测试场地；比赛场地为中心 (2.2, 4.7)
static constexpr float HOME_X = 0, HOME_Y = 0;

// 血量：不高于回家值回家，回到中心值以上再去中心点，中间锁存
static constexpr uint16_t GO_HOME_HP = 120, GO_CENTER_HP = 400;

// 小陀螺转速，rad/s；功率不足时优先衰减旋转，平移优先
static constexpr float SPIN_WR = 120 * rpm;

// 扫描：yaw 连续旋转，pitch 在编码器软限位间往返，rad/s
static constexpr float SCAN_YAW_SPEED = 180 * deg_s, SCAN_PITCH_SPEED = 60 * deg_s;

// 裁判自检阶段的云台匀速旋转，rad/s
static constexpr float SELF_CHECK_YAW_SPEED = 180 * deg_s;

// 锁定：自瞄有效距离，m；脱锁时间，s
static constexpr float LOCK_DISTANCE = 5, LOCK_TIMEOUT = 500 * ms;

// 自主状态：回家与锁定锁存、扫描方向
static bool is_go_home = false, is_lock = false, scan_up = false;
static Angle lock_yaw, lock_pitch; // 锁定目标，IMU 系
static Timer lock_lost_timer;      // 识别到有效目标时重新开始

// 底盘：导航目标点按血量锁存切换，速度为大 yaw 雷达系转到云台头部系后叠加遥控平移
// 过洞 1 随头对准枪口、不叠加小陀螺；过洞 0 分离模式小陀螺叠加遥控拨轮
void sentry_chassis() {
    if (referee.robot_status.maximum_HP != 0) {
        const uint16_t hp = referee.robot_status.current_HP;
        if (hp <= GO_HOME_HP)
            is_go_home = true;
        else if (hp >= GO_CENTER_HP)
            is_go_home = false;
    }
    nav.set_target(is_go_home ? HOME_X : CENTER_X, is_go_home ? HOME_Y : CENTER_Y);

    const auto [nav_vx, nav_vy] = rotate(nav.vx, nav.vy, -gimbal.yaw2.measure);
    if (nav.is_through_hole) {
        chassis.set_mode(ChassisMatrix::FOLLOW_MODE);
        chassis.set_speed(nav_vx + vt13.vx, nav_vy + vt13.vy, vt13.wr);
    } else {
        chassis.set_mode(ChassisMatrix::DETACH_MODE);
        chassis.set_speed(nav_vx + vt13.vx, nav_vy + vt13.vy, SPIN_WR + vt13.wr);
    }
}

// 云台：有效距离内识别到目标即锁定并跟随视觉角度，脱锁超时回到扫描
// 低血量回家期间只扫描，识别到目标也不锁定、不跟随，避免主动交火
// 扫描速度叠加自检旋转与遥控 yaw/pitch 速度；锁定期间以视觉角为准，遥控不参与
void sentry_gimbal(const bool is_detect, const Angle target_yaw, const Angle target_pitch, const float distance) {
    const bool has_target = is_detect && distance <= LOCK_DISTANCE;
    if (has_target && !is_go_home) {
        is_lock = true;
        lock_yaw = target_yaw;
        lock_pitch = target_pitch;
        lock_lost_timer.restart();
    } else if (is_go_home || lock_lost_timer.elapsed() > LOCK_TIMEOUT) {
        is_lock = false;
    }

    if (is_lock) {
        gimbal.set_angle(lock_yaw, lock_pitch);
        return;
    }

    // 端点按编码器角判断，到限位后反向
    if (gimbal.pitch.ecd.ref >= PITCH_MAX)
        scan_up = false;
    if (gimbal.pitch.ecd.ref <= PITCH_MIN)
        scan_up = true;
    gimbal.set_speed(SCAN_YAW_SPEED + sentry_self_check_yaw_speed() + vt13.yaw_speed,
                     (scan_up ? SCAN_PITCH_SPEED : -SCAN_PITCH_SPEED) - vt13.pitch_speed);
}

// 裁判自检阶段的云台匀速旋转，遥控与自主叠加
float sentry_self_check_yaw_speed() {
    return referee.game_status.game_progress == Referee::REFEREE_SELF_CHECK ? SELF_CHECK_YAW_SPEED : 0;
}

// 发射机构：摩擦轮常开，视觉开火许可时按设定弹频
void sentry_shooter() {
    shooter.set_bullet_speed(vt13.config.bullet_speed);
    shooter.set_bullet_freq(aim.is_fire ? vt13.config.bullet_freq : 0);
}
