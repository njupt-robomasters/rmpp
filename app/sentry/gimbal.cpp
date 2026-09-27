#include "app.hpp"
#include "bsp/timer.hpp"
#include "math/math.hpp"
#include <cmath>

static constexpr float YAW_SPEED = 180 * deg_s, PITCH_SPEED = 60 * deg_s;
static constexpr Angle PITCH_MIN = -22 * deg, PITCH_MAX = 0 * deg;
static constexpr float LOCK_DISTANCE = 5, LOCK_TIMEOUT = 500 * ms;
static constexpr float INSTA360_CHECK_TIMEOUT = 1, INSTA360_PUNISH_TIME = 0;
static constexpr int GO_HOME_HP = 120;

enum GimbalState { SCAN,
                   INSTA360_CHECK,
                   LOCK };

static GimbalState gimbal_state = SCAN;
static Timer timer_lock_lost, timer_insta360_check, timer_insta360_punish;
static Angle target_yaw, target_pitch;
static bool scan_up = false;

void gimbal_loop() {
    gimbal.set_enable(rc.is_enable);
    if (rc.is_calib) {
        gimbal.set_yaw_zero();
        gimbal.save_yaw_offset(storage);
        buzzer.play(Buzzer::G5G5G5);
    }
    if (rc.is_turn_back) {
        gimbal.yaw.ecd.ref += 180 * deg;
        gimbal.yaw.imu.ref += 180 * deg;
    }
    float wyaw = 0, wpitch = 0;
    bool angle_mode = false;
    if (sentry_active()) {
        switch (gimbal_state) {
        case SCAN:
            wyaw = YAW_SPEED;
            wpitch = scan_up ? PITCH_SPEED : -PITCH_SPEED;
            if (gimbal.pitch.imu.measure > PITCH_MAX)
                scan_up = false;
            if (gimbal.pitch.imu.measure < PITCH_MIN)
                scan_up = true;
            if (mavlink.vision.is_detect && mavlink.vision.distance <= LOCK_DISTANCE) {
                gimbal_state = LOCK;
                target_yaw = mavlink.vision.yaw;
                target_pitch = mavlink.vision.pitch;
                timer_lock_lost.restart();
            }
            if (!std::isnan(mavlink.insta360.a0) &&
                timer_insta360_punish.elapsed() > INSTA360_PUNISH_TIME) {
                timer_insta360_punish.restart();
                target_yaw = mavlink.insta360.a0 + gimbal.yaw.imu_minus_ecd;
                target_pitch = (PITCH_MIN + PITCH_MAX) / 2;
                angle_mode = true;
                gimbal_state = INSTA360_CHECK;
                timer_insta360_check.restart();
            }
            break;
        case INSTA360_CHECK:
            angle_mode = true;
            if (mavlink.vision.is_detect && mavlink.vision.distance <= LOCK_DISTANCE) {
                gimbal_state = LOCK;
                target_yaw = mavlink.vision.yaw;
                target_pitch = mavlink.vision.pitch;
                timer_lock_lost.restart();
            }
            if (timer_insta360_check.elapsed() > INSTA360_CHECK_TIMEOUT) {
                timer_insta360_check.restart();
                gimbal_state = SCAN;
                angle_mode = false;
                timer_insta360_punish.restart();
            }
            break;
        case LOCK:
            angle_mode = true;
            if (referee.robot.hp <= GO_HOME_HP) {
                gimbal_state = SCAN;
                angle_mode = false;
            } else if (mavlink.vision.is_detect && mavlink.vision.distance <= LOCK_DISTANCE) {
                target_yaw = mavlink.vision.yaw;
                target_pitch = mavlink.vision.pitch;
                timer_lock_lost.restart();
            } else if (timer_lock_lost.elapsed() > LOCK_TIMEOUT) {
                timer_lock_lost.restart();
                gimbal_state = SCAN;
                angle_mode = false;
            }
            break;
        }
    } else {
        gimbal_state = SCAN;
        scan_up = false;
        target_yaw = target_pitch = 0 * deg;
        if (referee.game.game_progress == Referee::REFEREE_SELF_CHECK)
            wyaw = YAW_SPEED;
    }

    // 人工自瞄优先，其次为自主角度目标，最后叠加速度
    if (rc.is_auto_aim && mavlink.vision.is_detect)
        gimbal.set_angle(mavlink.vision.yaw, mavlink.vision.pitch);
    else if (angle_mode)
        gimbal.set_angle(target_yaw, target_pitch);
    else
        gimbal.set_speed(clamp(rc.wyaw + wyaw, rc.config.wyaw), clamp(rc.wpitch + wpitch, rc.config.wpitch));
    gimbal.set_chassis_wr(chassis.wr.measure);
    gimbal.update();
}
