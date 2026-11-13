#include "app.hpp"
#include "bsp/timer.hpp"
#include "melody/music/g5g5g5.hpp"
#include <algorithm>
#include <cmath>

// 操控参数
static constexpr float WYAW_MAX = 360 * deg_s;   // yaw 最大角速度，rad/s
static constexpr float WPITCH_MAX = 360 * deg_s; // pitch 最大角速度，rad/s
static constexpr float AYAW = 3600 * deg_s;      // 鼠标 yaw 最大角加速度，rad/s^2
static constexpr float APITCH = 3600 * deg_s;    // 鼠标 pitch 最大角加速度，rad/s^2

// 自主扫描与锁定参数
static constexpr float YAW_SPEED = 180 * deg_s, PITCH_SPEED = 60 * deg_s;
static constexpr Angle PITCH_MIN = -22 * deg, PITCH_MAX = 0 * deg;
static constexpr float LOCK_DISTANCE = 5, LOCK_TIMEOUT = 500 * ms;
static constexpr float INSTA360_CHECK_TIMEOUT = 1, INSTA360_PUNISH_TIME = 0;
static constexpr int GO_HOME_HP = 120;

enum GimbalState { SCAN,
                   INSTA360_CHECK,
                   LOCK };

// 跨周期状态
static GimbalState gimbal_state = SCAN;
static Timer lock_lost_timer, insta360_check_timer, insta360_punish_timer;
static Angle target_yaw, target_pitch;
static bool scan_up = false;
static Timer loop_timer;
static float mouse_wyaw = 0, mouse_wpitch = 0;
static bool last_calib = false, last_r = false;

void gimbal_loop() {
    const float dt = loop_timer.restart();
    gimbal.set_enable(sentry_enable());

    // yaw 零点校准：VT13 C 挡按住 fn 和 photo，或 FSi6X SWC 上、SWA 下、SWD 下，按下时触发一次
    const bool calib = (vt13.mode == VT13::C && vt13.fn && vt13.photo) ||
                       (fsi6x.swc == FSi6X::UP && fsi6x.swa == FSi6X::DOWN && fsi6x.swd == FSi6X::DOWN);
    if (calib && !last_calib) {
        gimbal.set_yaw_zero();
        gimbal.save_yaw_offset(kv_store);
        melody.play(G5G5G5);
    }
    last_calib = calib;

    // R 键掉头
    if (vt13.key.r && !last_r) {
        gimbal.yaw.ecd.ref += 180 * deg;
        gimbal.yaw.imu.ref += 180 * deg;
    }
    last_r = vt13.key.r;

    // 鼠标限制角加速度；断联时直接清零
    if (vt13.is_connect) {
        mouse_wyaw += std::clamp(vt13.mouse.yaw * WYAW_MAX - mouse_wyaw, -AYAW * dt, AYAW * dt);
        mouse_wpitch += std::clamp(vt13.mouse.pitch * WPITCH_MAX - mouse_wpitch, -APITCH * dt, APITCH * dt);
    } else {
        mouse_wyaw = mouse_wpitch = 0;
    }

    // 摇杆：射击开关打开时 pitch 摇杆改为调节弹频，不再控制云台
    const float yaw = vt13.yaw + fsi6x.yaw;
    float pitch = 0;
    if (!vt13.trigger)
        pitch += vt13.pitch;
    if (fsi6x.swb != FSi6X::DOWN)
        pitch += fsi6x.pitch;

    // 自主扫描与锁定，输出角度目标或叠加速度
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
            if (aim.vision.is_detect && aim.vision.distance <= LOCK_DISTANCE) {
                gimbal_state = LOCK;
                target_yaw = aim.vision.yaw;
                target_pitch = aim.vision.pitch;
                lock_lost_timer.restart();
            }
            if (!std::isnan(aim.insta360.a0) &&
                insta360_punish_timer.elapsed() > INSTA360_PUNISH_TIME) {
                insta360_punish_timer.restart();
                target_yaw = aim.insta360.a0 + gimbal.yaw.imu_minus_ecd;
                target_pitch = (PITCH_MIN + PITCH_MAX) / 2;
                angle_mode = true;
                gimbal_state = INSTA360_CHECK;
                insta360_check_timer.restart();
            }
            break;
        case INSTA360_CHECK:
            angle_mode = true;
            if (aim.vision.is_detect && aim.vision.distance <= LOCK_DISTANCE) {
                gimbal_state = LOCK;
                target_yaw = aim.vision.yaw;
                target_pitch = aim.vision.pitch;
                lock_lost_timer.restart();
            }
            if (insta360_check_timer.elapsed() > INSTA360_CHECK_TIMEOUT) {
                insta360_check_timer.restart();
                gimbal_state = SCAN;
                angle_mode = false;
                insta360_punish_timer.restart();
            }
            break;
        case LOCK:
            angle_mode = true;
            if (referee.robot_status.current_HP <= GO_HOME_HP) {
                gimbal_state = SCAN;
                angle_mode = false;
            } else if (aim.vision.is_detect && aim.vision.distance <= LOCK_DISTANCE) {
                target_yaw = aim.vision.yaw;
                target_pitch = aim.vision.pitch;
                lock_lost_timer.restart();
            } else if (lock_lost_timer.elapsed() > LOCK_TIMEOUT) {
                lock_lost_timer.restart();
                gimbal_state = SCAN;
                angle_mode = false;
            }
            break;
        }
    } else {
        gimbal_state = SCAN;
        scan_up = false;
        target_yaw = target_pitch = 0 * deg;
        if (referee.game_status.game_progress == 2)
            wyaw = YAW_SPEED;
    }

    // 人工自瞄优先，其次为自主角度目标，最后叠加速度
    // 自瞄：VT13 photo、FSi6X SWD 下或按住鼠标右键，识别到目标时跟随视觉角度
    const bool is_auto_aim = vt13.photo || fsi6x.swd == FSi6X::DOWN || vt13.mouse.right;
    wyaw += yaw * WYAW_MAX + mouse_wyaw;
    wpitch += pitch * WPITCH_MAX + mouse_wpitch;
    if (is_auto_aim && aim.vision.is_detect)
        gimbal.set_angle(aim.vision.yaw, aim.vision.pitch);
    else if (angle_mode)
        gimbal.set_angle(target_yaw, target_pitch);
    else
        gimbal.set_speed(std::clamp(wyaw, -WYAW_MAX, WYAW_MAX), std::clamp(wpitch, -WPITCH_MAX, WPITCH_MAX));
    gimbal.set_chassis_wr(chassis.wr.measure);
    attitude.update();
    gimbal.update(attitude.yaw, attitude.pitch);
}
