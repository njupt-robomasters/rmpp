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

// 暂停自动扫描：yaw、pitch 扫描速度归零，自检 yaw 同步停止；保留目标锁定
static constexpr float YAW_SPEED = 0 * deg_s, PITCH_SPEED = 0 * deg_s;
static constexpr Angle PITCH_MIN = -22 * deg, PITCH_MAX = 0 * deg;
static constexpr float LOCK_DISTANCE = 5, LOCK_TIMEOUT = 500 * ms;           // 自瞄有效距离 m、脱锁时间 s
static constexpr float INSTA360_CHECK_TIMEOUT = 1, INSTA360_PUNISH_TIME = 0; // 全景目标检查时间、误识别惩罚时间，s
static constexpr int GO_HOME_HP = 120;                                       // 低于此血量放弃锁定

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
    // 把大 yaw 当前位置设为零点并写入 KvStore，蜂鸣器响 G5G5G5
    const bool calib = (vt13.mode == VT13::C && vt13.fn && vt13.photo) ||
                       (fsi6x.swc == FSi6X::UP && fsi6x.swa == FSi6X::DOWN && fsi6x.swd == FSi6X::DOWN);
    if (calib && !last_calib) {
        if (yaw1.is_connect)
            yaw1.config.offset += (yaw1.config.is_invert ? -yaw1.angle : yaw1.angle) * yaw1.config.reduction;
        kv_store.write("yaw_offset", yaw1.config.offset);
        melody.play(G5G5G5);
    }
    last_calib = calib;

    // R 键掉头
    if (vt13.key.r && !last_r)
        gimbal.turn_yaw(180 * deg);
    last_r = vt13.key.r;

    // 鼠标限制角加速度；断联时直接清零
    if (vt13.is_connect) {
        mouse_wyaw += std::clamp(vt13.mouse.yaw * WYAW_MAX - mouse_wyaw, -AYAW * dt, AYAW * dt);
        mouse_wpitch += std::clamp(vt13.mouse.pitch * WPITCH_MAX - mouse_wpitch, -APITCH * dt, APITCH * dt);
    } else {
        mouse_wyaw = mouse_wpitch = 0;
    }

    // S 挡自瞄时左摇杆用于调节弹频；肩键沿用 pitch 摇杆隔离
    const float yaw = vt13.yaw + fsi6x.yaw;
    float pitch = 0;
    if (!vt13.trigger && !(vt13.mode == VT13::S && vt13.photo))
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
            // 全景相机发现目标且不在惩罚时间内：转到全景角度（相对底盘）检查
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
        // 非自主模式复位状态机；自检 yaw 速度沿用当前扫描设置
        gimbal_state = SCAN;
        scan_up = false;
        target_yaw = target_pitch = 0 * deg;
        if (referee.game_status.game_progress == Referee::REFEREE_SELF_CHECK)
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

    // TODO：有效 action_flag=0 时 pitch1 抬头，=1 时折叠，待确定目标角和编码器零点
    // pitch1 当前仍在 GimbalFold 内固定失能

    // 识别到装甲板时小 yaw 优先追踪，并传入底盘旋转前馈
    gimbal.set_yaw_window(aim.vision.is_detect);
    gimbal.set_chassis_wr(chassis.wr.measure);
    gimbal.update(attitude.yaw, attitude.pitch);
}
