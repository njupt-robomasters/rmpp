#include "sentry.hpp"
#include "math/math.hpp"
#include "motor/gm6020.hpp"
#include <cmath>

extern GM6020 yaw2;

void Sentry::on_loop() {
    Robot::on_loop();

    // 比赛开始，并且为哨兵
    is_game_ = device.referee.game.game_progress == Referee::GAMING && device.referee.robot.type == Referee::SENTRY;
    const bool is_15s = device.referee.game.game_progress == Referee::REFEREE_SELF_CHECK;

    // 比赛中，或按下测试按钮
    if (is_game_ || device.rc.is_fn) {
        handle_chassis();
        handle_gimbal();
        handle_shooter();
    } else if (is_15s) {
        handle15s();
    } else {
        handle_pause();
    }
}

void Sentry::handle_pause() {
    // 状态机
    chassis_status_ = GO_CENTER;
    gimbal_status_ = SCAN;

    // 底盘
    vx_.software = vy_.software = wr_.software = 0.0f;

    // 云台
    gimbal_mode_.software = GIMBAL_SPEED_MODE;
    wyaw_.software = wpitch_.software = 0.0f;
    yaw_.software = pitch_.software = 0.0f;

    // 发射机构
    is_rub_.software = is_shoot_.software = false;
}

void Sentry::handle15s() {
    // 云台速度模式
    gimbal_mode_.software = GIMBAL_SPEED_MODE;

    // yaw旋转
    wyaw_.software = YAW_SPEED;

    // pitch停止
    wpitch_.software = 0.0f;
}

void Sentry::handle_chassis() {
    // 导航坐标点
    float center_x{}, center_y{}, home_x{}, home_y{};
    if (FORCE_GAME_POSITION) {
        center_x = GAME_CENTER_X;
        center_y = GAME_CENTER_Y;
        home_x = GAME_HOME_X;
        home_y = GAME_HOME_Y;
    } else if (FORCE_TEST_POSITION) {
        center_x = TEST_CENTER_X;
        center_y = TEST_CENTER_Y;
        home_x = TEST_HOME_X;
        home_y = TEST_HOME_Y;
    } else {
        if (is_game_) {
            center_x = GAME_CENTER_X;
            center_y = GAME_CENTER_Y;
            home_x = GAME_HOME_X;
            home_y = GAME_HOME_Y;
        } else {
            center_x = TEST_CENTER_X;
            center_y = TEST_CENTER_Y;
            home_x = TEST_HOME_X;
            home_y = TEST_HOME_Y;
        }
    }

    // 导航状态机
    switch (chassis_status_) {
    case GO_CENTER: {
        device.mavlink.target_position.x = center_x;
        device.mavlink.target_position.y = center_y;
        std::tie(vx_.software, vy_.software) = math::rotate(device.mavlink.chassis_speed.vx, device.mavlink.chassis_speed.vy, yaw2.angle.measure);

        // 血量低于设定值回家
        if (device.referee.robot.hp <= GO_HOME_HP) {
            chassis_status_ = GO_HOME;
        }
        break;
    }

    case GO_HOME: {
        device.mavlink.target_position.x = home_x;
        device.mavlink.target_position.y = home_y;
        std::tie(vx_.software, vy_.software) = math::rotate(device.mavlink.chassis_speed.vx, device.mavlink.chassis_speed.vy, yaw2.angle.measure);

        // 血量恢复到设定值去中心点
        if (device.referee.robot.hp >= GO_CENTER_HP) {
            chassis_status_ = GO_CENTER;
        }
    } break;
    }

    // 小陀螺
    const float v_software = std::sqrtf(math::square(vx_.software) + math::square(vy_.software));
    // 导航速度越快，小陀螺越慢
    wr_.software = math::map(v_software, 0.0f, config.vxy_max, config.wr_max, device.chassis.config.reserve_wr);
}

void Sentry::handle_gimbal() {
    switch (gimbal_status_) {
    case SCAN: {
        // 云台速度模式
        gimbal_mode_.software = GIMBAL_SPEED_MODE;

        // yaw旋转
        wyaw_.software = YAW_SPEED;

        // pitch上下摆动
        static bool is_up = false;
        if (is_up) {
            wpitch_.software = PITCH_SPEED;
            if (device.gimbal.pitch.imu.measure > PITCH_MAX)
                is_up = false;
        } else {
            wpitch_.software = -PITCH_SPEED;
            if (device.gimbal.pitch.imu.measure < PITCH_MIN)
                is_up = true;
        }

        // 自瞄检测到目标
        if (device.mavlink.vision.is_detect && device.mavlink.vision.distance <= LOCK_DISTANCE) {
            gimbal_status_ = LOCK;
            timer_lock_lost_.update_dt();
        }

        // insta360检测到目标，且不在惩罚时间内
        if (!std::isnan(device.mavlink.insta360.a0) && timer_insta360_punish_.get_dt() > INSTA360_PUNISH_TIME) {
            // 云台角度模式
            gimbal_mode_.software = GIMBAL_ANGLE_MODE;

            // 转到insta360的位置
            yaw_.software = device.mavlink.insta360.a0 + device.gimbal.yaw.imu_minus_ecd;
            pitch_.software = (PITCH_MIN + PITCH_MAX) / 2;

            gimbal_status_ = INSTA360_CHECK;
            timer_insta360_check_.update_dt();
        }
    } break;

    case INSTA360_CHECK: {
        // 视觉检测到目标
        if (device.mavlink.vision.is_detect && device.mavlink.vision.distance <= LOCK_DISTANCE) {
            gimbal_status_ = LOCK;
        }

        // 检测超时
        if (timer_insta360_check_.get_dt() > INSTA360_CHECK_TIMEOUT) {
            gimbal_status_ = SCAN;
            timer_insta360_punish_.update_dt(); // insta360惩罚
        }
    } break;

    case LOCK: {
        if (device.referee.robot.hp <= GO_HOME_HP) { // 血量低于设定值，放弃锁定
            gimbal_status_ = SCAN;
        } else {
            if (device.mavlink.vision.is_detect && device.mavlink.vision.distance <= LOCK_DISTANCE) { // 自瞄识别到
                timer_lock_lost_.update_dt();

                // 云台角度模式
                gimbal_mode_.software = GIMBAL_ANGLE_MODE;

                // 响应自瞄
                yaw_.software = device.mavlink.vision.yaw;
                pitch_.software = device.mavlink.vision.pitch;
            } else if (timer_lock_lost_.get_dt() > LOCK_TIMEOUT) { // 自瞄丢失超过一定时间
                gimbal_status_ = SCAN;
            }
        }
    } break;
    }
}

void Sentry::handle_shooter() {
    // 比赛自动开摩擦轮
    is_rub_.software = is_game_;

    // 自动火控
    is_shoot_.software = device.mavlink.vision.is_fire;
}
