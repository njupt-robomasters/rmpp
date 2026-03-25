#include "Sentry.hpp"
#include "motor/GM6020.hpp"

extern GM6020 yaw2;

void Sentry::OnLoop() {
    Robot::OnLoop();

    // 比赛开始，并且为哨兵
    is_game = device.referee.game.game_progress == Referee::GAMING && (device.referee.robot.id == 7 || device.referee.robot.id == 107);
    const bool is_15s = device.referee.game.game_progress == Referee::REFEREE_SELF_CHECK;

    // 比赛中，或按下测试按钮
    if (is_game || device.rc.is_fn) {
        handleChassis();
        handleGimbal();
        handleShooter();
    } else if (is_15s) {
        handle15s();
    } else {
        handlePause();
    }
}

void Sentry::handlePause() {
    // 状态机
    chassis_status = GO_CENTER;
    gimbal_status = SCAN;

    // 底盘
    vx.software = vy.software = wr.software = 0 * default_unit;

    // 云台
    gimbal_mode.software = GIMBAL_SPEED_MODE;
    wyaw.software = wpitch.software = 0 * default_unit;
    yaw.software = pitch.software = 0 * default_unit;

    // 发射机构
    is_rub.software = is_shoot.software = false;
}

void Sentry::handle15s() {
    // 云台速度模式
    gimbal_mode.software = GIMBAL_SPEED_MODE;

    // yaw旋转
    wyaw.software = YAW_SPEED;

    // pitch停止
    wpitch.software = 0 * default_unit;
}

void Sentry::handleChassis() {
    // 导航
    switch (chassis_status) {
        case GO_CENTER: {
            if (is_game) {
                device.mavlink.target_position.x = GAME_CENTER_X;
                device.mavlink.target_position.y = GAME_CENTER_Y;
            } else {
                device.mavlink.target_position.x = TEST_CENTER_X;
                device.mavlink.target_position.y = TEST_CENTER_Y;
            }
            std::tie(vx.software, vy.software) = rotate(device.mavlink.chassis_speed.vx, device.mavlink.chassis_speed.vy, yaw2.angle.measure);

            // 血量低于设定值回家
            if (device.referee.robot.hp <= GO_HOME_HP) {
                chassis_status = GO_HOME;
            }
            break;
        }

        case GO_HOME: {
            device.mavlink.target_position.x = 0 * m;
            device.mavlink.target_position.y = 0 * m;
            std::tie(vx.software, vy.software) = rotate(device.mavlink.chassis_speed.vx, device.mavlink.chassis_speed.vy, yaw2.angle.measure);

            // 血量恢复到设定值去中心点
            if (device.referee.robot.hp >= GO_CENTER_HP) {
                chassis_status = GO_CENTER;
            }
        }
        break;
    }

    // 小陀螺
    const UnitFloat<m_s> v_software = sqrt(square(vx.software) + square(vy.software));
    // 导航速度越快，小陀螺越慢
    wr.software = map(v_software, 0 * m_s, config.vxy_max, config.wr_max, device.chassis.config.reserve_wr);
}

void Sentry::handleGimbal() {
    switch (gimbal_status) {
        case SCAN: {
            // 云台速度模式
            gimbal_mode.software = GIMBAL_SPEED_MODE;

            // yaw旋转
            wyaw.software = YAW_SPEED;

            // pitch上下摆动
            static bool is_up = false;
            if (is_up) {
                wpitch.software = PITCH_SPEED;
                if (device.gimbal.pitch.imu.measure > PITCH_MAX) is_up = false;
            } else {
                wpitch.software = -PITCH_SPEED;
                if (device.gimbal.pitch.imu.measure < PITCH_MIN) is_up = true;
            }

            // 自瞄检测到目标
            if (device.mavlink.vision.is_detect) {
                gimbal_status = LOCK;
                dwt_lock_lost.UpdateDT();
            }

            // insta360检测到目标，且不在惩罚时间内
            if (!std::isnan(device.mavlink.insta360.a0) && dwt_insta360_punish.GetDT() > INSTA360_PUNISH_TIME) {
                // 云台角度模式
                gimbal_mode.software = GIMBAL_ANGLE_MODE;

                // 转到insta360的位置
                yaw.software = device.mavlink.insta360.a0 * deg + device.gimbal.yaw.imu_minus_ecd;
                pitch.software = (PITCH_MIN + PITCH_MAX) / 2;

                gimbal_status = INSTA360_CHECK;
                dwt_insta360_check.UpdateDT();
            }
        }
        break;

        case INSTA360_CHECK: {
            // 视觉检测到目标
            if (device.mavlink.vision.is_detect) {
                gimbal_status = LOCK;
            }

            // 检测超时
            if (dwt_insta360_check.GetDT() > INSTA360_CHECK_TIMEOUT) {
                gimbal_status = SCAN;
                dwt_insta360_punish.UpdateDT(); // insta360惩罚
            }
        }
        break;

        case LOCK: {
            if (device.mavlink.vision.is_detect) { // 自瞄识别到
                dwt_lock_lost.UpdateDT();

                // 云台角度模式
                gimbal_mode.software = GIMBAL_ANGLE_MODE;

                // 响应自瞄
                yaw.software = device.mavlink.vision.yaw;
                pitch.software = device.mavlink.vision.pitch;
            } else if (dwt_lock_lost.GetDT() > LOCK_TIMEOUT) { // 自瞄丢失超过一定时间
                gimbal_status = SCAN;
            }
        }
        break;
    }
}

void Sentry::handleShooter() {
    // 比赛自动开摩擦轮
    is_rub.software = is_game;

    // 自动火控
    is_shoot.software = device.mavlink.vision.is_fire;
}