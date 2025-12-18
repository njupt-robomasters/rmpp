#include "robot.hpp"

void Robot::OnLoop() {
    // 控制器
    handle_disconnect();
    handle_dj6();
    handle_vt13();
    handle_mavlink();

    // 传感器
    handle_imu();

    // 执行器
    handle_chassis();
    handle_gimbal();
    handle_shooter();

    // 裁判系统交互
    handle_referee();
    handle_ui();
}

void Robot::SetEnable(const bool is_enable) {
    chassis.SetEnable(is_enable);
    gimbal.SetEnable(is_enable);
    shooter.SetEnable(is_enable);
}

void Robot::handle_disconnect() {
    // 检查遥控器断联
    if (vt13.is_connected) {
        switch (vt13.mode) {
            case VT13::C:
                SetEnable(false);
                break;
            case VT13::N:
                SetEnable(true);
                break;
            case VT13::S:
                SetEnable(true);
                break;
            case VT13::ERR:
                SetEnable(false);
                break;
        }
    } else if (dj6.is_connected) {
        SetEnable(true);
    } else {
        SetEnable(false);
    }
}

void Robot::handle_dj6() {
    vx.rc = dj6.x * config.vxy_max;
    vy.rc = dj6.y * config.vxy_max;
    yaw_speed.rc = dj6.yaw * config.yaw_max;
    pitch_speed.rc = dj6.pitch * config.pitch_max;

    // 左拨杆：发射结构
    static DJ6::switch_e switch_left_last = DJ6::ERR;
    if (switch_left_last != dj6.switch_left) { // 状态改变才处理
        if (switch_left_last != DJ6::ERR) {    // 刚连上不算
            switch (dj6.switch_left) {
                case DJ6::ERR:
                    break;
                case DJ6::UP:
                    shooter.SetPrepareShoot(false);
                    shooter.SetShoot(false);
                    break;
                case DJ6::MID:
                    shooter.SetPrepareShoot(true);
                    shooter.SetShoot(false);
                    break;
                case DJ6::DOWN:
                    shooter.SetPrepareShoot(true);
                    shooter.SetShoot(true);
                    break;
            }
        }
    }
    switch_left_last = dj6.switch_left;

    // 右拨杆：底盘模式、测试小陀螺
    static DJ6::switch_e switch_right_last = DJ6::ERR;
    if (switch_right_last != dj6.switch_right) { // 状态改变才处理
        switch (dj6.switch_right) {
            case DJ6::ERR:
                break;
            case DJ6::UP:
                chassis.SetMode(Chassis_Template::FOLLOW_MODE);
                vr.rc = 0 * default_unit;
                break;
            case DJ6::MID:
                chassis.SetMode(Chassis_Template::DETACH_MODE);
                vr.rc = 0 * default_unit;
                break;
            case DJ6::DOWN:
                chassis.SetMode(Chassis_Template::DETACH_MODE);
                if (switch_right_last != DJ6::ERR) { // 刚连上不算
                    vr.rc = config.vr_max;
                }
                break;
        }
    }
    switch_right_last = dj6.switch_right;

    dj6.OnLoop();
}

void Robot::handle_vt13() {
    // 遥控器
    // 摇杆
    vx.vt13 = vt13.x * config.vxy_max;
    vy.vt13 = vt13.y * config.vxy_max;
    vr.vt13 = -vt13.wheel * config.vr_max;
    yaw_speed.vt13 = vt13.yaw * config.yaw_max;
    pitch_speed.vt13 = vt13.pitch * config.pitch_max;

    // 左fn键 -> 分离模式
    static bool fn_left_last = false;
    if (vt13.fn_left && fn_left_last != vt13.fn_left) { // 按下
        chassis.SetMode(Chassis_Template::DETACH_MODE);
    }
    fn_left_last = vt13.fn_left;

    // 右fn键 -> 跟随模式
    static bool fn_right_last = false;
    if (vt13.fn_right && fn_right_last != vt13.fn_right) { // 按下
        chassis.SetMode(Chassis_Template::FOLLOW_MODE);
    }
    fn_right_last = vt13.fn_right;

    // S挡开摩擦轮
    static VT13::mode_e mode_last = VT13::mode_e::ERR;
    if (mode_last != vt13.mode) {     // 状态改变才处理
        if (mode_last != VT13::ERR) { // 刚连上不算
            switch (vt13.mode) {
                case VT13::ERR:
                    break;
                case VT13::C:
                    shooter.SetPrepareShoot(false);
                    break;
                case VT13::N:
                    shooter.SetPrepareShoot(false);
                    break;
                case VT13::S:
                    shooter.SetPrepareShoot(true);
                    break;
            }
        }
        mode_last = vt13.mode;
    }

    // 扳机键射击
    static bool trigger_last = false;
    if (trigger_last != vt13.trigger) { // 状态改变才处理
        if (shooter.is_prepare_shoot) {
            shooter.SetShoot(vt13.trigger);
        }
    }
    trigger_last = vt13.trigger;

    // 客户端
    // wsad控制底盘
    // ws控制前后
    static BSP::Dwt dwt;
    const float dt = dwt.UpdateDT();
    if (vt13.key.w) {
        if (vx.client < 0) vx.client = 0 * default_unit;
        vx.client += config.axy * dt;
        vx.client = unit::clamp(vx.client, config.vxy_max);
    } else if (vt13.key.s) {
        if (vx.client > 0) vx.client = 0 * default_unit;
        vx.client -= config.axy * dt;
        vx.client = unit::clamp(vx.client, config.vxy_max);
    } else {
        if (vx.client > 0) {
            vx.client -= config.dxy * dt;
            if (vx.client < 0) vx.client = 0 * default_unit;
        }
        if (vx.client < 0) {
            vx.client += config.dxy * dt;
            if (vx.client > 0) vx.client = 0 * default_unit;
        }
    }
    // ad控制水平
    if (vt13.key.a) {
        if (vy.client < 0) vy.client = 0 * default_unit;
        vy.client += config.axy * dt;
        vy.client = unit::clamp(vy.client, config.vxy_max);
    } else if (vt13.key.d) {
        if (vy.client > 0) vy.client = 0 * default_unit;
        vy.client -= config.axy * dt;
        vy.client = unit::clamp(vy.client, config.vxy_max);
    } else {
        if (vy.client > 0) {
            vy.client -= config.dxy * dt;
            if (vy.client < 0) vy.client = 0 * default_unit;
        }
        if (vy.client < 0) {
            vy.client += config.dxy * dt;
            if (vy.client > 0) vy.client = 0 * default_unit;
        }
    }
    // f刷新UI
    static bool f_last = false;
    if (vt13.key.f && f_last != vt13.key.f) {
        ui.Init();
    }

    // 鼠标控制云台
    yaw_speed.client = vt13.mouse_yaw * config.yaw_max;
    pitch_speed.client = vt13.mouse_pitch * config.pitch_max;

    // 鼠标左键开火
    static bool mouse_left_last = false;
    if (mouse_left_last != vt13.mouse_left) { // 状态改变才处理
        shooter.SetShoot(vt13.mouse_left);
    }
    mouse_left_last = vt13.mouse_left;

    // 鼠标右键自瞄
    if (vt13.mouse_right) {
        yaw_speed.client = 0 * default_unit;
        pitch_speed.client = 0 * default_unit;
        gimbal.SetAngle(mavlink.aim.yaw, mavlink.aim.pitch);
    }

    // todo: 鼠标滚轮和中键控制小陀螺

    vt13.OnLoop();
}

void Robot::handle_referee() {
    static uint32_t last_hit_cnt = 0;
    if (last_hit_cnt != referee.hit_cnt) {
        hit.dwt.UpdateDT();
        switch (referee.hurt_data.armor_id) {
            case 0:
                hit.yaw_imu = gimbal.yaw.imu.measure - gimbal.yaw.ecd.measure;
                break;
            case 1:
                hit.yaw_imu = gimbal.yaw.imu.measure - gimbal.yaw.ecd.measure + 90 * deg;
                break;
            case 2:
                hit.yaw_imu = gimbal.yaw.imu.measure - gimbal.yaw.ecd.measure + 180 * deg;
                break;
            case 3:
                hit.yaw_imu = gimbal.yaw.imu.measure - gimbal.yaw.ecd.measure + 270 * deg;
                break;
            default:
                break;
        }
    }
    last_hit_cnt = referee.hit_cnt;

    referee.OnLoop();
}

void Robot::handle_mavlink() {
    mavlink.imu = {
        .yaw = imu.yaw,
        .pitch = imu.pitch,
        .roll = imu.roll
    };
    mavlink.referee = {
        .is_red = referee.robot_status.robot_id < 100,
        .bullet_speed = 24 * m_s
    };

    mavlink.OnLoop();
}

void Robot::handle_imu() {
    imu.OnLoop();
}

void Robot::handle_chassis() {
    // 设置底盘速度
    vx.sum = unit::clamp(vx.rc + vx.vt13 + vx.client + vx.nav, config.vxy_max);
    vy.sum = unit::clamp(vy.rc + vy.vt13 + vy.client + vy.nav, config.vxy_max);
    vr.sum = unit::clamp(vr.rc + vr.vt13 + vr.client + vr.nav, config.vr_max);
    chassis.SetSpeed(vx.sum, vy.sum, vr.sum);

    // 设置云台方向
    chassis.SetGimbalYaw(gimbal.yaw.ecd.measure);

    chassis.OnLoop();
}

void Robot::handle_gimbal() {
    // 设置云台速度
    pitch_speed.sum = unit::clamp(pitch_speed.rc + pitch_speed.vt13 + pitch_speed.client + pitch_speed.nav, config.pitch_max);
    yaw_speed.sum = unit::clamp(yaw_speed.rc + yaw_speed.vt13 + yaw_speed.client + yaw_speed.nav, config.yaw_max);
    gimbal.SetSpeed(yaw_speed.sum, pitch_speed.sum);

    // 设置小陀螺前馈
    gimbal.SetChassisVR(chassis.wr.measure);

    gimbal.OnLoop();
}

void Robot::handle_shooter() {
    shooter.SetBulletSpeed(config.bullet_speed); // 设置弹速
    shooter.SetBulletFreq(config.bullet_freq);   // 设置弹频

    shooter.OnLoop();
}

void Robot::handle_ui() {
    ui.is_detected = mavlink.aim.is_detected;
    ui.gimbal_yaw = gimbal.yaw.ecd.measure;
    // if (hit.dwt.GetDT() < HIT_TIMEOUT) {
    //     ui.is_hit = true;
    //     ui.hit = hit.yaw_imu - gimbal.yaw.imu.measure;
    // } else {
    //     ui.is_hit = false;
    // }
    ui.is_hit = true;
    ui.hit = hit.yaw_imu - gimbal.yaw.imu.measure;

    ui.OnLoop();
}