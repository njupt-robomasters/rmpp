#include "robot.hpp"

void Robot::OnLoop() {
    // 控制器
    handle_disconnect();
    handle_dj6();
    handle_vt13();
    handle_client();
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

void Robot::setEnable(const bool is_enable) {
    chassis.SetEnable(is_enable);
    gimbal.SetEnable(is_enable);
    shooter.SetEnable(is_enable);
}

void Robot::handle_disconnect() {
    if (vt13.is_connected) { // VT13遥控器优先
        switch (vt13.mode) {
            case VT13::ERR:
                setEnable(false);
                break;
            case VT13::C:
                setEnable(false);
                break;
            case VT13::N:
                setEnable(true);
                break;
            case VT13::S:
                setEnable(true);
                break;
        }
    } else if (dj6.is_connected) {
        setEnable(true);
    } else {
        setEnable(false);
    }
}

void Robot::handle_dj6() {
    vx.dj6 = dj6.x * config.vxy_max;
    vy.dj6 = dj6.y * config.vxy_max;
    yaw_speed.dj6 = dj6.yaw * config.yaw_max;
    pitch_speed.dj6 = dj6.pitch * config.pitch_max;

    // 左拨杆：发射机构
    switch (dj6.switch_left) {
        case DJ6::ERR:
            is_prepare_shoot.dj6 = false;
            is_shoot.dj6 = false;
        case DJ6::UP:
            is_prepare_shoot.dj6 = false;
            is_shoot.dj6 = false;
            break;
        case DJ6::MID:
            is_prepare_shoot.dj6 = true;
            is_shoot.dj6 = false;
            break;
        case DJ6::DOWN:
            is_prepare_shoot.dj6 = true;
            is_shoot.dj6 = true;
            break;
    }

    // 右拨杆：底盘模式、测试小陀螺
    // 只响应变化量
    static DJ6::switch_e switch_right_last = DJ6::ERR;
    if (switch_right_last != dj6.switch_right) {
        switch (dj6.switch_right) {
            case DJ6::ERR:
                wr.dj6 = 0 * default_unit;
            case DJ6::UP:
                chassis.SetMode(Chassis_Template::DETACH_MODE);
                wr.dj6 = 0 * default_unit;
                break;
            case DJ6::MID:
                chassis.SetMode(Chassis_Template::FOLLOW_MODE);
                wr.dj6 = 0 * default_unit;
                break;
            case DJ6::DOWN:
                chassis.SetMode(Chassis_Template::DETACH_MODE);
                wr.dj6 = config.wr_max;
                break;
        }
    }
    switch_right_last = dj6.switch_right;

    dj6.OnLoop();
}

void Robot::handle_vt13() {
    // 摇杆
    vx.vt13 = vt13.x * config.vxy_max;
    vy.vt13 = vt13.y * config.vxy_max;
    wr.vt13 = vt13.wheel * config.wr_max;
    yaw_speed.vt13 = vt13.yaw * config.yaw_max;
    pitch_speed.vt13 = vt13.pitch * config.pitch_max;

    // 左fn键：底盘分离模式
    if (vt13.fn_left) {
        chassis.SetMode(Chassis_Template::DETACH_MODE);
    }

    // 右fn键：底盘跟随模式
    if (vt13.fn_right) {
        chassis.SetMode(Chassis_Template::FOLLOW_MODE);
    }

    // S挡：开摩擦轮
    switch (vt13.mode) {
        case VT13::ERR:
            is_prepare_shoot.vt13 = false;
            break;
        case VT13::C:
            is_prepare_shoot.vt13 = false;
            break;
        case VT13::N:
            is_prepare_shoot.vt13 = false;
            break;
        case VT13::S:
            is_prepare_shoot.vt13 = true;
            break;
    }

    // 扳机键射击
    is_shoot.vt13 = vt13.trigger;

    vt13.OnLoop();
}

void Robot::handle_client() {
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

    // f刷新ui
    if (vt13.key.f) {
        ui.Init();
    }

    // 鼠标控制云台
    yaw_speed.client = vt13.mouse_yaw * config.yaw_max;
    pitch_speed.client = vt13.mouse_pitch * config.pitch_max;

    // 鼠标左键开火
    is_shoot.client = vt13.mouse_left;

    // 鼠标右键自瞄
    if (vt13.mouse_right && mavlink.aim.is_detected) {
        yaw_speed.client = 0 * default_unit;
        pitch_speed.client = 0 * default_unit;
        gimbal.SetAngle(mavlink.aim.yaw, mavlink.aim.pitch);
    }

    // shift开小陀螺
    if (vt13.key.shift) {
        wr.client = config.wr_max;
    }
    // ctrl关小陀螺
    if (vt13.key.ctrl) {
        wr.client = 0 * default_unit;
    }
    // 失能/断联关小陀螺
    if (vt13.is_connected == false || vt13.mode == VT13::ERR || vt13.mode == VT13::C) {
        wr.client = 0 * default_unit;
    }
}

void Robot::handle_mavlink() {
    mavlink.imu = {
        .yaw = imu.yaw,
        .pitch = imu.pitch,
        .roll = imu.roll
    };
    mavlink.referee = {
        .is_red = referee.robot_status.robot_id < 100,
        .bullet_speed = referee.shoot_data.initial_speed * m_s
    };

    mavlink.OnLoop();
}

void Robot::handle_imu() {
    imu.OnLoop();
}

void Robot::handle_chassis() {
    // 设置底盘速度
    vx.sum = unit::clamp(vx.dj6 + vx.vt13 + vx.client + vx.mavlink, config.vxy_max);
    vy.sum = unit::clamp(vy.dj6 + vy.vt13 + vy.client + vy.mavlink, config.vxy_max);
    wr.sum = unit::clamp(wr.dj6 + wr.vt13 + wr.client + wr.mavlink, config.wr_max);
    chassis.SetSpeed(vx.sum, vy.sum, wr.sum);

    // 设置云台方向
    chassis.SetGimbalYaw(gimbal.yaw.ecd.measure);

    // 设置底盘功率限制
    chassis.SetPowerLimit(referee.robot_status.chassis_power_limit * W);

    chassis.OnLoop();
}

void Robot::handle_gimbal() {
    // 设置云台速度
    pitch_speed.sum = unit::clamp(pitch_speed.dj6 + pitch_speed.vt13 + pitch_speed.client + pitch_speed.mavlink, config.pitch_max);
    yaw_speed.sum = unit::clamp(yaw_speed.dj6 + yaw_speed.vt13 + yaw_speed.client + yaw_speed.mavlink, config.yaw_max);
    gimbal.SetSpeed(yaw_speed.sum, pitch_speed.sum);

    // 设置小陀螺前馈
    gimbal.SetChassisWr(chassis.wr.measure);

    gimbal.OnLoop();
}

void Robot::handle_shooter() {
    shooter.SetBulletSpeed(config.bullet_speed); // 设置弹速
    shooter.SetBulletFreq(config.bullet_freq);   // 设置弹频

    // 摩擦轮
    shooter.SetPrepareShoot(is_prepare_shoot.dj6 || is_prepare_shoot.vt13 || is_prepare_shoot.client);

    // 拨弹电机
    const uint16_t remaining_heat = referee.robot_status.shooter_barrel_heat_limit - referee.power_heat_data.shooter_17mm_1_barrel_heat;
    if (remaining_heat <= config.heat_protect) { // 枪口热量保护
        shooter.SetShoot(false);
    } else {
        shooter.SetShoot(is_shoot.dj6 || is_shoot.vt13 || is_shoot.client);
    }

    shooter.OnLoop();
}

void Robot::handle_referee() {
    // 用于击打反馈
    static uint32_t last_hit_cnt = 0;
    if (last_hit_cnt != referee.hit_cnt) { // 检测到击打，记录受击打方向（imu参考系）
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

void Robot::handle_ui() {
    // yaw
    ui.yaw = gimbal.yaw.ecd.measure;

    // hit
    if (hit.dwt.GetDT() < HIT_TIMEOUT) {
        ui.is_hit = true;
        ui.hit = hit.yaw_imu - gimbal.yaw.imu.measure;
    } else {
        ui.is_hit = false;
    }

    // bullet_speed
    ui.bullet_speed_1 = shooter.bullet_speed.measure1;
    ui.bullet_speed_2 = shooter.bullet_speed.measure2;

    // shoot_current
    ui.shoot_current = shooter.shoot_current;

    // enemy_hp
    if (referee.robot_status.robot_id < 100) { // 红方
        ui.enemy_hp = referee.game_robot_HP.blue_3_robot_HP;
    } else { // 蓝方
        ui.enemy_hp = referee.game_robot_HP.red_3_robot_HP;
    }

    // aim
    ui.is_aim_detected = mavlink.aim.is_detected;
    ui.is_aim_connected = mavlink.is_connected;

    ui.OnLoop();
}