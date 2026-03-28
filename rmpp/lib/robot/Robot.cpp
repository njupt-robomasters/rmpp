#include "Robot.hpp"

Robot::Robot(const config_t& config, const device_t& device) : config(config), device(device) {}

void Robot::Init() {
    device.gimbal.LoadYawOffset(device.flashdb);
    device.buzzer.Play(Buzzer::C5D5G5);
}

void Robot::OnLoop() {
    device.led.OnLoop();
    device.buzzer.OnLoop();

    // 1V1特化参数
    if (device.referee.game.type == Referee::RMUL_1V1) {
        config.bullet_freq = 10 * Hz;
        config.heat_protect = 50;
    }

    // 断联检测
    bool ignore_disconnect = false;
    ignore_disconnect |= device.referee.game.game_progress == Referee::REFEREE_SELF_CHECK;
    ignore_disconnect |= device.referee.game.game_progress == Referee::COUNTDOWN_5SEC;
    ignore_disconnect |= device.referee.game.game_progress == Referee::GAMING;
    if (ignore_disconnect) { // 忽略断联检测
        device.chassis.SetEnable(true);
        device.gimbal.SetEnable(true);
        device.shooter.SetEnable(true);
    } else {
        device.chassis.SetEnable(device.rc.is_enable);
        device.gimbal.SetEnable(device.rc.is_enable);
        device.shooter.SetEnable(device.rc.is_enable);
    }

    // 控制器
    handleRC();
    handleClient();
    handleMavlink();

    // 执行器
    handleChassis();
    handleGimbal();
    handleShooter();

    // 裁判系统
    handleReferee();
    handleUI();
}

void Robot::handleRC() {
    device.rc.OnLoop();

    //【底盘】
    vx.rc = device.rc.x * config.vxy_max;
    vy.rc = device.rc.y * config.vxy_max;
    wr.rc = device.rc.r * config.wr_max;
    // 底盘跟随
    if (device.rc.is_fn) {
        device.chassis.SetMode(Chassis::FOLLOW_MODE);
    }

    //【云台】
    wyaw.rc = device.rc.yaw * config.wyaw;
    wpitch.rc = device.rc.pitch * config.wpitch;
    // 自瞄
    if (device.rc.is_auto_aim && device.mavlink.vision.is_detect) { // 使能自瞄且识别到
        gimbal_mode.rc = GIMBAL_ANGLE_MODE;                         // 云台角度控制
        yaw.rc = device.mavlink.vision.yaw;
        pitch.rc = device.mavlink.vision.pitch;
    } else {
        gimbal_mode.rc = GIMBAL_SPEED_MODE; // 云台速度控制
    }
    // 校准yaw偏移
    static bool is_calib_last = false;
    if (!is_calib_last && device.rc.is_calib) { // 上升沿
        device.gimbal.SetYawZero();
        device.gimbal.SaveYawOffset(device.flashdb);
        device.buzzer.Play(Buzzer::G5G5G5);
    }
    is_calib_last = device.rc.is_calib;

    //【发射机构】
    // 摩擦轮（S挡开启）
    is_rub.rc = device.rc.is_rub;
    // 拨弹电机
    if (device.rc.is_auto_aim) { // 自瞄状态下，扳机键批准自动火控
        is_shoot.rc = device.rc.is_shoot && device.mavlink.vision.is_fire;
    } else { // 非自瞄状态下，扳机键开火
        is_shoot.rc = device.rc.is_shoot;
    }
}

void Robot::handleClient() {
    // 维护dt
    static BSP::Dwt dwt;
    const UnitFloat dt = dwt.UpdateDT();

    //【底盘】
    // 前后（w、s键），限制加速度
    if (device.rc.key.w) { // 前
        if (vx.client < 0) vx.client = 0 * default_unit;
        vx.client += config.axy * dt;
        vx.client = unit::clamp(vx.client, config.vxy_max);
    } else if (device.rc.key.s) { // 后
        if (vx.client > 0) vx.client = 0 * default_unit;
        vx.client -= config.axy * dt;
        vx.client = unit::clamp(vx.client, config.vxy_max);
    } else { // 松开减速
        if (vx.client > 0) {
            vx.client -= config.dxy * dt;
            if (vx.client < 0) vx.client = 0 * default_unit;
        }
        if (vx.client < 0) {
            vx.client += config.dxy * dt;
            if (vx.client > 0) vx.client = 0 * default_unit;
        }
    }
    // 水平（a、d键），限制加速度
    if (device.rc.key.a) { // 左
        if (vy.client < 0) vy.client = 0 * default_unit;
        vy.client += config.axy * dt;
        vy.client = unit::clamp(vy.client, config.vxy_max);
    } else if (device.rc.key.d) { // 右
        if (vy.client > 0) vy.client = 0 * default_unit;
        vy.client -= config.axy * dt;
        vy.client = unit::clamp(vy.client, config.vxy_max);
    } else { // 松开减速
        if (vy.client > 0) {
            vy.client -= config.dxy * dt;
            if (vy.client < 0) vy.client = 0 * default_unit;
        }
        if (vy.client < 0) {
            vy.client += config.dxy * dt;
            if (vy.client > 0) vy.client = 0 * default_unit;
        }
    }
    // 旋转（shift、ctrl键），限制角加速度
    if (device.rc.key.shift) { // 逆时针旋转
        if (wr.client < 0) wr.client = 0 * default_unit;
        wr.client += config.ar * dt;
        wr.client = unit::clamp(wr.client, config.wr_max);
    } else if (device.rc.key.ctrl) { // 顺逆时针旋转
        if (wr.client > 0) wr.client = 0 * default_unit;
        wr.client -= config.ar * dt;
        wr.client = unit::clamp(wr.client, config.wr_max);
    } else { // 松开减速
        if (wr.client > 0) {
            wr.client -= config.dr * dt;
            if (wr.client < 0) wr.client = 0 * default_unit;
        }
        if (wr.client < 0) {
            wr.client += config.dr * dt;
            if (wr.client > 0) wr.client = 0 * default_unit;
        }
    }

    //【云台】
    // yaw（鼠标上下），限制角加速度
    UnitFloat<deg_s> dwyaw = device.rc.mouse.yaw * config.wyaw - wyaw.client;
    const UnitFloat<deg_s> dwyaw_max = config.ayaw * dt;
    if (dwyaw > dwyaw_max) dwyaw = dwyaw_max;
    if (dwyaw < -dwyaw_max) dwyaw = -dwyaw_max;
    wyaw.client += dwyaw;
    // pitch（鼠标左右），限制角加速度
    UnitFloat<deg_s> dwpitch = device.rc.mouse.pitch * config.wpitch - wpitch.client;
    const UnitFloat<deg_s> dwpitch_max = config.apitch * dt;
    if (dwpitch > dwpitch_max) dwpitch = dwpitch_max;
    if (dwpitch < -dwpitch_max) dwpitch = -dwpitch_max;
    wpitch.client += dwpitch;
    // 自瞄（鼠标右键）
    if (device.rc.mouse.right && device.mavlink.vision.is_detect) { // 使能自瞄且识别到
        gimbal_mode.client = GIMBAL_ANGLE_MODE;                     // 云台角度控制
        yaw.client = device.mavlink.vision.yaw;
        pitch.client = device.mavlink.vision.pitch;
    } else {
        gimbal_mode.client = GIMBAL_SPEED_MODE; // 云台速度控制
    }
    // 一键掉头（r键）
    static bool last_r = false;
    if (!last_r && device.rc.key.r) { // 上升沿
        device.gimbal.yaw.ecd.ref = device.gimbal.yaw.ecd.ref + 180 * deg;
        device.gimbal.yaw.imu.ref = device.gimbal.yaw.imu.ref + 180 * deg;
    }
    last_r = device.rc.key.r;

    //【发射机构】
    // 拨弹电机
    static bool force_fire = false;
    static bool last_left = false;
    static BSP::Dwt dwt_last_click;
    if (device.rc.mouse.right) { // 允许自瞄状态下
        // 双击左键强制开火
        if (!last_left && device.rc.mouse.left) {
            if (dwt_last_click.GetDT() < 250 * ms) {
                force_fire = true;
            }
            dwt_last_click.UpdateDT();
        }
        last_left = device.rc.mouse.left;

        if (device.rc.mouse.left) { // 按下左键
            if (force_fire) {       // 强制开火
                is_shoot.client = true;
            } else { // 自动火控
                is_shoot.client = device.mavlink.vision.is_fire;
            }
        } else { // 释放左键
            is_shoot.client = false;
            force_fire = false; // 复位强制开火
        }
    } else { // 非自瞄状态下，鼠标左键开火
        is_shoot.client = device.rc.mouse.left;
        force_fire = false; // 复位强制开火
    }

    // 刷新ui（f键）
    if (device.rc.key.f) {
        device.ui.Init();
    }
}

void Robot::handleMavlink() {
    device.mavlink.imu = {
        .yaw = device.gimbal.imu.yaw,
        .pitch = device.gimbal.imu.pitch,
        .roll = device.gimbal.imu.roll,
    };
    device.mavlink.referee = {
        .is_red = device.referee.game.is_red,
        .game_progress = device.referee.game.game_progress,
        .stage_remain_time = device.referee.game.stage_remain_time,
        .bullet_speed = device.referee.shooter.bullet_speed,
    };

    device.mavlink.OnLoop();
}

void Robot::handleChassis() {
    // 设置底盘速度
    const UnitFloat<> vx = unit::clamp(this->vx.rc + this->vx.client + this->vx.software, config.vxy_max);
    const UnitFloat<> vy = unit::clamp(this->vy.rc + this->vy.client + this->vy.software, config.vxy_max);
    const UnitFloat<> wr = unit::clamp(this->wr.rc + this->wr.client + this->wr.software, config.wr_max);
    device.chassis.SetSpeed(vx, vy, wr);

    // 设置前进正方向
    device.chassis.SetGimbalYaw(device.gimbal.yaw.ecd.measure);

    // 设置功率限制
    device.chassis.SetPowerLimit(device.referee.chassis.power_limit);

    // 设置缓冲能量
    device.chassis.SetBufferEnergy(device.referee.chassis.buffer_energy);

    device.chassis.OnLoop();
}

void Robot::handleGimbal() {
    // 设置角度或速度
    if (gimbal_mode.rc == GIMBAL_ANGLE_MODE) {
        device.gimbal.SetAngle(yaw.rc, pitch.rc);
        device.gimbal.SetSpeed(0 * default_unit, 0 * default_unit);
    } else if (gimbal_mode.client == GIMBAL_ANGLE_MODE) {
        device.gimbal.SetAngle(yaw.client, pitch.client);
        device.gimbal.SetSpeed(0 * default_unit, 0 * default_unit);
    } else if (gimbal_mode.software == GIMBAL_ANGLE_MODE) {
        device.gimbal.SetAngle(yaw.software, pitch.software);
        device.gimbal.SetSpeed(0 * default_unit, 0 * default_unit);
    } else {
        const UnitFloat<> yaw_speed = unit::clamp(this->wyaw.rc + this->wyaw.client + this->wyaw.software, config.wyaw);
        const UnitFloat<> pitch_speed = unit::clamp(this->wpitch.rc + this->wpitch.client + this->wpitch.software, config.wpitch);
        device.gimbal.SetSpeed(yaw_speed, pitch_speed);
    }

    // 设置小陀螺前馈
    device.gimbal.SetChassisWr(device.chassis.wr.measure);

    device.gimbal.OnLoop();
}

void Robot::handleShooter() {
    // 摩擦轮
    device.shooter.SetRub(is_rub.rc || is_rub.client || is_rub.software);

    // 拨弹电机
    if (is_shoot.rc) { // 遥控器射击，pitch摇杆控制弹频
        device.shooter.SetShoot(true);
        device.shooter.SetBulletFreq(config.bullet_freq * device.rc.shoot); // 设置弹频
    } else if (is_shoot.client || is_shoot.software) {
        device.shooter.SetShoot(true);
        device.shooter.SetBulletFreq(config.bullet_freq); // 设置弹频
    } else {
        device.shooter.SetShoot(false);
    }

    // 设置弹速
    device.shooter.SetBulletSpeed(config.bullet_speed);

    // 枪口热量保护
    device.shooter.SetHeatProtect(device.referee.shooter.heat_remain < config.heat_protect);

    device.shooter.OnLoop();
}

void Robot::handleReferee() {
    device.referee.SetYaw(device.gimbal.yaw.ecd.measure, device.gimbal.yaw.imu.measure);
    device.referee.OnLoop();
}

void Robot::handleUI() {
    // 云台角度
    device.ui.yaw_ecd = device.gimbal.yaw.ecd.measure; // 云台相对底盘角度
    device.ui.yaw = device.gimbal.yaw.imu.measure;
    device.ui.pitch = device.gimbal.pitch.imu.measure;

    // 伤害方向
    device.ui.is_hurt = device.referee.hurt.is_hurt;
    device.ui.hurt_dir = device.referee.hurt.dir;

    // 自瞄识别
    device.ui.is_detect = device.mavlink.vision.is_detect;

    // 提示条
    device.ui.cap_ratio = 0 * ratio;                        // 超级电容剩余能量比例
    device.ui.chassis_wr = device.chassis.wr.measure;       // 底盘旋转速度
    device.ui.shoot_freq = config.bullet_freq;              // 弹频
    device.ui.shoot_current = device.shooter.shoot_current; // 拨弹电机电流

    // 机器人状态
    device.chassis.UpdateUI(device.ui);                                            // 底盘
    device.gimbal.UpdateUI(device.ui);                                             // 云台
    device.shooter.UpdateUI(device.ui);                                            // 发射机构
    device.ui.robot.referee = device.referee.is_connect ? UI::GREEN : UI::PINK;    // 裁判系统
    device.ui.robot.aim = device.mavlink.is_connect_vision ? UI::GREEN : UI::PINK; // 自瞄

    device.ui.OnLoop();
}