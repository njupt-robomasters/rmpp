#include "Infantry.hpp"

Infantry::Infantry(const config_t& config, const device_t& device) : config(config), device(device) {}

void Infantry::Init() {
    device.gimbal.LoadYawOffset(device.flashdb);
    device.buzzer.Play(Buzzer::C5D5G5);
}

void Infantry::OnLoop() {
    handleConnect();

    device.led.OnLoop();
    device.buzzer.OnLoop();

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

void Infantry::handleConnect() {
    // 比赛时候忽略断联保护
    if (device.referee.game.game_progress == Referee::GAMING) {
        device.chassis.SetEnable(true);
        device.gimbal.SetEnable(true);
        device.shooter.SetEnable(true);
    } else {
        device.chassis.SetEnable(device.rc.is_enable);
        device.gimbal.SetEnable(device.rc.is_enable);
        device.shooter.SetEnable(device.rc.is_enable);
    }
}

void Infantry::handleRC() {
    device.rc.OnLoop();

    vx.rc = device.rc.x * config.vxy_max;
    vy.rc = device.rc.y * config.vxy_max;
    wr.rc = device.rc.r * config.wr_max;
    wyaw.rc = device.rc.yaw * config.wyaw;
    if (device.rc.is_shoot) {
        wpitch.rc = 0 * default_unit;
    } else {
        wpitch.rc = device.rc.pitch * config.wpitch;
    }
    is_rub.rc = device.rc.is_rub;

    if (device.rc.is_shoot) { // 遥控器手动开火
        is_shoot.rc = true;
    } else if (device.rc.vt13.fn_right && device.mavlink.auto_aim.is_fire) { // 遥控器自瞄自动火控
        is_shoot.rc = true;
    } else {
        is_shoot.rc = false;
    }

    // 左fn底盘跟随
    if (device.rc.vt13.fn_left) {
        device.chassis.SetMode(Chassis::FOLLOW_MODE);
    }

    // 右fn键自瞄
    if (device.rc.vt13.fn_right && device.mavlink.auto_aim.is_detect) {
        gimbal_mode.rc = GIMBAL_ANGLE_MODE;
        yaw.rc = device.mavlink.auto_aim.yaw;
        pitch.rc = device.mavlink.auto_aim.pitch;
    } else {
        gimbal_mode.rc = GIMBAL_SPEED_MODE;
    }

    // 校准yaw偏移
    static bool set_yaw_zero_last = false;
    const bool set_yaw_zero = device.rc.vt13.mode == VT13::C && device.rc.vt13.fn_left && device.rc.vt13.fn_right;
    if (set_yaw_zero && !set_yaw_zero_last) {
        device.gimbal.SetYawZero();
        device.gimbal.SaveYawOffset(device.flashdb);
        device.buzzer.Play(Buzzer::G5G5G5);
    }
    set_yaw_zero_last = set_yaw_zero;
}

void Infantry::handleClient() {
    // 维护dt
    static BSP::Dwt dwt;
    const UnitFloat dt = dwt.UpdateDT();

    // ws控制前后，限制加速度
    if (device.rc.vt13.key.w) { // 前
        if (vx.client < 0) vx.client = 0 * default_unit;
        vx.client += config.axy * dt;
        vx.client = unit::clamp(vx.client, config.vxy_max);
    } else if (device.rc.vt13.key.s) { // 后
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
    // ad控制水平，限制加速度
    if (device.rc.vt13.key.a) { // 左
        if (vy.client < 0) vy.client = 0 * default_unit;
        vy.client += config.axy * dt;
        vy.client = unit::clamp(vy.client, config.vxy_max);
    } else if (device.rc.vt13.key.d) { // 右
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
    // shift ctrl控制旋转
    if (device.rc.vt13.key.shift) { // 逆时针旋转
        if (wr.client < 0) wr.client = 0 * default_unit;
        wr.client += config.ar * dt;
        wr.client = unit::clamp(wr.client, config.wr_max);
    } else if (device.rc.vt13.key.ctrl) { // 顺逆时针旋转
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

    // 鼠标控制云台yaw，限制角加速度
    UnitFloat<deg_s> dwyaw = device.rc.vt13.mouse.yaw * config.wyaw - wyaw.client;
    const UnitFloat<deg_s> dwyaw_max = config.ayaw * dt;
    if (dwyaw > dwyaw_max) dwyaw = dwyaw_max;
    if (dwyaw < -dwyaw_max) dwyaw = -dwyaw_max;
    wyaw.client += dwyaw;
    // 鼠标控制云台pitch，限制角加速度
    UnitFloat<deg_s> dwpitch = device.rc.vt13.mouse.pitch * config.wpitch - wpitch.client;
    const UnitFloat<deg_s> dwpitch_max = config.apitch * dt;
    if (dwpitch > dwpitch_max) dwpitch = dwpitch_max;
    if (dwpitch < -dwpitch_max) dwpitch = -dwpitch_max;
    wpitch.client += dwpitch;

    // 鼠标左键开火
    is_shoot.client = device.rc.vt13.mouse.left;

    // 鼠标右键自瞄
    if (device.rc.vt13.mouse.right && device.mavlink.auto_aim.is_detect) {
        gimbal_mode.client = GIMBAL_ANGLE_MODE;
        yaw.client = device.mavlink.auto_aim.yaw;
        pitch.client = device.mavlink.auto_aim.pitch;
    } else {
        gimbal_mode.client = GIMBAL_SPEED_MODE;
    }

    // f刷新ui
    if (device.rc.vt13.key.f) {
        device.ui.Init();
    }

    // r一键掉头
    static bool last_r = false;
    if (!last_r && device.rc.vt13.key.r) { // 上升沿
        device.gimbal.yaw.ecd.ref = device.gimbal.yaw.ecd.ref + 180 * deg;
        device.gimbal.yaw.imu.ref = device.gimbal.yaw.imu.ref + 180 * deg;
    }
    last_r = device.rc.vt13.key.r;
}

void Infantry::handleMavlink() {
    device.mavlink.imu = {
        .yaw = device.gimbal.imu.yaw,
        .pitch = device.gimbal.imu.pitch,
        .roll = device.gimbal.imu.roll
    };
    device.mavlink.referee = {
        .is_red = device.referee.game.is_red,
        .bullet_speed = device.referee.shooter.bullet_speed
    };

    device.mavlink.OnLoop();
}

void Infantry::handleChassis() {
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

void Infantry::handleGimbal() {
    if (gimbal_mode.rc == GIMBAL_ANGLE_MODE) {
        device.gimbal.SetAngle(yaw.rc, pitch.rc);
    } else if (gimbal_mode.client == GIMBAL_ANGLE_MODE) {
        device.gimbal.SetAngle(yaw.client, pitch.client);
    } else if (gimbal_mode.software == GIMBAL_ANGLE_MODE) {
        device.gimbal.SetAngle(yaw.software, pitch.software);
    } else {
        const UnitFloat<> yaw_speed = unit::clamp(this->wyaw.rc + this->wyaw.client + this->wyaw.software, config.wyaw);
        const UnitFloat<> pitch_speed = unit::clamp(this->wpitch.rc + this->wpitch.client + this->wpitch.software, config.wpitch);
        device.gimbal.SetSpeed(yaw_speed, pitch_speed);
    }

    // 设置小陀螺前馈
    device.gimbal.SetChassisWr(device.chassis.wr.measure);

    device.gimbal.OnLoop();
}

void Infantry::handleShooter() {
    device.shooter.SetBulletSpeed(config.bullet_speed); // 设置弹速

    // 摩擦轮
    device.shooter.SetRub(is_rub.rc || is_rub.client || is_rub.software);

    // 拨弹电机
    if (is_shoot.rc) {                                                                      // 遥控器射击，pitch摇杆控制弹频
        device.shooter.SetShoot(device.referee.shooter.heat_remain >= config.heat_protect); // 枪口热量保护
        device.shooter.SetBulletFreq(config.bullet_freq * device.rc.pitch);                 // 设置弹频
    } else if (is_shoot.client || is_shoot.software) {
        device.shooter.SetShoot(device.referee.shooter.heat_remain >= config.heat_protect); // 枪口热量保护
        device.shooter.SetBulletFreq(config.bullet_freq);                                   // 设置弹频
    } else {
        device.shooter.SetShoot(false);
    }

    device.shooter.OnLoop();
}

void Infantry::handleReferee() {
    device.referee.SetYaw(device.gimbal.yaw.ecd.measure, device.gimbal.yaw.imu.measure);
    device.referee.OnLoop();
}

void Infantry::handleUI() {
    // 云台角度
    device.ui.yaw_ecd = device.gimbal.yaw.ecd.measure; // 云台相对底盘角度
    device.ui.yaw = device.gimbal.yaw.imu.measure;
    device.ui.pitch = device.gimbal.pitch.imu.measure;

    // 伤害方向
    device.ui.is_hurt = device.referee.is_hurt;
    device.ui.hurt_dir = device.referee.hurt_dir;

    // 超级电容
    device.ui.robot.cap = UI::PINK;
    device.ui.cap_ratio = 0 * ratio;

    // 底盘旋转速度
    device.ui.chassis_wr = device.chassis.wr.measure;

    // 弹频
    device.ui.shoot_freq = config.bullet_freq;

    // 拨弹电机电流
    device.ui.shoot_current = device.shooter.shoot_current;

    // 自瞄
    device.ui.robot.aim = device.mavlink.is_connect ? UI::GREEN : UI::PINK;
    device.ui.is_detect = device.mavlink.auto_aim.is_detect;

    // 电机连接状态
    device.chassis.UpdateUI(device.ui);
    device.gimbal.UpdateUI(device.ui);
    device.shooter.UpdateUI(device.ui);

    device.ui.OnLoop();
}