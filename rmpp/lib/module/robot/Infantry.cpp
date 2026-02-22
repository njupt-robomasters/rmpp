#include "Infantry.hpp"

Infantry::Infantry(const config_t& config, const device_t& device) : config(config), device(device) {}

void Infantry::OnLoop() {
    handleConnect();

    // 控制器
    handleRC();
    handleClient();
    handleMavlink();

    // 传感器
    handleIMU();

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
    if (device.referee.game.progress == Referee::COUNTDOWN_5SEC || device.referee.game.progress == Referee::GAMING) {
        device.chassis.SetEnable(true);
        device.gimbal.SetEnable(true);
        device.shooter.SetEnable(true);
        return;
    }

    device.chassis.SetEnable(device.rc.is_enable);
    device.gimbal.SetEnable(device.rc.is_enable);
    device.shooter.SetEnable(device.rc.is_enable);

    // 断联关小陀螺
    if (device.rc.is_enable == false) {
        wr.client = 0 * default_unit;
    }
}

void Infantry::handleRC() {
    device.rc.OnLoop();

    vx.rc = device.rc.x * config.vxy_max;
    vy.rc = device.rc.y * config.vxy_max;
    wr.rc = device.rc.r * config.wr_max;
    yaw_speed.rc = device.rc.yaw * config.yaw_max;
    pitch_speed.rc = device.rc.pitch * config.pitch_max;
    is_rub.rc = device.rc.is_rub;;
    is_shoot.rc = device.rc.is_shoot;

    // 左fn键：底盘分离模式
    if (device.rc.vt13.fn_left) {
        device.chassis.SetMode(Chassis::DETACH_MODE);
    }

    // 右fn键：底盘跟随模式
    if (device.rc.vt13.fn_right) {
        device.chassis.SetMode(Chassis::FOLLOW_MODE);
    }
}

void Infantry::handleClient() {
    // wsad控制底盘
    // ws控制前后
    const UnitFloat dt = dwt_acc.UpdateDT();
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
    // ad控制水平
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

    // 鼠标控制云台
    yaw_speed.client = device.rc.vt13.mouse.yaw * config.yaw_max;
    pitch_speed.client = device.rc.vt13.mouse.pitch * config.pitch_max;

    // 鼠标左键开火
    is_shoot.client = device.rc.vt13.mouse.left;

    // 鼠标右键自瞄
    if (device.rc.vt13.mouse.right && device.mavlink.aim.is_detect) {
        yaw_speed.client = 0 * default_unit;
        pitch_speed.client = 0 * default_unit;
        device.gimbal.SetAngle(device.mavlink.aim.yaw, device.mavlink.aim.pitch);
    }

    // shift开小陀螺
    if (device.rc.vt13.key.shift) {
        wr.client = config.wr_max;
    }
    // ctrl关小陀螺
    if (device.rc.vt13.key.ctrl) {
        wr.client = 0 * default_unit;
    }

    // f刷新ui
    if (device.rc.vt13.key.f) {
        device.ui.Init();
    }
}

void Infantry::handleMavlink() {
    device.mavlink.imu = {
        .yaw = device.imu.yaw,
        .pitch = device.imu.pitch,
        .roll = device.imu.roll
    };
    device.mavlink.referee = {
        .is_red = device.referee.game.is_red,
        .bullet_speed = device.referee.shooter.bullet_speed
    };

    device.mavlink.OnLoop();
}

void Infantry::handleIMU() {
    device.imu.OnLoop();
}

void Infantry::handleChassis() {
    // 设置底盘速度
    vx.sum = unit::clamp(vx.rc + vx.client + vx.mavlink, config.vxy_max);
    vy.sum = unit::clamp(vy.rc + vy.client + vy.mavlink, config.vxy_max);
    wr.sum = unit::clamp(wr.rc + wr.client + wr.mavlink, config.wr_max);
    device.chassis.SetSpeed(vx.sum, vy.sum, wr.sum);

    // 设置前进正方向
    device.chassis.SetGimbalYaw(device.gimbal.yaw.ecd.measure);

    // 设置功率限制
    device.chassis.SetPowerLimit(device.referee.chassis.power_limit);

    // 设置缓冲能量
    device.chassis.SetBufferEnergy(device.referee.chassis.buffer_energy);

    device.chassis.OnLoop();
}

void Infantry::handleGimbal() {
    // 设置云台速度
    pitch_speed.sum = unit::clamp(pitch_speed.rc + pitch_speed.client + pitch_speed.mavlink, config.pitch_max);
    yaw_speed.sum = unit::clamp(yaw_speed.rc + yaw_speed.client + yaw_speed.mavlink, config.yaw_max);
    device.gimbal.SetSpeed(yaw_speed.sum, pitch_speed.sum);

    // 设置小陀螺前馈
    device.gimbal.SetChassisWr(device.chassis.wr.measure);

    device.gimbal.OnLoop();
}

void Infantry::handleShooter() {
    device.shooter.SetBulletSpeed(config.bullet_speed); // 设置弹速
    device.shooter.SetBulletFreq(config.bullet_freq);   // 设置弹频

    // 摩擦轮
    device.shooter.SetRub(is_rub.fsi6x || is_rub.rc || is_rub.client);

    // 拨弹电机
    if (device.referee.shooter.heat_remain < config.heat_protect) { // 枪口热量保护
        device.shooter.SetShoot(false);
    } else {
        device.shooter.SetShoot(is_shoot.fsi6x || is_shoot.rc || is_shoot.client);
    }

    device.shooter.OnLoop();
}

void Infantry::handleReferee() {
    device.referee.SetYaw(device.imu.yaw, device.gimbal.yaw.ecd.measure);
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
    device.ui.is_detect = device.mavlink.aim.is_detect;

    // 电机连接状态
    device.chassis.HandleUI(device.ui);
    device.gimbal.HandleUI(device.ui);
    device.shooter.HandleUI(device.ui);

    device.ui.OnLoop();
}