#include "Infantry.hpp"

Infantry::Infantry(const config_t& config, const device_t& device) : config(config), device(device) {}

void Infantry::OnLoop() {
    handleConnect();

    // 控制器
    handleFSi6X();
    handleVT13();
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

void Infantry::setEnable(const bool is_enable) {
    device.chassis.SetEnable(is_enable);
    device.gimbal.SetEnable(is_enable);
    device.shooter.SetEnable(is_enable);
}

void Infantry::handleConnect() {
    if (device.vt13.is_connect) { // VT13遥控器优先
        if (device.vt13.is_protect) {
            setEnable(false);
        } else {
            setEnable(true);
        }
    } else if (device.fsi6x.is_connect) {
        if (device.fsi6x.is_protect) {
            setEnable(false);
        } else {
            setEnable(true);
        }
    } else {
        setEnable(false);
    }
}

void Infantry::handleFSi6X() {
    vx.fsi6x = device.fsi6x.x * config.vxy_max;
    vy.fsi6x = device.fsi6x.y * config.vxy_max;
    yaw_speed.fsi6x = device.fsi6x.yaw * config.yaw_max;
    pitch_speed.fsi6x = device.fsi6x.pitch * config.pitch_max;

    // 小陀螺
    if (device.fsi6x.swb == FSi6X::DOWN) {
        wr.fsi6x = device.fsi6x.vra * config.wr_max;
    } else {
        wr.fsi6x = 0 * default_unit;
    }

    // 发射机构
    switch (device.fsi6x.swc) {
        case FSi6X::ERR:
            is_rub.fsi6x = false;
            is_shoot.fsi6x = false;
        case FSi6X::UP:
            is_rub.fsi6x = false;
            is_shoot.fsi6x = false;
            break;
        case FSi6X::MID:
            is_rub.fsi6x = true;
            is_shoot.fsi6x = false;
            break;
        case FSi6X::DOWN:
            is_rub.fsi6x = true;
            is_shoot.fsi6x = true;
            break;
    }

    device.fsi6x.OnLoop();
}

void Infantry::handleVT13() {
    auto handleRC = [this]() {
        // 摇杆
        vx.vt13_rc = device.vt13.x * config.vxy_max;
        vy.vt13_rc = device.vt13.y * config.vxy_max;
        wr.vt13_rc = device.vt13.wheel * config.wr_max;
        yaw_speed.vt13_rc = device.vt13.yaw * config.yaw_max;
        pitch_speed.vt13_rc = device.vt13.pitch * config.pitch_max;

        // 左fn键：底盘分离模式
        if (device.vt13.fn_left) {
            device.chassis.SetMode(Chassis::DETACH_MODE);
        }

        // 右fn键：底盘跟随模式
        if (device.vt13.fn_right) {
            device.chassis.SetMode(Chassis::FOLLOW_MODE);
        }

        // S挡：开摩擦轮
        if (device.vt13.mode == VT13::S) {
            is_rub.vt13_rc = true;
        } else {
            is_rub.vt13_rc = false;
        }
    };

    auto handleClient = [this]() {
        // 扳机键：射击
        is_shoot.vt13_rc = device.vt13.trigger;

        // wsad控制底盘
        // ws控制前后
        const UnitFloat dt = dwt_acc.UpdateDT();
        if (device.vt13.key.w) { // 前
            if (vx.vt13_client < 0) vx.vt13_client = 0 * default_unit;
            vx.vt13_client += config.axy * dt;
            vx.vt13_client = unit::clamp(vx.vt13_client, config.vxy_max);
        } else if (device.vt13.key.s) { // 后
            if (vx.vt13_client > 0) vx.vt13_client = 0 * default_unit;
            vx.vt13_client -= config.axy * dt;
            vx.vt13_client = unit::clamp(vx.vt13_client, config.vxy_max);
        } else { // 松开减速
            if (vx.vt13_client > 0) {
                vx.vt13_client -= config.dxy * dt;
                if (vx.vt13_client < 0) vx.vt13_client = 0 * default_unit;
            }
            if (vx.vt13_client < 0) {
                vx.vt13_client += config.dxy * dt;
                if (vx.vt13_client > 0) vx.vt13_client = 0 * default_unit;
            }
        }
        // ad控制水平
        if (device.vt13.key.a) { // 左
            if (vy.vt13_client < 0) vy.vt13_client = 0 * default_unit;
            vy.vt13_client += config.axy * dt;
            vy.vt13_client = unit::clamp(vy.vt13_client, config.vxy_max);
        } else if (device.vt13.key.d) { // 右
            if (vy.vt13_client > 0) vy.vt13_client = 0 * default_unit;
            vy.vt13_client -= config.axy * dt;
            vy.vt13_client = unit::clamp(vy.vt13_client, config.vxy_max);
        } else { // 松开减速
            if (vy.vt13_client > 0) {
                vy.vt13_client -= config.dxy * dt;
                if (vy.vt13_client < 0) vy.vt13_client = 0 * default_unit;
            }
            if (vy.vt13_client < 0) {
                vy.vt13_client += config.dxy * dt;
                if (vy.vt13_client > 0) vy.vt13_client = 0 * default_unit;
            }
        }

        // f刷新ui
        if (device.vt13.key.f) {
            device.ui.Init();
        }

        // 鼠标控制云台
        yaw_speed.vt13_client = device.vt13.mouse.yaw * config.yaw_max;
        pitch_speed.vt13_client = device.vt13.mouse.pitch * config.pitch_max;

        // 鼠标左键开火
        is_shoot.vt13_client = device.vt13.mouse.left;

        // 鼠标右键自瞄
        if (device.vt13.mouse.right && device.mavlink.aim.is_detect) {
            yaw_speed.vt13_client = 0 * default_unit;
            pitch_speed.vt13_client = 0 * default_unit;
            device.gimbal.SetAngle(device.mavlink.aim.yaw, device.mavlink.aim.pitch);
        }

        // shift开小陀螺
        if (device.vt13.key.shift) {
            wr.vt13_client = config.wr_max;
        }
        // ctrl关小陀螺
        if (device.vt13.key.ctrl) {
            wr.vt13_client = 0 * default_unit;
        }
        // 失能/断联关小陀螺
        if (device.vt13.is_connect == false || device.vt13.is_protect == true) {
            wr.vt13_client = 0 * default_unit;
        }
    };

    handleRC();
    handleClient();

    device.vt13.OnLoop();
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
    vx.sum = unit::clamp(vx.fsi6x + vx.vt13_rc + vx.vt13_client + vx.mavlink, config.vxy_max);
    vy.sum = unit::clamp(vy.fsi6x + vy.vt13_rc + vy.vt13_client + vy.mavlink, config.vxy_max);
    wr.sum = unit::clamp(wr.fsi6x + wr.vt13_rc + wr.vt13_client + wr.mavlink, config.wr_max);
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
    pitch_speed.sum = unit::clamp(pitch_speed.fsi6x + pitch_speed.vt13_rc + pitch_speed.vt13_client + pitch_speed.mavlink, config.pitch_max);
    yaw_speed.sum = unit::clamp(yaw_speed.fsi6x + yaw_speed.vt13_rc + yaw_speed.vt13_client + yaw_speed.mavlink, config.yaw_max);
    device.gimbal.SetSpeed(yaw_speed.sum, pitch_speed.sum);

    // 设置小陀螺前馈
    device.gimbal.SetChassisWr(device.chassis.wr.measure);

    device.gimbal.OnLoop();
}

void Infantry::handleShooter() {
    device.shooter.SetBulletSpeed(config.bullet_speed); // 设置弹速
    device.shooter.SetBulletFreq(config.bullet_freq);   // 设置弹频

    // 摩擦轮
    device.shooter.SetRub(is_rub.fsi6x || is_rub.vt13_rc || is_rub.vt13_client);

    // 拨弹电机
    if (device.referee.shooter.heat_remain < config.heat_protect) { // 枪口热量保护
        device.shooter.SetShoot(false);
    } else {
        device.shooter.SetShoot(is_shoot.fsi6x || is_shoot.vt13_rc || is_shoot.vt13_client);
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