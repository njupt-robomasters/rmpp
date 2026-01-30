#include "Infantry.hpp"

Infantry::Infantry(const config_t& config, const device_t& device) : config(config), device(device) {}

void Infantry::OnLoop() {
    handle_disconnect();

    // 控制器
    handle_fsi6x();
    handle_vt13_rc();
    handle_vt13_client();
    handle_mavlink();

    // 传感器
    handle_imu();

    // 执行器
    handle_chassis();
    handle_gimbal();
    handle_shooter();

    // 裁判系统
    handle_referee();
    handle_ui();
}

void Infantry::setEnable(const bool is_enable) {
    device.chassis.SetEnable(is_enable);
    device.gimbal.SetEnable(is_enable);
    device.shooter.SetEnable(is_enable);
}

void Infantry::handle_disconnect() {
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

void Infantry::handle_fsi6x() {
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

void Infantry::handle_vt13_rc() {
    // 摇杆
    vx.vt13 = device.vt13.x * config.vxy_max;
    vy.vt13 = device.vt13.y * config.vxy_max;
    wr.vt13 = device.vt13.wheel * config.wr_max;
    yaw_speed.vt13 = device.vt13.yaw * config.yaw_max;
    pitch_speed.vt13 = device.vt13.pitch * config.pitch_max;

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
        is_rub.vt13 = true;
    } else {
        is_rub.vt13 = false;
    }

    // 扳机键：射击
    is_shoot.vt13 = device.vt13.trigger;

    device.vt13.OnLoop();
}

void Infantry::handle_vt13_client() {
    // wsad控制底盘
    // ws控制前后
    const UnitFloat dt = dwt_acc.UpdateDT();
    if (device.vt13.key.w) { // 前
        if (vx.client < 0) vx.client = 0 * default_unit;
        vx.client += config.axy * dt;
        vx.client = unit::clamp(vx.client, config.vxy_max);
    } else if (device.vt13.key.s) { // 后
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
    if (device.vt13.key.a) { // 左
        if (vy.client < 0) vy.client = 0 * default_unit;
        vy.client += config.axy * dt;
        vy.client = unit::clamp(vy.client, config.vxy_max);
    } else if (device.vt13.key.d) { // 右
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

    // f刷新ui
    if (device.vt13.key.f) {
        device.ui.Init();
    }

    // 鼠标控制云台
    yaw_speed.client = device.vt13.mouse.yaw * config.yaw_max;
    pitch_speed.client = device.vt13.mouse.pitch * config.pitch_max;

    // 鼠标左键开火
    is_shoot.client = device.vt13.mouse.left;

    // 鼠标右键自瞄
    if (device.vt13.mouse.right && device.mavlink.aim.is_detect) {
        yaw_speed.client = 0 * default_unit;
        pitch_speed.client = 0 * default_unit;
        device.gimbal.SetAngle(device.mavlink.aim.yaw, device.mavlink.aim.pitch);
    }

    // shift开小陀螺
    if (device.vt13.key.shift) {
        wr.client = config.wr_max;
    }
    // ctrl关小陀螺
    if (device.vt13.key.ctrl) {
        wr.client = 0 * default_unit;
    }
    // 失能/断联关小陀螺
    if (device.vt13.is_connect == false || device.vt13.is_protect == true) {
        wr.client = 0 * default_unit;
    }
}

void Infantry::handle_mavlink() {
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

void Infantry::handle_imu() {
    device.imu.OnLoop();
}

void Infantry::handle_chassis() {
    // 设置底盘速度
    vx.sum = unit::clamp(vx.fsi6x + vx.vt13 + vx.client + vx.mavlink, config.vxy_max);
    vy.sum = unit::clamp(vy.fsi6x + vy.vt13 + vy.client + vy.mavlink, config.vxy_max);
    wr.sum = unit::clamp(wr.fsi6x + wr.vt13 + wr.client + wr.mavlink, config.wr_max);
    device.chassis.SetSpeed(vx.sum, vy.sum, wr.sum);

    // 设置云台方向
    device.chassis.SetGimbalYaw(device.gimbal.yaw.ecd.measure);

    // 设置底盘功率限制
    device.chassis.SetPowerLimit(device.referee.chassis.power_limit);

    device.chassis.OnLoop();
}

void Infantry::handle_gimbal() {
    // 设置云台速度
    pitch_speed.sum = unit::clamp(pitch_speed.fsi6x + pitch_speed.vt13 + pitch_speed.client + pitch_speed.mavlink, config.pitch_max);
    yaw_speed.sum = unit::clamp(yaw_speed.fsi6x + yaw_speed.vt13 + yaw_speed.client + yaw_speed.mavlink, config.yaw_max);
    device.gimbal.SetSpeed(yaw_speed.sum, pitch_speed.sum);

    // 设置小陀螺前馈
    device.gimbal.SetChassisWr(device.chassis.wr.measure);

    device.gimbal.OnLoop();
}

void Infantry::handle_shooter() {
    device.shooter.SetBulletSpeed(config.bullet_speed); // 设置弹速
    device.shooter.SetBulletFreq(config.bullet_freq);   // 设置弹频

    // 摩擦轮
    device.shooter.SetRub(is_rub.fsi6x || is_rub.vt13 || is_rub.client);

    // 拨弹电机
    if (device.referee.shooter.heat_remain < config.heat_protect) { // 枪口热量保护
        device.shooter.SetShoot(false);
    } else {
        device.shooter.SetShoot(is_shoot.fsi6x || is_shoot.vt13 || is_shoot.client);
    }

    device.shooter.OnLoop();
}

void Infantry::handle_referee() {
    device.referee.OnLoop(device.imu.yaw, device.gimbal.yaw.ecd.measure);
}

void Infantry::handle_ui() {
    // yaw
    device.ui.yaw = device.gimbal.yaw.ecd.measure;

    // hit
    if (device.referee.hurt.dwt.GetDT() < HIT_TIMEOUT) {
        device.ui.is_hurt = true;
        device.ui.hurt_dir = device.referee.hurt.dir.by_gimbal;
    } else {
        device.ui.is_hurt = false;
    }

    // bullet_speed
    device.ui.bullet_speed_1 = device.shooter.bullet_speed.measure.left;
    device.ui.bullet_speed_2 = device.shooter.bullet_speed.measure.right;

    // shoot_current
    device.ui.shoot_current = device.shooter.GetShootCurrentMeasure();

    // aim
    device.ui.aim.is_detect = device.mavlink.aim.is_detect;
    device.ui.aim.is_connect = device.mavlink.is_connect;

    device.ui.OnLoop();
}