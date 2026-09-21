#include "robot.hpp"
#include "math/conversions.hpp"
#include "math/math.hpp"

using math::deg;
using math::ms;

Robot::Robot(const Config& config, const Device& device) : config(config), device(device) {}

void Robot::init() {
    device.gimbal.load_yaw_offset(device.flashdb);
    device.buzzer.play(Buzzer::C5D5G5);
}

void Robot::on_loop() {
    device.led.on_loop();
    device.buzzer.on_loop();

    // 断联检测
    bool ignore_disconnect = false;
    ignore_disconnect |= device.referee.game.game_progress == Referee::REFEREE_SELF_CHECK;
    ignore_disconnect |= device.referee.game.game_progress == Referee::COUNTDOWN_5SEC;
    ignore_disconnect |= device.referee.game.game_progress == Referee::GAMING;
    if (ignore_disconnect) { // 忽略断联检测
        device.chassis.set_enable(true);
        device.gimbal.set_enable(true);
        device.shooter.set_enable(true);
    } else {
        device.chassis.set_enable(device.rc.is_enable);
        device.gimbal.set_enable(device.rc.is_enable);
        device.shooter.set_enable(device.rc.is_enable);
    }

    // 控制器
    handle_rc();
    handle_client();
    handle_mavlink();

    // 执行器
    handle_chassis();
    handle_gimbal();
    handle_shooter();

    // 裁判系统
    handle_referee();
    handle_ui();
}

void Robot::handle_rc() {
    device.rc.on_loop();

    //【底盘】
    vx_.rc = device.rc.x * config.vxy_max;
    vy_.rc = device.rc.y * config.vxy_max;
    wr_.rc = device.rc.r * config.wr_max;
    // 底盘跟随
    if (device.rc.is_fn) {
        device.chassis.set_mode(Chassis::FOLLOW_MODE);
    }

    //【云台】
    wyaw_.rc = device.rc.yaw * config.wyaw;
    wpitch_.rc = device.rc.pitch * config.wpitch;
    // 自瞄
    if (device.rc.is_auto_aim && device.mavlink.vision.is_detect) { // 使能自瞄且识别到
        gimbal_mode_.rc = GIMBAL_ANGLE_MODE;                        // 云台角度控制
        yaw_.rc = device.mavlink.vision.yaw;
        pitch_.rc = device.mavlink.vision.pitch;
    } else {
        gimbal_mode_.rc = GIMBAL_SPEED_MODE; // 云台速度控制
    }
    // 校准yaw偏移
    static bool is_calib_last = false;
    if (!is_calib_last && device.rc.is_calib) { // 上升沿
        device.gimbal.set_yaw_zero();
        device.gimbal.save_yaw_offset(device.flashdb);
        device.buzzer.play(Buzzer::G5G5G5);
    }
    is_calib_last = device.rc.is_calib;

    //【发射机构】
    // 摩擦轮（S挡开启）
    is_rub_.rc = device.rc.is_rub;
    // 拨弹电机
    if (device.rc.is_auto_aim) { // 自瞄状态下，扳机键批准自动火控
        is_shoot_.rc = device.rc.is_shoot && device.mavlink.vision.is_fire;
    } else { // 非自瞄状态下，扳机键开火
        is_shoot_.rc = device.rc.is_shoot;
    }
}

void Robot::handle_client() {
    // 维护dt
    static rmpp::Timer timer;
    const float dt = timer.update_dt();

    //【底盘】
    // 前后（w、s键），限制加速度
    if (device.rc.key.w) { // 前
        if (vx_.client < 0)
            vx_.client = 0.0f;
        vx_.client += config.axy * dt;
        vx_.client = math::clamp(vx_.client, config.vxy_max);
    } else if (device.rc.key.s) { // 后
        if (vx_.client > 0)
            vx_.client = 0.0f;
        vx_.client -= config.axy * dt;
        vx_.client = math::clamp(vx_.client, config.vxy_max);
    } else { // 松开减速
        if (vx_.client > 0) {
            vx_.client -= config.dxy * dt;
            if (vx_.client < 0)
                vx_.client = 0.0f;
        }
        if (vx_.client < 0) {
            vx_.client += config.dxy * dt;
            if (vx_.client > 0)
                vx_.client = 0.0f;
        }
    }
    // 水平（a、d键），限制加速度
    if (device.rc.key.a) { // 左
        if (vy_.client < 0)
            vy_.client = 0.0f;
        vy_.client += config.axy * dt;
        vy_.client = math::clamp(vy_.client, config.vxy_max);
    } else if (device.rc.key.d) { // 右
        if (vy_.client > 0)
            vy_.client = 0.0f;
        vy_.client -= config.axy * dt;
        vy_.client = math::clamp(vy_.client, config.vxy_max);
    } else { // 松开减速
        if (vy_.client > 0) {
            vy_.client -= config.dxy * dt;
            if (vy_.client < 0)
                vy_.client = 0.0f;
        }
        if (vy_.client < 0) {
            vy_.client += config.dxy * dt;
            if (vy_.client > 0)
                vy_.client = 0.0f;
        }
    }
    // 旋转（shift、ctrl键），限制角加速度
    if (device.rc.key.shift) { // 逆时针旋转
        if (wr_.client < 0)
            wr_.client = 0.0f;
        wr_.client += config.ar * dt;
        wr_.client = math::clamp(wr_.client, config.wr_max);
    } else if (device.rc.key.ctrl) { // 顺逆时针旋转
        if (wr_.client > 0)
            wr_.client = 0.0f;
        wr_.client -= config.ar * dt;
        wr_.client = math::clamp(wr_.client, config.wr_max);
    } else { // 松开减速
        if (wr_.client > 0) {
            wr_.client -= config.dr * dt;
            if (wr_.client < 0)
                wr_.client = 0.0f;
        }
        if (wr_.client < 0) {
            wr_.client += config.dr * dt;
            if (wr_.client > 0)
                wr_.client = 0.0f;
        }
    }

    //【云台】
    // yaw（鼠标上下），限制角加速度
    float dwyaw = device.rc.mouse.yaw * config.wyaw - wyaw_.client;
    const float dwyaw_max = config.ayaw * dt;
    if (dwyaw > dwyaw_max)
        dwyaw = dwyaw_max;
    if (dwyaw < -dwyaw_max)
        dwyaw = -dwyaw_max;
    wyaw_.client += dwyaw;
    // pitch（鼠标左右），限制角加速度
    float dwpitch = device.rc.mouse.pitch * config.wpitch - wpitch_.client;
    const float dwpitch_max = config.apitch * dt;
    if (dwpitch > dwpitch_max)
        dwpitch = dwpitch_max;
    if (dwpitch < -dwpitch_max)
        dwpitch = -dwpitch_max;
    wpitch_.client += dwpitch;
    // 自瞄（鼠标右键）
    if (device.rc.mouse.right && device.mavlink.vision.is_detect) { // 使能自瞄且识别到
        gimbal_mode_.client = GIMBAL_ANGLE_MODE;                    // 云台角度控制
        yaw_.client = device.mavlink.vision.yaw;
        pitch_.client = device.mavlink.vision.pitch;
    } else {
        gimbal_mode_.client = GIMBAL_SPEED_MODE; // 云台速度控制
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
    static rmpp::Timer timer_last_click;
    if (device.rc.mouse.right) { // 允许自瞄状态下
        // 双击左键强制开火
        if (!last_left && device.rc.mouse.left) {
            if (timer_last_click.get_dt() < 250 * ms) {
                force_fire = true;
            }
            timer_last_click.update_dt();
        }
        last_left = device.rc.mouse.left;

        if (device.rc.mouse.left) { // 按下左键
            if (force_fire) {       // 强制开火
                is_shoot_.client = true;
            } else { // 自动火控
                is_shoot_.client = device.mavlink.vision.is_fire;
            }
        } else { // 释放左键
            is_shoot_.client = false;
            force_fire = false; // 复位强制开火
        }
    } else { // 非自瞄状态下，鼠标左键开火
        is_shoot_.client = device.rc.mouse.left;
        force_fire = false; // 复位强制开火
    }

    // 刷新ui（f键）
    if (device.rc.key.f) {
        device.ui.init();
    }

    // 英雄调整弹速
    if (device.referee.robot.type == Referee::HERO) {
        static bool last_z = false;
        if (!last_z && device.rc.key.z) {
            config.bullet_speed -= 0.1f;
        }
        last_z = device.rc.key.z;

        static bool last_c = false;
        if (!last_c && device.rc.key.c) {
            config.bullet_speed += 0.1f;
        }
        last_c = device.rc.key.c;
    }

    // 步兵调弹频
    if (device.referee.robot.type == Referee::INFANTRY) {
        static bool last_z = false;
        if (!last_z && device.rc.key.z) {
            config.bullet_freq -= 5.0f;
        }
        last_z = device.rc.key.z;

        static bool last_c = false;
        if (!last_c && device.rc.key.c) {
            config.bullet_freq += 5.0f;
        }
        last_c = device.rc.key.c;

        config.bullet_freq = math::clamp(config.bullet_freq, 5, 25);
    }
}

void Robot::handle_mavlink() {
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

    device.mavlink.on_loop();
}

void Robot::handle_chassis() {
    // 设置底盘速度
    const float vx = math::clamp(this->vx_.rc + this->vx_.client + this->vx_.software, config.vxy_max);
    const float vy = math::clamp(this->vy_.rc + this->vy_.client + this->vy_.software, config.vxy_max);
    const float wr = math::clamp(this->wr_.rc + this->wr_.client + this->wr_.software, config.wr_max);
    device.chassis.set_speed(vx, vy, wr);

    // 设置前进正方向
    device.chassis.set_gimbal_yaw(device.gimbal.yaw.ecd.measure);

    // 设置功率限制
    device.chassis.set_power_limit(device.referee.chassis.power_limit);

    // 设置缓冲能量
    device.chassis.set_buffer_energy(device.referee.chassis.buffer_energy);

    device.chassis.on_loop();
}

void Robot::handle_gimbal() {
    // 设置角度或速度
    if (gimbal_mode_.rc == GIMBAL_ANGLE_MODE) {
        device.gimbal.set_angle(yaw_.rc, pitch_.rc);
        device.gimbal.set_speed(0.0f, 0.0f);
    } else if (gimbal_mode_.client == GIMBAL_ANGLE_MODE) {
        device.gimbal.set_angle(yaw_.client, pitch_.client);
        device.gimbal.set_speed(0.0f, 0.0f);
    } else if (gimbal_mode_.software == GIMBAL_ANGLE_MODE) {
        device.gimbal.set_angle(yaw_.software, pitch_.software);
        device.gimbal.set_speed(0.0f, 0.0f);
    } else {
        const float yaw_speed = math::clamp(this->wyaw_.rc + this->wyaw_.client + this->wyaw_.software, config.wyaw);
        const float pitch_speed = math::clamp(this->wpitch_.rc + this->wpitch_.client + this->wpitch_.software, config.wpitch);
        device.gimbal.set_speed(yaw_speed, pitch_speed);
    }

    // 设置小陀螺前馈
    device.gimbal.set_chassis_wr(device.chassis.wr.measure);

    device.gimbal.on_loop();
}

void Robot::handle_shooter() {
    // 摩擦轮
    device.shooter.set_rub(is_rub_.rc || is_rub_.client || is_rub_.software);

    // 拨弹电机
    if (is_shoot_.rc) { // 遥控器射击，pitch摇杆控制弹频
        device.shooter.set_shoot(true);
        device.shooter.set_bullet_freq(config.bullet_freq * device.rc.shoot); // 设置弹频
    } else if (is_shoot_.client || is_shoot_.software) {
        device.shooter.set_shoot(true);
        device.shooter.set_bullet_freq(config.bullet_freq); // 设置弹频
    } else {
        device.shooter.set_shoot(false);
    }

    // 设置弹速
    device.shooter.set_bullet_speed(config.bullet_speed);

    // 枪口热量保护
    device.shooter.set_heat_protect(device.referee.shooter.heat_remain < config.heat_protect);

    device.shooter.on_loop();
}

void Robot::handle_referee() {
    device.referee.set_yaw(device.gimbal.yaw.ecd.measure, device.gimbal.yaw.imu.measure);
    device.referee.on_loop();
}

void Robot::handle_ui() {
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
    device.ui.cap = 0.0f;                             // 超级电容剩余能量比例
    device.ui.wr = device.chassis.wr.measure;         // 底盘旋转速度
    if (device.referee.robot.type == Referee::HERO) { // 英雄显示弹速
        device.ui.bullet = config.bullet_speed;
        device.ui.bullet_is_speed = true;
    } else if (device.referee.robot.type == Referee::INFANTRY) { // 步兵显示弹频
        device.ui.bullet = config.bullet_freq;
        device.ui.bullet_is_speed = false;
    }
    device.ui.shoot = device.shooter.shoot_current; // 拨弹电机电流

    // 机器人状态
    device.chassis.update_ui(device.ui);                                           // 底盘
    device.gimbal.update_ui(device.ui);                                            // 云台
    device.shooter.update_ui(device.ui);                                           // 发射机构
    device.ui.robot.referee = device.referee.is_connect ? Ui::GREEN : Ui::PINK;    // 裁判系统
    device.ui.robot.aim = device.mavlink.is_connect_vision ? Ui::GREEN : Ui::PINK; // 自瞄

    device.ui.on_loop();
}
