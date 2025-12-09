#include "app.hpp"

static void handle_rc() {
    var.vx.rc = dj6.x * cfg.vxy_max;
    var.vy.rc = dj6.y * cfg.vxy_max;
    var.yaw_speed.rc = dj6.yaw * cfg.yaw_max_speed;
    var.pitch_speed.rc = dj6.pitch * cfg.pitch_max_speed;

    // 左拨杆：发射结构
    switch (dj6.switch_left) {
        case DJ6::ERR:
            shooter.SetPrepareShoot(false);
            shooter.SetShoot(false);
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

    // 右拨杆：云台模式、测试小陀螺
    switch (dj6.switch_right) {
        case DJ6::ERR:
            var.vr.rc = 0 * default_unit;
            break;
        case DJ6::UP:
            gimbal.SetMode(Gimbal::ECD_MODE);
            var.vr.rc = 0 * default_unit;
            break;
        case DJ6::MID:
            gimbal.SetMode(Gimbal::IMU_MODE);
            var.vr.rc = 0 * default_unit;
            break;
        case DJ6::DOWN:
            gimbal.SetMode(Gimbal::IMU_MODE);
            var.vr.rc = cfg.vr_max;
            break;
    }
}

static void handle_vt13() {
    var.vx.vt13 = vt13.x * cfg.vxy_max;
    var.vy.vt13 = vt13.y * cfg.vxy_max;
    var.yaw_speed.vt13 = vt13.yaw * cfg.yaw_max_speed;
    var.pitch_speed.vt13 = vt13.pitch * cfg.pitch_max_speed;
}

static void handle_client() {
    var.yaw_speed.client = -vt13.mouse_x * cfg.yaw_max_speed;
    var.pitch_speed.client = -vt13.mouse_y * cfg.pitch_max_speed;
}

static void handle_nav() {}

void loop_remote() {
    dj6.OnLoop();
    handle_rc();

    vt13.OnLoop();
    handle_vt13();

    handle_client();
    handle_nav();

    var.vx.sum = unit::clamp(var.vx.rc + var.vx.vt13 + var.vx.client + var.vx.nav, cfg.vxy_max);
    var.vy.sum = unit::clamp(var.vy.rc + var.vy.vt13 + var.vy.client + var.vy.nav, cfg.vxy_max);
    var.vr.sum = unit::clamp(var.vr.rc + var.vr.vt13 + var.vr.client + var.vr.nav, cfg.vr_max);

    var.pitch_speed.sum = unit::clamp(var.pitch_speed.rc + var.pitch_speed.vt13 + var.pitch_speed.client + var.pitch_speed.nav, cfg.pitch_max_speed);
    var.yaw_speed.sum = unit::clamp(var.yaw_speed.rc + var.yaw_speed.vt13 + var.yaw_speed.client + var.yaw_speed.nav, cfg.yaw_max_speed);
}