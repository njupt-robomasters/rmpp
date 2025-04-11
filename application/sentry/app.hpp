#pragma once

#include "app.h"

#include "cmsis_os.h"

#include "bsp_led.h"
#include "bsp_dwt.h"
#include "bsp_can.h"
#include "bsp_pwm.h"
#include "bsp_uart.h"
#include "bsp_cdc.h"

#include "dj6.hpp"
#include "module/chassis.hpp"
#include "referee.hpp"
#include "nav.hpp"

#include "task/task_led.h"
#include "task/task_chassis.h"
#include "task/task_protocol.h"

class Settings {
public:
    // 底盘PID参数
    PID::param_t servo_pid = {.kp = 0.6f};
    PID::param_t wheel_pid = {.kp = 10000.0f, .ki = 50.0f};

    // 底盘运动参数
    float vxy_max = 3.0f; // 前后左右平移速度【单位：m/s】
    float rpm_max = 120.0f; // 旋转角速度【单位：rpm】

    // 导航参数
    uint16_t go_home_hp = 200;
    float home_x = -0.375, home_y = -0.375;
    float center_x = 2.31, center_y = 4.93;
    float center_range = 0.375;
    // float home_x = 0, home_y = 0;
    // float center_x = 0, center_y = 1;
    // float center_range = 0.375;
};

class Status {
public:
    struct {
        float vx = 0, vy = 0, rpm = 0;
    } rc{}, nav{};

    bool is_nav_on = false;
    float nav_target_x = 0, nav_target_y = 0;

    enum {
        GO_CENTER,
        IN_CENTER,
        GO_HOME,
        IN_HOME
    } nav_state = GO_CENTER;

    enum {
        FORCE_HOME,
        FORCE_CENTER,
        FOLLOW_STATEMACHINE
    } nav_policy = FORCE_HOME;

    bool ignore_rc_disconnect = false;
};

extern Settings settings;
extern Status status;
extern DJ6 dj6;
extern Chassis chassis;
extern Referee referee;
extern NAV nav;
