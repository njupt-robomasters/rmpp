#pragma once

#include "app.h"

#include "cmsis_os.h"

#include "bsp_led.h"
#include "bsp_dwt.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "bsp_cdc.h"
#include "bsp_pwm.h"

#include "dj6.hpp"
#include "referee.hpp"

#include "module/chassis.hpp"

#include "task/task_led.h"
#include "task/task_chassis.h"
#include "task/task_protocol.h"

class Settings {
public:
    // PID参数
    PID::param_t servo_pid = {.kp = 0.7f, .kd = 0.012f};
    PID::param_t wheel_pid = {.kp = 10000.0f, .ki = 50.0f};

    float chassis_vxy_max = 2.0f; // 前后左右平移速度【单位：m/s】
    float chassis_vr_rpm = 60.0f; // 旋转角速度【单位：rpm】
};

extern Settings settings;
extern DJ6 dj6;
extern Referee referee;
extern Chassis chassis;
