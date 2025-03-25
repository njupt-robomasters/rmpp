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
#include "chassis_4wheel.hpp"
#include "imu.hpp"
#include "module/gimbal.hpp"
#include "referee.hpp"
#include "ui.hpp"
#include "rv2.hpp"

#include "task/task_led.h"
#include "task/task_chassis.h"
#include "task/task_imu.h"
#include "task/task_gimbal.h"
#include "task/task_protocol.h"

class Settings {
public:
    // 底盘
    PID::param_t wheel_pid = {.kp = 10000.0f, .ki = 50.0f};

    // 云台
    float pitch_aps_max = 60.0f; // 【单位：度/s】
    float yaw_aps_max = 180.0f; // 【单位：度/s】
    const float mouse_x_max = 300.0f;
    const float mouse_y_max = 300.0f;
    const IMU::param_t imu_param = {.yaw = 0, .pitch = 0, .roll = 180};
    PID::param_t pitch_pid = {.kp = 1.0f, .kd = 0.014f}; // pitch
    PID::param_t yaw_pid = {.kp = 0.7f, .kd = 0.012f}; // yaw
    PID::param_t shoot_pid = {.kp = 10, .ki = 40, .i_limit = 5, .max_out = 10}; // shoot
};

class Status {
public:
    // 底盘
    float chassis_vxy_max = 2.0f; // 前后左右平移速度【单位：m/s】
    float chassis_avy = 0.5f; // 平移方向加速度【单位：m/s^2】
    float chassis_vr_rpm = 10.0f; // 旋转角速度【单位：rpm】

    // 云台
    Gimbal::mode_e gimbal_mode = Gimbal::IMU_MODE;
    bool is_prepare_shoot = false;
    bool is_shoot = false;
    bool is_rv2_mode = false;
    float shoot_freq = 15.0f; // 【单位：Hz】
};

extern Settings settings;
extern Status status;
extern DJ6 dj6;
extern Chassis4Wheel chassis;
extern IMU imu;
extern Gimbal gimbal;
extern Referee referee;
extern UI ui;
extern RV2 rv2;
