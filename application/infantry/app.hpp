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
#include "imu.hpp"
#include "module/chassis.hpp"
#include "module/gimbal.hpp"
#include "referee.hpp"
#include "ui.hpp"
#include "rv2.hpp"
#include "super_capacity.hpp"

#include "task/task_led.h"
#include "task/task_chassis.h"
#include "task/task_imu.h"
#include "task/task_gimbal.h"
#include "task/task_protocol.h"

class Settings {
public:
    // 底盘
    float wheel_radius = 0.063f; // 轮子半径【单位：m】
    float chassis_radius = 0.225f; // 底盘半径【单位：m】
    // 操作手控制参数
    // 移动
    float axy = 1.0f; // 移动加速度【单位：m/s^2】
    float vxy_limit_min = 1.0f; // 【单位：m/s】
    float vxy_limit_max = 5.0f; // 【单位：m/s】
    float vxy_limit_per_press = 1.0f; // 【单位：m/s】
    // 旋转
    float rpm_max = 100.0f; // 最大小陀螺转速【单位：rpm】
    float rpm_per_press = 20.0f; // 小陀螺转速调节挡位【单位：rpm】
    // 底盘PID参数
    PID::param_t wheel_pid = {.kp = 10000.0f, .ki = 50.0f};

    // 云台
    float pitch_aps_max = 60.0f; // 【单位：度/s】
    float yaw_aps_max = 180.0f; // 【单位：度/s】
    float mouse_x_max = 300.0f;
    float mouse_y_max = 300.0f;
    // IMU参数
    IMU::param_t imu_param = {.yaw = 0, .pitch = 0, .roll = 180};
    IMU::calib_t imu_calib = {-0.00298113562, 0.00259035057, -0.00091807748, 9.82509899};
    // 云台PID参数
    PID::param_t pitch_pid = {.kp = 1.0f, .kd = 0.014f}; // pitch
    PID::param_t yaw_pid = {.kp = 0.7f, .kd = 0.012f}; // yaw
    PID::param_t shoot_pid = {.kp = 10, .ki = 40, .i_limit = 5, .max_out = 10}; // shoot
};

class Status {
public:
    // 底盘
    struct {
        float vxy_limit = 5.0f; // 移动最大速度【单位：m/s】
        float rpm = 0.0f; // 当前小陀螺旋转速度【单位：rpm】
        struct {
            float vx = 0, vy = 0;
        } rc{}, video{};
    } chassis;

    // 云台
    struct {
        Gimbal::mode_e mode = Gimbal::IMU_MODE;

        struct {
            float pitch_speed = 0, yaw_speed = 0;
        } rc{}, video{};

        bool is_prepare_shoot = false;
        bool is_shoot = false;
        float shoot_freq = 10.0f; // 【单位：Hz】
        bool is_rv2_mode = false;
    } gimbal;

    bool ignore_rc_disconnect = false;
};

extern Settings settings;
extern Status status;
extern DJ6 dj6;
extern IMU imu;
extern Chassis chassis;
extern Gimbal gimbal;
extern Referee referee;
extern UI ui;
extern RV2 rv2;
extern SuperCapacity superCapacity;
