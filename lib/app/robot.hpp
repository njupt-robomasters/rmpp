#pragma once

// 控制器
#include "rc/vt13.hpp"
#include "rc/dj6.hpp"
#include "mavlink/mavlink.hpp"
#include "referee/referee.hpp"

// 传感器
#include "imu/imu.hpp"

// 执行器
#include "app/chassis.hpp"
#include "app/gimbal.hpp"
#include "app/shooter.hpp"

#include "ui/ui.hpp"

class Robot {
public:
    struct config_t {
        // 底盘速度
        UnitFloat<m_s> vxy_max; // 前后左右平移速度
        UnitFloat<rpm> wr_max;  // 旋转角速度
        UnitFloat<m_ss> axy;    // 加速度（仅键盘操作有效）
        UnitFloat<m_ss> dxy;    // 减速度（仅键盘操作有效）

        // 云台速度
        UnitFloat<deg_s> yaw_max;
        UnitFloat<deg_s> pitch_max;

        // 发射机构
        UnitFloat<m_s> bullet_speed; // 弹速
        UnitFloat<Hz> bullet_freq;   // 弹频
        uint16_t heat_protect = 0;   // 枪口热量保护
    };

    Robot(const config_t& config,
          DJ6& dj6, VT13& vt13, Mavlink& mavlink,
          IMU& imu,
          Chassis_Template& chassis, Gimbal_Template& gimbal, Shooter_Template& shooter,
          Referee& referee, UI& ui) :
        config(config),
        dj6(dj6), vt13(vt13), mavlink(mavlink),
        imu(imu),
        chassis(chassis), gimbal(gimbal), shooter(shooter),
        referee(referee), ui(ui) {}

    void SetEnable(bool is_enable);

    void OnLoop();

private:
    static constexpr float HIT_TIMEOUT = 1;

    // 配置
    const config_t& config;

    // 控制器
    DJ6& dj6;
    VT13& vt13;
    Mavlink& mavlink;

    // 传感器
    IMU& imu;

    // 执行器
    Chassis_Template& chassis;
    Gimbal_Template& gimbal;
    Shooter_Template& shooter;

    // 裁判系统交互
    Referee& referee;
    UI& ui;

    // 底盘运动
    struct {
        UnitFloat<m_s> rc, vt13, client, nav, sum;
    } vx, vy, wr;

    // 云台运动
    struct {
        UnitFloat<deg_s> rc, vt13, client, nav, sum;
    } yaw_speed, pitch_speed;

    struct {
        BSP::Dwt dwt;
        Angle<deg> yaw_imu;
    } hit;

    UnitFloat<m_ss> ax, ay;
    UnitFloat<deg_ss> ar;

    // 控制器
    void handle_disconnect();
    void handle_dj6();
    void handle_vt13();
    void handle_mavlink();

    // 传感器
    void handle_imu();

    // 执行器
    void handle_chassis();
    void handle_gimbal();
    void handle_shooter();

    // 裁判系统交互
    void handle_referee();
    void handle_ui();
};