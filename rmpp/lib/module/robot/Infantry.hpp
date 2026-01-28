#pragma once

// 控制器
#include "rc/FSi6X.hpp"
#include "rc/VT13.hpp"
#include "mavlink/Mavlink.hpp"

// 传感器
#include "imu/IMU.hpp"

// 执行器
#include "module/chassis/Chassis.hpp"
#include "module/gimbal/Gimbal.hpp"
#include "module/shooter/Shooter.hpp"

// 裁判系统
#include "referee/Referee.hpp"
#include "ui/UI.hpp"

class Infantry {
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
    } config;

    struct device_t {
        FSi6X& fsi6x;
        VT13& vt13;
        Mavlink& mavlink;

        IMU& imu;

        Chassis& chassis;
        Gimbal& gimbal;
        Shooter& shooter;

        Referee& referee;
        UI& ui;
    } device;

    Infantry(const config_t& config, const device_t& device);

    // 需要在循环中调用
    void OnLoop();

private:
    static constexpr float HIT_TIMEOUT = 1;

    // 底盘
    struct {
        UnitFloat<m_s> fsi6x, vt13, client, mavlink, sum;
    } vx, vy, wr;

    // 云台
    struct {
        UnitFloat<deg_s> fsi6x, vt13, client, mavlink, sum;
    } yaw_speed, pitch_speed;

    // 发射机构
    struct {
        bool fsi6x = false, vt13 = false, client = false;
    } is_rub, is_shoot;

    // 用于缓加减速
    BSP::Dwt dwt_acc;

    void setEnable(bool is_enable);

    void handle_disconnect();

    // 控制器
    void handle_fsi6x();
    void handle_vt13_rc();
    void handle_vt13_client();
    void handle_mavlink();

    // 传感器
    void handle_imu();

    // 执行器
    void handle_chassis();
    void handle_gimbal();
    void handle_shooter();

    // 裁判系统
    void handle_referee();
    void handle_ui();
};