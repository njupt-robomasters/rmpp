#pragma once

// 控制器
#include "rc/vt13.hpp"
#include "rc/dj6.hpp"
#include "rc/nuc.hpp"
#include "referee/referee.hpp"

// 传感器
#include "imu/imu.hpp"

// 执行器
#include "app/chassis.hpp"
#include "app/gimbal.hpp"
#include "app/shooter.hpp"

class Control {
public:
    struct speed_t {
        // 底盘速度
        UnitFloat<m_s> vxy_max; // 前后左右平移速度
        UnitFloat<rpm> vr_max;  // 旋转角速度
        UnitFloat<m_ss> axy;

        // 云台速度
        UnitFloat<deg_s> yaw_max;
        UnitFloat<deg_s> pitch_max;
    };

    Control(const speed_t& speed,
            DJ6& dj6, VT13& vt13, Referee& referee, NUC& nuc,
            IMU& imu,
            Chassis_Template& chassis, Gimbal_Template& gimbal, Shooter_Template& shooter) :
        speed(speed),
        dj6(dj6), vt13(vt13), referee(referee), nuc(nuc),
        imu(imu),
        chassis(chassis), gimbal(gimbal), shooter(shooter) {}

    void SetEnable(bool is_enable);

    void OnLoop();

private:
    // 底盘运动
    struct {
        UnitFloat<m_s> rc, vt13, client, nav, sum;
    } vx, vy, vr;

    // 云台运动
    struct {
        UnitFloat<deg_s> rc, vt13, client, nav, sum;
    } yaw_speed, pitch_speed;

    // 速度配置
    const speed_t& speed;

    // 控制器
    DJ6& dj6;
    VT13& vt13;
    Referee& referee;
    NUC& nuc;

    // 传感器
    IMU& imu;

    // 执行器
    Chassis_Template& chassis;
    Gimbal_Template& gimbal;
    Shooter_Template& shooter;

    // 控制器
    void handle_disconnect();
    void handle_dj6();
    void handle_vt13();
    void handle_nuc();
    void handle_referee();

    // 传感器
    void handle_imu();

    // 执行器
    void handle_chassis();
    void handle_gimbal();
    void handle_shooter();
};