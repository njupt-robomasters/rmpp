#pragma once

// 杂项
#include "misc/LED.hpp"
#include "misc/Buzzer.hpp"
#include "flashdb/FlashDB.hpp"

// 控制器
#include "rc/RC.hpp"
#include "mavlink/Mavlink.hpp"

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
        UnitFloat<deg_s> yaw_speed_max;
        UnitFloat<deg_s> pitch_speed_max;

        // 发射机构
        UnitFloat<m_s> bullet_speed; // 弹速
        UnitFloat<Hz> bullet_freq;   // 弹频
        uint16_t heat_protect = 0;   // 枪口热量保护
    } config;

    struct device_t {
        LED& led;
        Buzzer& buzzer;
        FlashDB& flashdb;

        RC& rc;
        Mavlink& mavlink;

        Chassis& chassis;
        Gimbal& gimbal;
        Shooter& shooter;

        Referee& referee;
        UI& ui;
    } device;

    Infantry(const config_t& config, const device_t& device);

    void Init();

    // 需要在循环中调用
    void OnLoop();

protected:
    // 底盘
    struct VelInfo {
        UnitFloat<m_s> rc, client, sum;
    } vx, vy, wr;

    enum {
        SPEED_MODE,
        ANGLE_MODE,
    } gimbal_control_mode = SPEED_MODE;

    // 云台
    struct {
        UnitFloat<deg_s> rc, client, sum;
    } yaw_speed, pitch_speed;

    UnitFloat<deg_s> yaw_angle, pitch_angle;

    // 发射机构
    struct {
        bool rc = false, client = false;
    } is_rub, is_shoot;

private:
    // 用于缓加减速
    BSP::Dwt dwt_acc;

    void handleConnect();

    // 控制器
    void handleRC();
    void handleClient();
    void handleMavlink();

    // 执行器
    virtual void handleChassis();
    void handleGimbal();
    void handleShooter();

    // 裁判系统
    void handleReferee();
    void handleUI();
};