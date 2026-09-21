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

class Robot {
public:
    struct config_t {
        // SI: m/s, rad/s, m/s^2, rad/s^2; firing rate is shots/s.
        // 底盘速度
        float vxy_max{}; // 移动速度
        float wr_max{};  // 旋转角速度
        float axy{};     // 移动加速度（仅键盘操作有效）
        float dxy{};     // 移动减速度（仅键盘操作有效）
        float ar{};      // 旋转角加速度（仅键盘操作有效）
        float dr{};      // 旋转角减速度（仅键盘操作有效）

        // 云台速度
        float wyaw{};   // yaw角速度
        float wpitch{}; // pitch角速度
        float ayaw{};   // yaw角加速度（仅鼠标操作有效）
        float apitch{}; // pitch角加速度（仅鼠标操作有效）

        // 发射机构
        float bullet_speed{};        // 弹速
        float bullet_freq{};         // 弹频
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

    Robot(const config_t& config, const device_t& device);

    void Init();

    // 需要在循环中调用
    void OnLoop();

protected:
    // 底盘
    struct {
        float rc{}, client{}, software{};
    } vx, vy, wr;

    enum gimbal_mode_e {
        GIMBAL_SPEED_MODE,
        GIMBAL_ANGLE_MODE,
    };

    // 云台控制模式
    struct {
        gimbal_mode_e rc = GIMBAL_SPEED_MODE, client = GIMBAL_SPEED_MODE, software = GIMBAL_SPEED_MODE;
    } gimbal_mode;

    // 云台速度
    struct {
        float rc{}, client{}, software{};
    } wyaw, wpitch;

    // 云台角度 (rad, ordinary values until assigned to Angle)
    struct {
        float rc{}, client{}, software{};
    } yaw, pitch;

    // 发射机构
    struct {
        bool rc = false, client = false, software = false;
    } is_rub, is_shoot;

private:
    // 控制器
    void handleRC();
    void handleClient();
    void handleMavlink();

    // 执行器
    void handleChassis();
    void handleGimbal();
    void handleShooter();

    // 裁判系统
    void handleReferee();
    void handleUI();
};
