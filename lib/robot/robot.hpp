#pragma once

// 杂项
#include "flashdb/flash_db.hpp"
#include "misc/buzzer.hpp"
#include "misc/led.hpp"

// 控制器
#include "mavlink/mavlink.hpp"
#include "rc/rc.hpp"

// 执行器
#include "module/chassis/chassis.hpp"
#include "module/gimbal/gimbal.hpp"
#include "module/shooter/shooter.hpp"

// 裁判系统
#include "referee/referee.hpp"
#include "ui/ui.hpp"

class Robot {
  public:
    struct Config {
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
        float bullet_speed{};      // 弹速
        float bullet_freq{};       // 弹频
        uint16_t heat_protect = 0; // 枪口热量保护
    } config;

    struct Device {
        Led& led;
        Buzzer& buzzer;
        FlashDb& flashdb;

        Rc& rc;
        Mavlink& mavlink;

        Chassis& chassis;
        Gimbal& gimbal;
        Shooter& shooter;

        Referee& referee;
        Ui& ui;
    } device;

    Robot(const Config& config, const Device& device);

    void init();

    // 需要在循环中调用
    void on_loop();

  protected:
    // 底盘
    struct {
        float rc{}, client{}, software{};
    } vx_, vy_, wr_;

    enum GimbalMode {
        GIMBAL_SPEED_MODE,
        GIMBAL_ANGLE_MODE,
    };

    // 云台控制模式
    struct {
        GimbalMode rc = GIMBAL_SPEED_MODE, client = GIMBAL_SPEED_MODE, software = GIMBAL_SPEED_MODE;
    } gimbal_mode_;

    // 云台速度
    struct {
        float rc{}, client{}, software{};
    } wyaw_, wpitch_;

    // 云台角度 (rad, ordinary values until assigned to Angle)
    struct {
        float rc{}, client{}, software{};
    } yaw_, pitch_;

    // 发射机构
    struct {
        bool rc = false, client = false, software = false;
    } is_rub_, is_shoot_;

  private:
    // 控制器
    void handle_rc();
    void handle_client();
    void handle_mavlink();

    // 执行器
    void handle_chassis();
    void handle_gimbal();
    void handle_shooter();

    // 裁判系统
    void handle_referee();
    void handle_ui();
};
