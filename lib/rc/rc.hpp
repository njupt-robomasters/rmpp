#pragma once

#include "fsi6x.hpp"
#include "vt13.hpp"

class Rc {
  public:
    // 速度使用 m/s、rad/s，加减速度使用 m/s^2、rad/s^2
    struct Config {
        float vxy_max{}, wr_max{};
        float axy{}, dxy{}, ar{}, dr{};
        float wyaw{}, wpitch{}, ayaw{}, apitch{};
    } config;

    bool is_enable = false;

    bool is_rub = false, is_auto_aim = false, is_fn = false;
    // 校准、掉头和调参仅在按下的周期输出，刷新 UI 保留按住生效
    bool is_calib = false, is_turn_back = false, is_refresh_ui = false;
    int8_t bullet_step = 0;
    // 人工输入之和；app 合并自主指令后再按 config 限幅
    float vx{}, vy{}, wr{}, wyaw{}, wpitch{};

    struct ShootCommand {
        bool is_shoot = false;
        float rate = 0;
    };

    // 引用应用持有的遥控器，实例需在管理器使用期间保持有效
    Rc(const Config& config, FSi6X& fsi6x, VT13& vt13);
    Rc(const Config& config, FSi6X& fsi6x);
    Rc(const Config& config, VT13& vt13);

    // 下次 update 时保持使能，断联输入仍正常清零，默认关闭
    void set_ignore_disconnect(bool ignore);

    // 更新已接入遥控器的断联状态并汇总控制量，需要在循环中调用
    void update();

    // 仅传入视觉开火许可，保留遥控器优先、键鼠次之的规则
    ShootCommand shoot_command(bool vision_fire) const;

  private:
    FSi6X* fsi6x_ = nullptr;
    VT13* vt13_ = nullptr;
    bool ignore_disconnect_ = false;
    Timer timer_update_, timer_click_;
    float keyboard_vx_{}, keyboard_vy_{}, keyboard_wr_{};
    float mouse_wyaw_{}, mouse_wpitch_{};
    bool radio_shoot_ = false, radio_auto_aim_ = false;
    float radio_rate_{};
    bool mouse_left_ = false, mouse_right_ = false;
    bool force_fire_ = false, has_click_ = false;
    bool last_calib_ = false, last_r_ = false, last_z_ = false, last_c_ = false;

    void update_client(float dt);
};
