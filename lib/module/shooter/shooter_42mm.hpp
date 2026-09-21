#pragma once

#include "math/angle.hpp"
#include "motor/motor.hpp"
#include "shooter.hpp"

using math::Angle;

class Shooter42mm : public Shooter {
  public:
    // 发射机构电机
    struct Motors {
        Motor &rub1, &rub2, &rub3, &rub4; // 摩擦轮
        Motor& shoot;                     // 拨弹电机
    } motor;

    Shooter42mm(const Config& config, const Motors& motor);

    // 设置发射结构使能/失能
    void set_enable(bool is_enable) override;

    // 在自定义UI上显示电机连接状态
    void update_ui(Ui& ui) override;

    // 需要在循环中调用
    void on_loop() override;

  private:
    static constexpr float RUB12_RATIO = 0.5f;
    static constexpr float SHOOT_FINISH_RATIO = 0.95; // 用于检查拨弹完成
    static constexpr float SHAKE_FREQ = 1;            // 抖动频率
    static constexpr float SHANE_AMP = 1;             // 抖动幅度（一个弹 = 1）

    // 摩擦轮线速度
    float rub1_measure_{}, rub2_measure_{}, rub3_measure_{}, rub4_measure_{};

    // 用于检测发送信号上升沿
    bool is_shoot_last_ = false;

    // 用于抖动下弹
    Angle shake_upper_;
    rmpp::Timer timer_shake_;

    enum {
        FREE,
        SHOOT,
        SHAKE,
    } state_ = FREE;

    // 正解：电机速度 -> 弹速、弹频
    void forward() override;

    // 逆解：弹速、弹频 -> 电机速度
    void backward() override;
};
