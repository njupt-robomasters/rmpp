#pragma once

#include "motor/motor.hpp"
#include "shooter.hpp"

class Shooter17mm : public Shooter {
  public:
    // 发射机构电机
    struct Motors {
        Motor &rub1, &rub2; // 摩擦轮
        Motor& shoot;       // 拨弹电机
    } motor;

    Shooter17mm(const Config& config, const Motors& motor);

    // 设置发射结构使能/失能
    void set_enable(bool is_enable) override;

    // 在自定义UI上显示电机连接状态
    void update_ui(RobotUi& ui) override;

    // 需要在循环中调用
    void update() override;

  private:
    // 摩擦轮线速度
    float rub1_measure_{}, rub2_measure_{};

    // 正解：电机速度 -> 弹速、弹频
    void forward() override;

    // 逆解：弹速、弹频 -> 电机速度
    void backward() override;
};
