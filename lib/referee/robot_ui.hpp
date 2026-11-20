#pragma once

#include "math/angle.hpp"
#include "math/unit.hpp"
#include "referee_sender.hpp"

// 选手端自定义 UI：准星、云台朝向与受击方向、四角提示条与数值、整车示意图
// 启动时 init() 排入初始布局；update() 刷新仪表与设备状态，由 RefereeSender 合并同组更新并限频发送
// RefereeSender 由 app 持有并 send()，调用时传入
class RobotUi {
  public:
    using Color = RefereeSender::Color;
    using enum RefereeSender::Color;

    struct Config {
        // 提示条满量程
        float super_cap_voltage_max = 23.85f; // 电容电压，V
        float chassis_wr_max = 120 * rpm;     // 底盘转速，rad/s
        float bullet_freq_max = 20.0f;        // 弹频，Hz
        float shoot_output_max = 20.0f;       // 拨弹电流/力矩，A / Nm
    };

  private:
    const Config config;

  public:
    // 云台：相对底盘角度画中心圆弧，yaw、pitch 显示为度
    Angle yaw_ecd{}, yaw_angle{}, pitch_angle{};

    // 受击：is_hurt 时在 hurt_dir 方向画粉色圆弧
    bool is_hurt = false;
    Angle hurt_dir{};

    bool is_auto_aim_detect = false; // 自瞄识别到目标，显示准星圆

    // 电容电压，V；底盘转速，rad/s；设定弹频，Hz；拨弹实测电流或力矩，A 或 N·m
    float super_cap_voltage = 0, chassis_wr = 0, bullet_freq = 0, shoot_output = 0;

    // 设备状态颜色；base 为轮腿下板或双 yaw 的底层转轴
    Color w1 = WHITE, w2 = WHITE, w3 = WHITE, w4 = WHITE;
    Color s1 = WHITE, s2 = WHITE, s3 = WHITE, s4 = WHITE;
    Color base = WHITE, yaw = WHITE, pitch = WHITE;
    Color rub_left_stage1 = WHITE, rub_left_stage2 = WHITE, rub_right_stage1 = WHITE, rub_right_stage2 = WHITE;
    Color shoot = WHITE, referee = WHITE, aim = WHITE, cap = WHITE;

    explicit RobotUi(const Config& config);

    // 丢弃排队中的 UI 请求并排入初始布局；启动时及选手端重连后调用
    void init(RefereeSender& sender);

    void update(RefereeSender& sender);

  private:
    void draw_car(RefereeSender& sender, RefereeSender::Operation operation);
};
