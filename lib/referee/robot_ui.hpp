#pragma once

#include "math/angle.hpp"
#include "math/unit.hpp"
#include "referee_sender.hpp"

// 选手端自定义 UI：准星、云台朝向与受击方向、四角提示条与数值、整车示意图
// 启动时 init() 排入初始布局；update() 每周期排入仪表 MODIFY，排队中的旧仪表被替换，由 RefereeSender 按 30 Hz 发出
// RefereeSender 由 app 持有并 send()，调用时传入
class RobotUi {
  public:
    using Color = RefereeSender::Color;
    using enum RefereeSender::Color;

    struct Config {
        // 提示条满量程
        float cap_max = 23.85f; // 超电能量比例
        float wr_max = 120 * rpm; // 底盘转速，rad/s
        float bullet_max = 20.0f; // 弹频，Hz
        float shoot_max = 20.0f; // 拨弹电流，A
    };

    // 显示量，app 每周期填写
    struct Display {
        // 云台：相对底盘角度画中心圆弧，yaw、pitch 显示为度
        Angle yaw_ecd{}, yaw{}, pitch{};

        // 受击：is_hurt 时在 hurt_dir 方向画粉色圆弧
        bool is_hurt = false;
        Angle hurt_dir{};

        bool is_detect = false; // 自瞄识别到目标，显示准星圆

        // 提示条与数值：超电能量比例；底盘转速，rad/s；弹频或弹速；拨弹电流，A
        float cap = 0, wr = 0, bullet = 0, shoot = 0;

        // 设备状态颜色；当前 referee 用于云台圆弧、aim 用于准星圆，其余暂未绘制
        Color w1 = GREEN, w2 = GREEN, w3 = GREEN, w4 = GREEN;
        Color s1 = GREEN, s2 = GREEN, s3 = GREEN, s4 = GREEN;
        Color yaw1 = GREEN, yaw2 = GREEN, pitch_motor = GREEN;
        Color rub1 = GREEN, rub2 = GREEN, rub3 = GREEN, rub4 = GREEN;
        Color shooter = GREEN, referee = GREEN, aim = GREEN;
    };

  private:
    const Config config;

  public:
    explicit RobotUi(const Config& config);

    // 丢弃排队中的 UI 请求并排入初始布局；启动时及选手端重连后调用
    void init(RefereeSender& sender);

    void update(RefereeSender& sender, const Display& display);
};
