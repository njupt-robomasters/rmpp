#pragma once

#include "bsp/can.hpp"
#include "bsp/timer.hpp"
#include "math/angle.hpp"
#include "math/unit.hpp"

class Ui {
  public:
    struct Config {
        // CAN通信参数
        Can can;
        uint32_t master_id = 0;
        uint32_t slave_id = 1;

        // 量程单位：比值、rad/s、m/s（英雄）或发每秒、A
        float max_cap = 1.0f;
        float max_wr = 120 * rpm;
        float max_bullet = 25.0f;
        float max_shoot = 10.0f;
    } config;

    enum Color {
        MAIN = 0,   // 红/蓝（己方颜色）
        YELLOW = 1, // 黄色
        GREEN = 2,  // 绿色
        ORANGE = 3, // 橙色
        PURPLE = 4, // 紫红色
        PINK = 5,   // 粉色
        CYAN = 6,   // 青色
        BLACK = 7,  // 黑色
        WHITE = 8,  // 白色
    };

    // 云台角度
    Angle yaw_ecd;    // 云台相对底盘角度
    Angle yaw, pitch; // 云台IMU角度

    // 伤害方向
    bool is_hurt = false;
    Angle hurt_dir;

    // 自瞄识别
    bool is_detect = false;

    // 提示条
    float cap{};                  // 超级电容剩余能量比例
    float wr{};                   // 底盘旋转速度
    float bullet{};               // 英雄弹速/步兵弹频
    bool bullet_is_speed = false; // 为真时表示弹速（m/s），否则表示发射频率（发每秒）
    float shoot{};                // 拨弹电机电流

    // 机器人状态
    struct {
        Color w1 = GREEN, w2 = GREEN, w3 = GREEN, w4 = GREEN;
        Color s1 = GREEN, s2 = GREEN, s3 = GREEN, s4 = GREEN;
        Color yaw1 = GREEN, yaw2 = GREEN, pitch = GREEN;
        Color rub1 = GREEN, rub2 = GREEN, rub3 = GREEN, rub4 = GREEN;
        Color shoot = GREEN;
        Color referee = GREEN;
        Color aim = GREEN;
    } robot;

    Ui(const Config& config);

    static void add_can_data(const uint8_t data[], size_t len);

    void init();

    // 需要在循环中调用
    void update();

    void send_can_cmd();

  private:
    // CAN发送队列
    static constexpr size_t TXBUF_SIZE = 1024;
    static uint8_t txbuf[TXBUF_SIZE];
    static size_t head, tail;

    // 发送频率
    static constexpr float SEND_FREQ = 30;

    enum {
        INIT,
        UPDATE
    } state_ = INIT;

    size_t init_index_ = 0;
    size_t update_index_ = 0;

    Timer timer_; // 用于控制发送频率

    // 更新UI库
    void update_lib() const;
};
