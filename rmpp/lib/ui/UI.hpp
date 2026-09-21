#pragma once

#include "bsp/bsp.hpp"
#include "math/Angle.hpp"
#include "math/Conversions.hpp"

using math::Angle;
using math::rpm;

class UI {
public:
    struct config_t {
        // CAN通信参数
        uint8_t can_port = 1;
        uint32_t master_id = 0;
        uint32_t slave_id = 1;

        // SI ranges: ratio, rad/s, m/s (hero) or shots/s, A.
        float max_cap = 1.0f;
        float max_wr = 120 * rpm;
        float max_bullet = 25.0f;
        float max_shoot = 10.0f;
    } config;

    enum color_e {
        MAIN   = 0, // 红/蓝（己方颜色）
        YELLOW = 1, // 黄色
        GREEN  = 2, // 绿色
        ORANGE = 3, // 橙色
        PURPLE = 4, // 紫红色
        PINK   = 5, // 粉色
        CYAN   = 6, // 青色
        BLACK  = 7, // 黑色
        WHITE  = 8, // 白色
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
    bool bullet_is_speed = false; // m/s when true, shots/s otherwise
    float shoot{};                // 拨弹电机电流

    // 机器人状态
    struct {
        color_e w1 = GREEN, w2 = GREEN, w3 = GREEN, w4 = GREEN;
        color_e s1 = GREEN, s2 = GREEN, s3 = GREEN, s4 = GREEN;
        color_e yaw1 = GREEN, yaw2 = GREEN, pitch = GREEN;
        color_e rub1 = GREEN, rub2 = GREEN, rub3 = GREEN, rub4 = GREEN;
        color_e shoot = GREEN;
        color_e referee = GREEN;
        color_e aim = GREEN;
    } robot;

    UI(const config_t& config);

    static void AddCanData(const uint8_t data[], size_t len);

    void Init();

    // 需要在循环中调用
    void OnLoop();

    void SendCanCmd();

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
    } state = INIT;

    size_t init_index = 0;
    size_t update_index = 0;

    BSP::Dwt dwt; // 用于控制发送频率

    // 更新UI库
    void updateLib() const;
};
