#pragma once

#include "bsp/timer.hpp"
#include "math/unit.hpp"
#include "motor/vesc.hpp"
#include "pid/pid.hpp"

// 17mm 发射机构：双 VESC 摩擦轮速度模式，拨弹电机速度闭环；摩擦轮与拨弹电机各自处理掉线
class Shooter17mm {
  private:

    // 设备绑定
    struct Motors {
        Vesc &rub1, &rub2; // 摩擦轮
        Motor& shoot;      // 拨弹电机
    } motor;

  public:
    // 发射机构配置
    struct Config {
        const Pid::Config* shoot_pid_config;
        float bullet_per_angle = 1.0f / rev; // 每弧度发射弹数，默认每转一发
        float rub_radius = 0.03f;            // 摩擦轮半径
        float block_time = 0.2f;             // 卡弹检测时间，s；拨弹 PID 输出持续饱和达到该时间判定卡弹
        float reverse_time = 0.25f;          // 卡弹倒转时间
        uint16_t heat_protect = 0;           // 剩余热量保护阈值
    };

  private:
    const Config config;

    // 指令
    bool is_enable = false;

  public:
    // 弹速 (m/s)
    struct {
        float ref = 0, measure = 0;
    } bullet_speed;

  private:
    // 弹频 (shots/s)
    struct {
        float ref = 0, measure = 0;
    } bullet_freq;

  public:
    // 摩擦轮线速度，m/s
    float rub_speed[2]{};

  private:
    bool is_heat_protect = false; // 热量保护状态

    // 拨弹状态机
    enum {
        STOP,
        FEED,
        REVERSE,
    } state = STOP;

    float shoot_output = 0; // 拨弹 PID 输出，单位随电机 set_output()；update() 末尾保存，失能时为 0

    // 拨弹闭环
    Pid shoot_pid;

    // 跨周期状态
    float block_elapsed = 0;
    float reverse_elapsed = 0;

    Timer update_timer;

  public:
    Shooter17mm(const Motors& motor, const Config& config);

    // 设置发射机构使能/失能
    void set_enable(bool is_enable);

    // 设置弹速参考值，m/s；0 关闭摩擦轮，保持到下次设置
    void set_bullet_speed(float bullet_speed);

    // 设置弹频参考值，发/秒；0 停止拨弹，负值反转，保持到下次设置
    void set_bullet_freq(float bullet_freq);

    // 设置裁判剩余热量，允许过热时的负值；未设置时不限制
    void set_heat_remain(int32_t heat_remain);

    // 每个控制周期调用
    void update();
};
