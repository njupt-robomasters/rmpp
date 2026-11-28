#pragma once

#include "bsp/timer.hpp"
#include "math/unit.hpp"
#include "motor/vesc.hpp"
#include "pid/pid.hpp"

// 17mm 发射机构：双 VESC 摩擦轮速度模式，拨弹电机速度闭环；摩擦轮与拨弹电机各自处理掉线
class Shooter17mm {
  private:
    static constexpr float DT_MIN = 0.0001f, DT_MAX = 0.01f; // 控制周期限幅，s
    static constexpr float HEAT_PER_SHOT = 10;               // 17mm 单发消耗热量，heat/shot

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
        float block_time = 0.25f;            // 卡弹检测时间，s；拨弹 PID 输出持续饱和达到该时间判定卡弹
        float reverse_time = 0.25f;           // 卡弹倒转时间

        // 热量：链路延迟，s；保留热量，heat；降频增益，发/秒/heat
        float heat_delay = 0.3f;
        float heat_reserve = 10;
        float heat_gain = 0.25f;
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
        float ref = 0;       // 目标弹频
        float effective = 0; // 热量限制后的弹频
        float measure = 0;   // 拨弹反馈
    } bullet_freq;

    // 热量，set_heat() 输入，update() 计算
    struct {
        int32_t remain = 0; // 剩余热量，heat；可为负
        float cooling = 0;  // 冷却速率，heat/s
        float target = 0;   // 降频目标热量，heat；降到该值时弹频等于冷却速率对应值
    } heat;

  public:
    // 摩擦轮线速度，m/s
    float rub_speed[2]{};

  private:
    // 拨弹状态机
    enum {
        STOP,
        FEED,
        REVERSE,
    } state = STOP;

    float shoot_output = 0;

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

    // 设置热量状态；剩余热量 heat 过热时为负，冷却速率 heat/s
    void set_heat(int32_t heat_remain, float cooling_rate);

    // 每个控制周期调用
    void update();
};
