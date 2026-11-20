#pragma once

#include "bsp/timer.hpp"
#include "bsp/uart.hpp"
#include "container/fifo.hpp"
#include <vector>

class VT13 {
  public:
    static constexpr float DT_MIN = 0.0001f, DT_MAX = 0.01f; // 键盘斜坡步长限幅，s；避免首次调用和停顿后的大步长

  private:
    static constexpr size_t COMMAND_MAX = 60;  // 0x0311 自定义指令最大长度，B
    static constexpr size_t RX_CAPACITY = 128; // 接收缓冲，B；最长帧为 0x0311 的 69 B，加帧头前乱码余量

  public:
    // 0x0311 回调在串口接收中断内执行，数据仅在回调期间有效；启动前注册
    using RxCallback = std::function<void(const uint8_t*, size_t)>;

    // 运行时可由 app 修改，下次 update() 生效
    struct Config {
        Uart uart;
        float timeout = 0.5f;     // 断联检测超时时间
        float deadline = 0.05f;   // 摇杆死区，小于此值认为是0
        uint16_t mouse_max = 500; // 鼠标xy最大速度

        // 平移：摇杆满量程与合成速度上限，m/s；键盘加速度、松手减速度，m/s^2，INFINITY 为无斜坡
        float vxy_max = 0;
        float axy = 0, dxy = 0;

        // 旋转：拨轮满量程与合成角速度上限，rad/s；键盘角加速度、松手角减速度，rad/s^2，INFINITY 为无斜坡
        float wr_max = 0;
        float ar = 0, dr = 0;

        // 云台：摇杆满量程与合成角速度上限，rad/s
        float yaw_speed_max = 0, pitch_speed_max = 0;

        // 发射：S 挡弹速，m/s；最高弹频，发/秒
        float bullet_speed = 0;
        float bullet_freq = 0;
    } config;

    enum Mode {
        C,
        N,
        S,
        ERR,
    };

    bool is_connect = false; // 连接标志

    // 操控量，update() 计算，断联时为 0
    bool is_enable = false;               // N、S 挡使能
    float vx = 0, vy = 0;                 // 平移速度，m/s；x 前、y 左
    float wr = 0;                         // 旋转角速度，rad/s；逆时针为正
    float yaw_speed = 0, pitch_speed = 0; // 云台角速度，rad/s；yaw 逆时针为正，pitch 抬头为负
    float bullet_speed = 0;               // 弹速，m/s；S 挡开摩擦轮
    float bullet_freq = 0;                // 弹频，发/秒；负值反转拨弹
    bool is_auto_aim = false;             // 自瞄

    // 遥控器
    struct Rc {
        float x = 0, y = 0, pitch = 0, yaw = 0;                         // 摇杆，±1
        float wheel = 0;                                                // 左上角拨轮，±1
        Mode mode = ERR;                                                // 模式切换开关
        bool pause = false, trigger = false, fn = false, photo = false; // 按钮
    } rc;

    // 鼠标
    struct Mouse {
        float yaw = 0, pitch = 0;
        int16_t wheel = 0;
        bool left = false, right = false, middle = false;
    } mouse;

    // 键盘
    struct Key {
        bool w = false, s = false, a = false, d = false;
        bool shift = false, ctrl = false;
        bool q = false, e = false, r = false, f = false, g = false;
        bool z = false, x = false, c = false, v = false, b = false;
    } key;

  private:
    // 通道原始值
    struct __attribute__((packed)) {
        uint8_t sof_1 : 8;   // 0xA9
        uint8_t sof_2 : 8;   // 0x53
        uint16_t ch_0 : 11;  // 右手水平
        uint16_t ch_1 : 11;  // 右手竖直
        uint16_t ch_2 : 11;  // 左手竖直
        uint16_t ch_3 : 11;  // 左手水平
        uint8_t mode_sw : 2; // 挡位切换开关，c - 0，n - 1，s - 2
        bool pause : 1;      // 暂停按键
        bool fn_1 : 1;       // 自定义按键（左）
        bool fn_2 : 1;       // 自定义按键（右）
        uint16_t wheel : 11; // 拨轮
        bool trigger : 1;    // 扳机键

        uint8_t reserved_3bit : 3;

        int16_t mouse_x : 16;     // 鼠标X轴
        int16_t mouse_y : 16;     // 鼠标Y轴
        int16_t mouse_z : 16;     // 鼠标Z轴
        uint8_t mouse_left : 2;   // 鼠标左键
        uint8_t mouse_right : 2;  // 鼠标右键
        uint8_t mouse_middle : 2; // 鼠标中键

        uint8_t reserved_2bit : 2;

        uint16_t key : 16;   // 键盘按键
        uint16_t crc16 : 16; // CRC校验，CRC-16/CCITT-FALSE多项式 P(x) = x^16 + x^12 + x^5 + 1（对应0x1021），初始值0xFFFF，无输入输出反转，无异或
    } raw{};

    float key_vx = 0, key_vy = 0, key_wr = 0; // 键盘斜坡速度
    std::vector<RxCallback> callbacks_0311;
    Fifo<uint8_t, RX_CAPACITY> rx; // 未解析字节，串口接收中断写入
    Timer callback_timer;          // 用于断联检测
    Timer update_timer;

  public:
    VT13(const Config& config);

    void add_0311_callback(RxCallback callback);

    // 每周期调用：断联检测并计算操控量；断联时数据复位为默认值
    void update();

  private:
    // 串口接收回调：遥控帧与图传链路裁判帧混合在同一串口
    void callback(const uint8_t data[], size_t size);

    void parse_raw();

    // 解析摇杆值，归一化到±1
    float get_joy_stick(uint16_t value) const;

    // 按住正向键时负速度清零后加速，反向键同理，松开时减速到 0
    static float get_key_speed(float speed, bool positive, bool negative, float max, float acceleration, float deceleration, float dt);
};
